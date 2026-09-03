using System.Collections.Generic;
using UnityEngine;
using DroneRescue.Environment;
using DroneRescue.Fleet;

namespace DroneRescue.Planning
{
    /// <summary>
    /// The Mission Planner. All four Part 1 steps now live here:
    ///
    ///   Step 1  pop the highest-priority patient          PopNextPatient
    ///   Step 2  hard filter, exclusions not penalties      EvaluateFleet
    ///   Step 3  score the survivors, lowest wins           ScoreFleet
    ///   Step 4  dispatch, fly both legs, release the drone DispatchDrone / AdvanceMissions
    ///
    /// HUB-AND-SPOKE: everything this class knows about the fleet it reads from
    /// DisasterEnvironment.DroneList, the live shared state each drone republishes
    /// itself into. Orders go back down by writing a MissionAssignment onto
    /// DisasterEnvironment.Assignments. It never calls a method on a DroneAgent,
    /// never touches a GameObject, and no drone ever reads another drone through it.
    ///
    /// DYNAMIC EVENTS: the four Part 1 events re-enter the methods above rather
    /// than bringing handling of their own. BatteryLow and DroneFailed both end at
    /// Reassign, which re-runs the same filter and scoring over the fleet as it now
    /// stands. NewEmergency ends at Push, the same door normal detection uses.
    /// FireSpread ends at RepathActiveMissions and deliberately does NOT re-score
    /// anything: a changed obstacle map is a re-path, not a re-decision.
    /// </summary>
    [DisallowMultipleComponent]
    public class MissionPlanner : MonoBehaviour
    {
        [SerializeField] private DisasterEnvironment environment;

        [Tooltip("Only read from, for the collision and simulated-time figures in the run summary.")]
        [SerializeField] private FleetSimulator simulator;

        [Tooltip("Log every filter and scoring decision. Loud, but it is what the checkpoints are read from.")]
        [SerializeField] private bool verbose = true;

        [Header("Step 3 weights. Lowest score wins.")]
        [Tooltip("W1, on totalDistance in world units. The other two factors are 0 to 1 ratios, " +
                 "so W2 and W3 are what convert them into distance-comparable amounts. " +
                 "At the defaults a full tank of battery pressure is worth 60 units of detour " +
                 "and flying into a fire is worth 40.")]
        [SerializeField] private float w1Distance = 1f;

        [SerializeField] private float w2BatteryUtilization = 60f;
        [SerializeField] private float w3Risk = 40f;

        [Tooltip("RiskFactor(location). Ranks PLACES, never patients: see the naming rule in Part 1.")]
        [SerializeField] private RiskModel risk = new RiskModel();

        [Header("Step 4 dispatch")]
        [Tooltip("Run the whole scenario unattended. Turn off only to drive the planner from another script.")]
        [SerializeField] private bool autoRun = true;

        [Tooltip("Below this charge a drone goes Charging instead of Idle when it finishes a rescue.")]
        [SerializeField, Range(0f, 100f)] private float lowBatteryPercent = 25f;

        [Tooltip("Send a drone that finished low on charge to the nearest pad, rather than parking it at the hospital.")]
        [SerializeField] private bool flyToChargerWhenLow = true;

        [Tooltip("Seconds to wait before retrying a patient the whole fleet was infeasible for.")]
        [SerializeField, Min(0.1f)] private float retryInterval = 2f;

        /// <summary>The Part 1 priority queue. Popped in Step 1.</summary>
        public PatientQueue Queue { get; } = new PatientQueue();

        /// <summary>The live DroneList this planner reads. Owned by the environment, not by us.</summary>
        public IReadOnlyList<Drone> DroneList =>
            environment != null ? (IReadOnlyList<Drone>)environment.DroneList : new Drone[0];

        /// <summary>The live dispatch board. Also owned by the environment.</summary>
        public IReadOnlyList<MissionAssignment> Assignments =>
            environment != null ? (IReadOnlyList<MissionAssignment>)environment.Assignments : new MissionAssignment[0];

        /// <summary>MAX_RANGE for the Step 2 range filter, taken from the active scenario.</summary>
        public float MaxRange => environment != null && environment.Config != null ? environment.Config.maxRange : 0f;

        public DisasterEnvironment Environment => environment;

        /// <summary>The most recent winning evaluation. Phase 8's explainability panel reads this.</summary>
        public DroneEvaluation LastDecision { get; private set; }

        /// <summary>Patient the most recent decision was about, or null before the first dispatch.</summary>
        public Patient LastDecisionPatient { get; private set; }

        /// <summary>Dispatches issued this run, reassignments included. Phase 7 metric input.</summary>
        public int DispatchCount { get; private set; }

        /// <summary>Patients already put into the queue, so re-syncing cannot double-queue anyone.</summary>
        private readonly HashSet<Patient> _queued = new HashSet<Patient>();

        private readonly List<DroneEvaluation> _evaluationBuffer = new List<DroneEvaluation>();

        private bool _runStarted;
        private bool _runReported;
        private float _nextDispatchAttemptTime;
        private string _blockedPatientId;

        private void Awake()
        {
            if (environment == null)
                environment = FindAnyObjectByType<DisasterEnvironment>();
            if (simulator == null)
                simulator = FindAnyObjectByType<FleetSimulator>();
        }

        // -----------------------------------------------------------------
        // The autonomous run
        // -----------------------------------------------------------------

        /// <summary>
        /// The whole scenario, unattended: detect, queue, filter, score, dispatch,
        /// fly, deliver, release. Nothing here waits for a key press or an inspector
        /// button, which is the Phase 5 checkpoint.
        /// </summary>
        private void Update()
        {
            if (!autoRun || environment == null || environment.Navigation == null)
                return;

            // The environment registers the fleet in its own Start, so the first
            // frame or two can legitimately have nothing to plan for.
            if (environment.DroneList.Count == 0)
                return;

            if (!_runStarted)
            {
                _runStarted = true;
                OnDisasterDetected();
            }

            SyncQueueFromEnvironment();
            AdvanceMissions();
            DispatchWaitingPatients();
            ReportRunComplete();
        }

        /// <summary>
        /// The earthquake, as far as the planner is concerned: a set of casualties
        /// that now exists in shared state. There is no alert message and no call;
        /// detection already happened when the environment constructed the records.
        /// </summary>
        private void OnDisasterDetected()
        {
            Debug.Log("[Planner] EARTHQUAKE. " + environment.Patients.Count + " casualties detected, "
                      + environment.DroneList.Count + " drones on station, MAX_RANGE="
                      + MaxRange.ToString("F0") + "u.");
            Debug.Log("[Planner] Weights: W1(distance)=" + w1Distance
                      + " W2(batteryUtilization)=" + w2BatteryUtilization
                      + " W3(risk)=" + w3Risk + ". Lowest score wins.");
        }

        // -----------------------------------------------------------------
        // Queue maintenance
        // -----------------------------------------------------------------

        /// <summary>
        /// Pushes any detected-but-not-yet-queued patient into the priority queue.
        ///
        /// Detection and queueing are separate on purpose. A patient exists in shared
        /// state the moment DisasterEnvironment.DetectPatient constructs them; this
        /// is the planner noticing. Phase 6's NewEmergency event goes through the
        /// same Push below rather than a path of its own.
        /// </summary>
        public int SyncQueueFromEnvironment()
        {
            if (environment == null)
                return 0;

            int added = 0;
            foreach (var patient in environment.Patients)
            {
                if (patient.state != PatientState.Waiting)
                    continue;
                if (!_queued.Add(patient))
                    continue;

                Queue.Push(patient);
                added++;
            }

            return added;
        }

        /// <summary>Queues one patient. The single entry point, shared with NewEmergency in Phase 6.</summary>
        public void Push(Patient patient)
        {
            if (patient == null || !_queued.Add(patient))
                return;

            Queue.Push(patient);
            _runReported = false;

            if (verbose)
                Debug.Log("[Planner] New patient detected -> queued: " + patient.id + " [" + patient.priority + "]");
        }

        /// <summary>Step 1: pop the highest-priority patient.</summary>
        public Patient PopNextPatient()
        {
            var patient = Queue.Pop();
            if (patient != null)
                _queued.Remove(patient);
            return patient;
        }

        public Patient PeekNextPatient() => Queue.Peek();

        public void ResetQueue()
        {
            Queue.Clear();
            _queued.Clear();
        }

        // -----------------------------------------------------------------
        // Step 2: the hard filter
        // -----------------------------------------------------------------

        /// <summary>
        /// Measures every drone in the live list against one patient and records
        /// whether it survives the two Part 1 hard filters:
        ///
        ///   drone.status == Idle
        ///   totalDistance(drone, patient) is at most range(drone.batteryPercent)
        ///
        /// Nothing here is scored or weighted. These are exclusions, not penalties:
        /// a drone that fails either test is out of the running entirely, however
        /// attractive it might otherwise look. The continuous battery penalty is a
        /// separate thing and belongs to Step 3 below.
        ///
        /// Returns every drone, rejected ones included, because the checkpoint and
        /// the Phase 8 explainability panel both need to show what was excluded and
        /// why. Use FilterFeasible when only the survivors matter.
        /// </summary>
        public List<DroneEvaluation> EvaluateFleet(Patient patient)
        {
            _evaluationBuffer.Clear();

            if (patient == null || environment == null)
                return new List<DroneEvaluation>();

            Vector3 hospital = environment.NearestHospital(patient.location);
            float maxRange = MaxRange;

            foreach (var drone in environment.DroneList)
            {
                var evaluation = new DroneEvaluation
                {
                    drone = drone,
                    distanceToPatient = FlatDistance(drone.location, patient.location),
                    distanceToHospital = FlatDistance(patient.location, hospital),
                    range = drone.Range(maxRange)
                };

                evaluation.totalDistance = evaluation.distanceToPatient + evaluation.distanceToHospital;

                // Status first: a Busy, Charging or Offline drone is excluded whatever
                // its range, and reporting the status reason is the more useful one.
                if (drone.status != DroneStatus.Idle)
                    evaluation.verdict = FilterVerdict.NotIdle;
                else if (evaluation.totalDistance > evaluation.range)
                    evaluation.verdict = FilterVerdict.OutOfRange;
                else
                    evaluation.verdict = FilterVerdict.Feasible;

                _evaluationBuffer.Add(evaluation);
            }

            return new List<DroneEvaluation>(_evaluationBuffer);
        }

        /// <summary>
        /// The survivors of the hard filter, in DroneList order.
        ///
        /// An empty result means no drone can serve the patient right now, which is a
        /// real outcome and not an error: the patient stays queued for a later attempt.
        /// </summary>
        public List<DroneEvaluation> FilterFeasible(Patient patient)
        {
            var all = EvaluateFleet(patient);
            var feasible = new List<DroneEvaluation>(all.Count);

            for (int i = 0; i < all.Count; i++)
                if (all[i].Feasible)
                    feasible.Add(all[i]);

            if (verbose)
                LogDecisionTable(patient, all, feasible.Count, -1);

            return feasible;
        }

        // -----------------------------------------------------------------
        // Step 3: scoring
        // -----------------------------------------------------------------

        /// <summary>
        /// RiskFactor(location) from Part 1: fire proximity high, clear ground low.
        ///
        /// Computed from the patient's LOCATION and nothing else. The patient's
        /// triage priority is not an input here and never will be; that quantity
        /// orders the queue and this one scores drones.
        /// </summary>
        public float RiskFactor(Vector3 location) =>
            environment != null ? risk.Evaluate(environment.Grid, location) : 0f;

        /// <summary>
        /// Step 3. Scores every drone that survived Step 2 and returns the whole
        /// fleet with the scores attached, plus the index of the winner.
        ///
        ///   batteryUtilization = totalDistance / range
        ///   riskFactor         = RiskFactor(patient.location)
        ///   Score              = W1*totalDistance + W2*batteryUtilization + W3*riskFactor
        ///
        /// Lowest score wins. batteryUtilization IS the battery penalty and it is
        /// continuous: a drone that would finish on fumes is punished in proportion
        /// even though Step 2 already ruled the trip possible.
        ///
        /// riskFactor is identical for every candidate, since it depends on the
        /// patient's location rather than the drone's. It is still computed and
        /// carried per drone so the score stays one readable formula and the
        /// explainability panel has all three factors on every row.
        ///
        /// winnerIndex comes back as -1 when the fleet had no feasible candidate.
        /// </summary>
        public List<DroneEvaluation> ScoreFleet(Patient patient, out int winnerIndex)
        {
            var all = EvaluateFleet(patient);
            winnerIndex = -1;

            if (patient == null)
                return all;

            float riskFactor = RiskFactor(patient.location);
            float bestScore = float.PositiveInfinity;

            for (int i = 0; i < all.Count; i++)
            {
                var evaluation = all[i];
                if (!evaluation.Feasible)
                    continue;

                evaluation.batteryUtilization = evaluation.RangeUsedFraction;
                evaluation.riskFactor = riskFactor;

                evaluation.distanceTerm = w1Distance * evaluation.totalDistance;
                evaluation.batteryTerm = w2BatteryUtilization * evaluation.batteryUtilization;
                evaluation.riskTerm = w3Risk * evaluation.riskFactor;

                evaluation.score = evaluation.distanceTerm + evaluation.batteryTerm + evaluation.riskTerm;
                evaluation.scored = true;

                all[i] = evaluation;

                // Strictly lower, so an exact tie keeps the earlier drone. DroneList
                // order is fixed at registration, which makes ties reproducible.
                if (evaluation.score < bestScore)
                {
                    bestScore = evaluation.score;
                    winnerIndex = i;
                }
            }

            return all;
        }

        /// <summary>
        /// Step 3's answer: the single best drone for this patient, or false when the
        /// hard filter left nothing to choose from.
        /// </summary>
        public bool SelectBestDrone(Patient patient, out DroneEvaluation best)
        {
            var scored = ScoreFleet(patient, out int winnerIndex);

            int feasibleCount = 0;
            for (int i = 0; i < scored.Count; i++)
                if (scored[i].Feasible)
                    feasibleCount++;

            if (verbose)
                LogDecisionTable(patient, scored, feasibleCount, winnerIndex);

            if (winnerIndex < 0)
            {
                best = default;
                return false;
            }

            best = scored[winnerIndex];
            LastDecision = best;
            LastDecisionPatient = patient;
            return true;
        }

        // -----------------------------------------------------------------
        // Step 4: dispatch
        // -----------------------------------------------------------------

        /// <summary>
        /// Works down the queue, dispatching as many patients as the fleet can take
        /// right now.
        ///
        /// Strictly head-of-queue, per Part 1 Step 1. If the most urgent patient has
        /// no feasible drone the loop stops there rather than skipping ahead to an
        /// easier one, because reordering by convenience would quietly undo triage.
        /// The patient stays queued and is retried after retryInterval.
        /// </summary>
        public void DispatchWaitingPatients()
        {
            if (Queue.IsEmpty || Time.time < _nextDispatchAttemptTime)
                return;

            while (!Queue.IsEmpty)
            {
                var patient = Queue.Peek();

                if (!SelectBestDrone(patient, out var best))
                {
                    ReportBlocked(patient);
                    _nextDispatchAttemptTime = Time.time + retryInterval;
                    return;
                }

                PopNextPatient();

                if (DispatchDrone(patient, best) == null)
                {
                    // Route planning failed. Put the patient back and try again
                    // later rather than dropping them.
                    Push(patient);
                    _nextDispatchAttemptTime = Time.time + retryInterval;
                    return;
                }
            }

            _blockedPatientId = null;
        }

        /// <summary>
        /// Step 4. Marks the winning drone Busy and hands the first leg to the
        /// navigation engine.
        ///
        /// The handoff is a write, not a call: the route goes onto the shared
        /// dispatch board and the drone side picks it up. Both legs go through here,
        /// drone to patient now and patient to hospital when the first leg lands, so
        /// there is one dispatch path rather than one per leg.
        ///
        /// Returns null when no route exists between the drone and the patient.
        /// </summary>
        public MissionAssignment DispatchDrone(Patient patient, DroneEvaluation decision)
        {
            if (patient == null || environment == null || decision.drone == null)
                return null;

            var drone = decision.drone;
            var route = environment.Navigation.FindSafePath(drone.location, patient.location);

            if (route.Count == 0)
            {
                Debug.LogWarning("[Planner] No route from " + drone.id + " to " + patient.id
                                 + ". Patient stays queued.");
                return null;
            }

            drone.status = DroneStatus.Busy;
            patient.state = PatientState.Assigned;
            patient.assignedDroneId = drone.id;

            var assignment = new MissionAssignment
            {
                drone = drone,
                droneId = drone.id,
                patient = patient,
                leg = MissionLeg.ToPatient,
                state = MissionState.AwaitingRoute,
                pendingRoute = route,
                legGoal = patient.location,
                arrived = false,
                dispatchedAtTime = Time.time,
                landingSlot = IndexOfDrone(drone),
                decision = decision
            };

            environment.Assignments.Add(assignment);
            DispatchCount++;
            _blockedPatientId = null;

            Debug.Log("[Planner] DISPATCH " + drone.id + " -> " + patient.id
                      + " [" + patient.priority + "] | " + decision.ExplainScore()
                      + " | waypoints=" + route.Count);

            return assignment;
        }

        /// <summary>
        /// Drives every live mission: picks up arrivals, starts the next leg, and
        /// releases the drone when the rescue is done.
        ///
        /// Arrival is read off the shared board, which the drone side writes. The
        /// planner never asks a GameObject whether it got there.
        /// </summary>
        public void AdvanceMissions()
        {
            if (environment == null)
                return;

            var board = environment.Assignments;

            for (int i = 0; i < board.Count; i++)
            {
                var mission = board[i];

                if (mission.IsFinished || mission.state != MissionState.EnRoute || !mission.arrived)
                    continue;

                switch (mission.leg)
                {
                    case MissionLeg.ToPatient:
                        OnPatientReached(mission);
                        break;

                    case MissionLeg.ToHospital:
                        OnPatientDelivered(mission);
                        break;

                    default:
                        // Parked on a charging pad. The drone stays Charging.
                        mission.state = MissionState.Complete;
                        Debug.Log("[Planner] " + mission.droneId + " parked on charging pad.");
                        break;
                }
            }
        }

        private void OnPatientReached(MissionAssignment mission)
        {
            var patient = mission.patient;
            patient.state = PatientState.Reached;
            patient.reachedAtTime = Time.time;

            Debug.Log("[Planner] " + mission.droneId + " reached " + patient.id
                      + " after " + (Time.time - mission.dispatchedAtTime).ToString("F1") + "s.");

            // Second leg: patient to hospital. Each drone aims at its own stand on
            // the landing ring, because a shared centre point makes the first drone
            // to land block the pad for everyone behind it.
            Vector3 hospital = environment.NearestHospital(patient.location);
            Vector3 stand = environment.LandingSlot(hospital, mission.landingSlot, environment.DroneList.Count);

            StartLeg(mission, MissionLeg.ToHospital, stand, "hospital");
        }

        private void OnPatientDelivered(MissionAssignment mission)
        {
            var drone = mission.drone;
            mission.patient.state = PatientState.Delivered;

            Debug.Log("[Planner] " + mission.droneId + " DELIVERED " + mission.patient.id
                      + " to hospital. Battery now " + drone.batteryPercent.ToString("F0") + "%.");

            // Part 1 Step 4: Idle on completion, or Charging if the battery is now low.
            if (drone.batteryPercent < lowBatteryPercent)
            {
                drone.status = DroneStatus.Charging;
                Debug.Log("[Planner] " + drone.id + " battery low at "
                          + drone.batteryPercent.ToString("F0") + "% -> Charging, out of the dispatch pool.");

                if (flyToChargerWhenLow)
                {
                    Vector3 pad = environment.NearestChargingStation(drone.location);
                    Vector3 stand = environment.LandingSlot(pad, mission.landingSlot, environment.DroneList.Count);

                    // The Part 1 range budget covers drone to patient to hospital and
                    // nothing else, so the trip to a pad has to be afforded out of
                    // what is left. Same test as the Step 2 hard filter: fly only if
                    // the remaining range actually reaches. Sending it regardless is
                    // how a drone ends a run stranded at 0 percent.
                    float toPad = FlatDistance(drone.location, stand);
                    float remaining = drone.Range(MaxRange);

                    if (toPad <= remaining)
                    {
                        if (StartLeg(mission, MissionLeg.ToChargingStation, stand, "charging pad"))
                            return;
                    }
                    else
                    {
                        Debug.LogWarning("[Planner] " + drone.id + " cannot reach a charging pad: needs "
                                         + toPad.ToString("F1") + "u but has " + remaining.ToString("F1")
                                         + "u of range. Parking where it landed, still Charging.");
                    }
                }

                mission.state = MissionState.Complete;
                return;
            }

            drone.status = DroneStatus.Idle;
            mission.state = MissionState.Complete;
            Debug.Log("[Planner] " + drone.id + " released -> Idle.");
        }

        /// <summary>
        /// Plans one leg and puts it on the board. Aborts the mission when the
        /// navigation engine cannot reach the goal at all.
        /// </summary>
        private bool StartLeg(MissionAssignment mission, MissionLeg leg, Vector3 goal, string label)
        {
            var route = environment.Navigation.FindSafePath(mission.drone.location, goal);

            if (route.Count == 0)
            {
                Debug.LogWarning("[Planner] No route from " + mission.droneId + " to " + label
                                 + ". Mission aborted, drone released.");
                mission.state = MissionState.Aborted;
                mission.drone.status = DroneStatus.Idle;
                return false;
            }

            mission.leg = leg;
            mission.pendingRoute = route;
            mission.legGoal = goal;
            mission.arrived = false;
            mission.state = MissionState.AwaitingRoute;
            return true;
        }

        // -----------------------------------------------------------------
        // Reassignment. Phase 6's BatteryLow and DroneFailed both end here.
        // -----------------------------------------------------------------

        /// <summary>
        /// Cancels a mission in flight and puts its patient back into the queue, so
        /// the next DispatchWaitingPatients re-runs the SAME filter and scoring over
        /// the fleet as it now stands. There is no separate reassignment algorithm
        /// and there is not meant to be one.
        ///
        /// The drone's status is left alone: the caller owns that. BatteryLow sets
        /// Charging and DroneFailed sets Offline, and either way the drone is out of
        /// the pool because Step 2 excludes anything that is not Idle.
        /// </summary>
        public void Reassign(MissionAssignment mission)
        {
            if (mission == null || mission.IsFinished)
                return;

            mission.state = MissionState.Aborted;
            mission.pendingRoute = null;
            mission.cancelRoute = true;

            var patient = mission.patient;
            if (patient == null || patient.state == PatientState.Delivered)
                return;

            patient.state = PatientState.Waiting;
            patient.assignedDroneId = null;

            Push(patient);

            // Retry immediately: a failed drone should not cost the patient a wait.
            _nextDispatchAttemptTime = 0f;

            Debug.Log("[Planner] REASSIGN " + patient.id + ": " + mission.droneId
                      + " can no longer serve it, patient requeued.");
        }

        /// <summary>The live mission a drone is flying, or null when it has none.</summary>
        public MissionAssignment ActiveMissionOf(string droneId)
        {
            if (environment == null || string.IsNullOrEmpty(droneId))
                return null;

            var board = environment.Assignments;
            for (int i = 0; i < board.Count; i++)
                if (!board[i].IsFinished && board[i].droneId == droneId)
                    return board[i];

            return null;
        }

        // -----------------------------------------------------------------
        // The four Part 1 dynamic events.
        //
        // Every one of these is a handful of lines, and that is the point. None
        // of them plans, scores or routes anything itself. They change one fact
        // about the world and then re-enter the ordinary dispatch machinery,
        // which is what makes the system adapt without a second algorithm to
        // keep in step with the first.
        // -----------------------------------------------------------------

        /// <summary>
        /// BatteryLow(drone): status becomes Charging, and whatever it was carrying
        /// out goes back to the queue.
        ///
        /// Charging is not Idle, so Step 2 excludes this drone from the reassignment
        /// it just triggered. No special case is needed to stop it winning its own
        /// task back.
        /// </summary>
        public void OnBatteryLow(Drone drone)
        {
            if (drone == null)
                return;

            Debug.Log("[Event] BATTERY LOW: " + drone.id + " at "
                      + drone.batteryPercent.ToString("F0") + "% -> Charging.");

            drone.status = DroneStatus.Charging;
            Reassign(ActiveMissionOf(drone.id));
        }

        /// <summary>
        /// DroneFailed(drone): status becomes Offline, and its task is reassigned.
        ///
        /// Identical shape to BatteryLow on purpose. The only difference between a
        /// flat drone and a broken one, as far as planning goes, is the status it
        /// lands in, and Step 2 excludes both the same way.
        /// </summary>
        public void OnDroneFailed(Drone drone)
        {
            if (drone == null)
                return;

            Debug.Log("[Event] DRONE FAILED: " + drone.id + " -> Offline.");

            drone.status = DroneStatus.Offline;
            Reassign(ActiveMissionOf(drone.id));
        }

        /// <summary>
        /// NewEmergency(patient): push onto the priority queue.
        ///
        /// A Critical casualty arriving mid-run goes to the head of the queue and is
        /// dispatched on the next tick, ahead of any Stable patient still waiting.
        /// That reordering is the queue's comparator doing its job, not anything
        /// this method does.
        /// </summary>
        public void OnNewEmergency(Patient patient)
        {
            if (patient == null)
                return;

            Debug.Log("[Event] NEW EMERGENCY: " + patient.id + " [" + patient.priority
                      + "] detected at " + patient.location + ".");

            Push(patient);

            // A patient the fleet was previously infeasible for is no reason to make
            // this one wait out the retry timer.
            _nextDispatchAttemptTime = 0f;
        }

        /// <summary>
        /// The planner's half of FireSpread(zone): re-path every drone that is
        /// currently in the air, around the obstacle map as it now stands.
        ///
        /// NO SCORING RUNS HERE, deliberately. The assignments are still the right
        /// assignments; it is only the routes that have gone stale. Each leg is
        /// re-planned to the same destination it already had, through the same
        /// FindSafePath a first dispatch uses.
        ///
        /// The caller updates the obstacle map before calling this. Rebuilding the
        /// clearance map is the environment's job and happens once per event, not
        /// once per drone.
        /// </summary>
        public int RepathActiveMissions()
        {
            if (environment == null || environment.Navigation == null)
                return 0;

            int repathed = 0;
            var board = environment.Assignments;

            for (int i = 0; i < board.Count; i++)
            {
                var mission = board[i];
                if (mission.IsFinished || mission.drone == null)
                    continue;

                // Re-issuing the same leg re-plans it from where the drone is now.
                if (StartLeg(mission, mission.leg, mission.legGoal, mission.leg.ToString()))
                {
                    repathed++;
                    Debug.Log("[Planner] re-routed " + mission.droneId + " to " + mission.leg
                              + " around the new obstacle map.");
                }
            }

            return repathed;
        }

        // -----------------------------------------------------------------
        // Reporting
        // -----------------------------------------------------------------

        /// <summary>
        /// Logs the run summary once, when the queue is empty and nothing is still
        /// flying. Phase 7 replaces this with the eight metrics and the CSV.
        /// </summary>
        private void ReportRunComplete()
        {
            if (_runReported || DispatchCount == 0 || !Queue.IsEmpty)
                return;

            var board = environment.Assignments;
            for (int i = 0; i < board.Count; i++)
                if (!board[i].IsFinished)
                    return;

            _runReported = true;

            int delivered = 0;
            for (int i = 0; i < environment.Patients.Count; i++)
                if (environment.Patients[i].state == PatientState.Delivered)
                    delivered++;

            float totalBattery = 0f;
            float minBattery = float.PositiveInfinity;
            foreach (var drone in environment.DroneList)
            {
                totalBattery += drone.batteryPercent;
                minBattery = Mathf.Min(minBattery, drone.batteryPercent);
            }

            Debug.Log("[Planner] MISSION COMPLETE. " + delivered + " of " + environment.Patients.Count
                      + " patients delivered in " + Time.time.ToString("F1") + "s"
                      + " | dispatches=" + DispatchCount
                      + " | fleet battery avg=" + (totalBattery / environment.DroneList.Count).ToString("F0") + "%"
                      + " min=" + minBattery.ToString("F0") + "%");

            if (simulator != null)
            {
                var collisions = simulator.Collisions;
                Debug.Log("[Planner] SAFETY. collisions=" + collisions.CollisionCount
                          + " | closest approach=" + collisions.MinSeparation.ToString("F2")
                          + " units of clear air between bodies"
                          + " | simulated time=" + simulator.SimulatedTime.ToString("F1") + "s");
            }
        }

        /// <summary>Reports a head-of-queue stall once, not once per retry.</summary>
        private void ReportBlocked(Patient patient)
        {
            if (patient == null || _blockedPatientId == patient.id)
                return;

            _blockedPatientId = patient.id;
            Debug.LogWarning("[Planner] " + patient.id + " [" + patient.priority
                             + "] has no feasible drone. Holding at the head of the queue, retrying every "
                             + retryInterval.ToString("F0") + "s.");
        }

        private void LogDecisionTable(Patient patient, List<DroneEvaluation> all, int feasibleCount, int winnerIndex)
        {
            // One Debug.Log per line, not one multi-line log for the whole table.
            // Unity's Console list view shows only the first line of an entry, so a
            // combined log hides every drone verdict until the row is clicked.
            float hospitalLeg = all.Count > 0 ? all[0].distanceToHospital : 0f;

            Debug.Log("[Planner] Evaluating " + patient.id
                      + " [" + patient.priority + "] at " + patient.location
                      + " | MAX_RANGE=" + MaxRange.ToString("F0")
                      + " | hospital leg=" + hospitalLeg.ToString("F1") + "u"
                      + " | risk(location)=" + RiskFactor(patient.location).ToString("F2"));

            for (int i = 0; i < all.Count; i++)
                Debug.Log("[Planner]   " + (i == winnerIndex ? "WINNER " : "") + all[i].Explain());

            Debug.Log("[Planner]   -> " + feasibleCount + " of " + all.Count
                      + " drones feasible for " + patient.id + ".");
        }

        // -----------------------------------------------------------------
        // Helpers
        // -----------------------------------------------------------------

        private int IndexOfDrone(Drone drone)
        {
            var list = environment.DroneList;
            for (int i = 0; i < list.Count; i++)
                if (ReferenceEquals(list[i], drone))
                    return i;
            return 0;
        }

        /// <summary>
        /// Distances are measured on the ground plane. Drones fly at a fixed
        /// altitude and patients sit on the ground, so including the vertical gap
        /// would add a constant to every candidate and change nothing except the
        /// readability of the numbers.
        /// </summary>
        private static float FlatDistance(Vector3 a, Vector3 b)
        {
            float dx = a.x - b.x;
            float dz = a.z - b.z;
            return Mathf.Sqrt(dx * dx + dz * dz);
        }

#if UNITY_EDITOR
        /// <summary>Lets editor tooling wire this component up without SerializedObject.</summary>
        public void EditorAssign(DisasterEnvironment env)
        {
            environment = env;
        }
#endif
    }
}
