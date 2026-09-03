using System.Collections.Generic;
using UnityEngine;
using DroneRescue.Environment;
using DroneRescue.Fleet;

namespace DroneRescue.Planning
{
    /// <summary>
    /// The Mission Planner. Phase 4 implements Part 1 Step 1 and Step 2 only:
    /// the patient priority queue, and the hard filter that decides which drones
    /// are even allowed to be considered.
    ///
    /// There is deliberately NO scoring here yet, and nothing is dispatched. Step 3
    /// and Step 4 arrive in Phase 5 and slot in after FilterFeasible, which is why
    /// that method returns a list of evaluations rather than a single winner.
    ///
    /// HUB-AND-SPOKE: everything this class knows about the fleet it reads from
    /// DisasterEnvironment.DroneList, the live shared state each drone republishes
    /// itself into. It never talks to a DroneAgent, never touches a GameObject, and
    /// no drone ever reads another drone through it.
    /// </summary>
    [DisallowMultipleComponent]
    public class MissionPlanner : MonoBehaviour
    {
        [SerializeField] private DisasterEnvironment environment;

        [Tooltip("Log every filter decision, including the rejections. Loud, but it is the Phase 4 checkpoint.")]
        [SerializeField] private bool verbose = true;

        /// <summary>The Part 1 priority queue. Popped in Step 1.</summary>
        public PatientQueue Queue { get; } = new PatientQueue();

        /// <summary>The live DroneList this planner reads. Owned by the environment, not by us.</summary>
        public IReadOnlyList<Drone> DroneList =>
            environment != null ? (IReadOnlyList<Drone>)environment.DroneList : new Drone[0];

        /// <summary>MAX_RANGE for the Step 2 range filter, taken from the active scenario.</summary>
        public float MaxRange => environment != null && environment.Config != null ? environment.Config.maxRange : 0f;

        public DisasterEnvironment Environment => environment;

        /// <summary>Patients already put into the queue, so re-syncing cannot double-queue anyone.</summary>
        private readonly HashSet<Patient> _queued = new HashSet<Patient>();

        private readonly List<DroneEvaluation> _evaluationBuffer = new List<DroneEvaluation>();

        private void Awake()
        {
            if (environment == null)
                environment = FindAnyObjectByType<DisasterEnvironment>();
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
        /// separate thing and belongs to Step 3 in Phase 5.
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
        /// Phase 5's SelectBestDrone scores exactly this list. An empty result means
        /// no drone can serve the patient right now, which is a real outcome and not
        /// an error: the patient stays queued for a later attempt.
        /// </summary>
        public List<DroneEvaluation> FilterFeasible(Patient patient)
        {
            var all = EvaluateFleet(patient);
            var feasible = new List<DroneEvaluation>(all.Count);

            for (int i = 0; i < all.Count; i++)
                if (all[i].Feasible)
                    feasible.Add(all[i]);

            if (verbose)
                LogFilterDecision(patient, all, feasible.Count);

            return feasible;
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

        /// <summary>
        /// One Debug.Log per line, not one multi-line log for the whole table.
        /// Unity's Console list view shows only the first line of an entry, so a
        /// combined log hides every drone verdict until the row is clicked. Separate
        /// calls make the whole filter pass readable without selecting anything.
        /// </summary>
        private void LogFilterDecision(Patient patient, List<DroneEvaluation> all, int feasibleCount)
        {
            float hospitalLeg = all.Count > 0 ? all[0].distanceToHospital : 0f;

            Debug.Log("[Planner] Hard filter for " + patient.id
                      + " [" + patient.priority + "] at " + patient.location
                      + " | MAX_RANGE=" + MaxRange.ToString("F0")
                      + " | hospital leg=" + hospitalLeg.ToString("F1") + "u");

            for (int i = 0; i < all.Count; i++)
                Debug.Log("[Planner]   " + all[i].Explain());

            Debug.Log("[Planner]   -> " + feasibleCount + " of " + all.Count
                      + " drones feasible for " + patient.id + ".");
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
