using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Globalization;
using UnityEngine;
using DroneRescue.Environment;
using DroneRescue.Fleet;
using DroneRescue.Planning;
using DroneRescue.Visualization;
using Debug = UnityEngine.Debug;

namespace DroneRescue.Metrics
{
    /// <summary>
    /// Measures a mission run and writes it out: the eight Part 3 metrics as a CSV
    /// row, the per-casualty detail behind them, and a human-readable event feed.
    ///
    /// OBSERVER, NOT PARTICIPANT. It reads shared state, the planner's public
    /// counters and the simulator's collision tracker, and it writes nothing back
    /// into any of them. Deleting this component changes the numbers a run produces
    /// not at all, which is the only way a measurement is worth anything.
    ///
    /// It also does not ask to be told when the run ends. It polls the planner's
    /// RunComplete flag for a false-to-true transition, so the planner needs no
    /// reference to metrics and no callback to fire.
    ///
    /// THE NUMBERS AND THE FEED COME FROM DIFFERENT PLACES ON PURPOSE. The feed is
    /// assembled from RescueFeed, the same one-way notice board the on-screen
    /// display listens to, because a feed is prose and prose is what that carries.
    /// Every one of the eight metrics is computed from shared state instead. If the
    /// metrics were parsed out of the feed, rewording a log line would change the
    /// results, which is not a property a measurement should have.
    /// </summary>
    [DisallowMultipleComponent]
    public class MissionRecorder : MonoBehaviour
    {
        [SerializeField] private DisasterEnvironment environment;
        [SerializeField] private MissionPlanner planner;
        [SerializeField] private FleetSimulator simulator;

        [Header("Output")]
        [Tooltip("Leave empty for a MissionLogs folder next to Assets in the editor.")]
        [SerializeField] private string outputDirectory = "";

        [Tooltip("Append this run to the summary CSV. Turn off to rehearse without polluting the results file.")]
        [SerializeField] private bool writeSummaryCsv = true;

        [SerializeField] private bool writeEventFeed = true;
        [SerializeField] private bool writePatientCsv = true;

        /// <summary>
        /// Which dispatch rule this run used, read from the planner rather than typed
        /// here.
        ///
        /// It was a serialized string until Phase 9 gave the planner a real toggle,
        /// at which point two places named the mode and only one of them decided it.
        /// A comparison run mislabelled that way is worse than no comparison at all,
        /// because nothing in the output would look wrong.
        /// </summary>
        private string DispatchMode => planner != null ? planner.ModeName : "unknown";

        /// <summary>The metrics for the run that just finished, or null before then.</summary>
        public MissionMetrics LastRun { get; private set; }

        /// <summary>
        /// Simulated seconds the mission has been running, and the figure that stops
        /// when the mission does.
        ///
        /// The simulator's own clock is not this. That one measures the play session
        /// and keeps counting long after every casualty is delivered, which is the
        /// right behaviour for a clock and the wrong number to put in front of anyone
        /// as a mission time. This is maintained beside it so the live display and
        /// the recorded metric are the same measurement rather than two that agree
        /// until the run ends.
        /// </summary>
        public float MissionSeconds { get; private set; }

        /// <summary>The event feed as it stands, newest last. Phase 8's alerts panel reads this.</summary>
        public IReadOnlyList<string> EventFeed => _feed;

        private readonly List<string> _feed = new List<string>();
        private readonly Dictionary<string, float> _startBattery = new Dictionary<string, float>();

        /// <summary>Last charge seen per drone, so a recharge can be spotted as it happens.</summary>
        private readonly Dictionary<string, float> _lastBattery = new Dictionary<string, float>();

        /// <summary>Charge put back into each drone across the run.</summary>
        private readonly Dictionary<string, float> _rechargedBattery = new Dictionary<string, float>();

        private int _rechargeEvents;

        private MissionLogWriter _writer;
        private Stopwatch _stopwatch;
        private string _runId = "";
        private string _startedAtUtc = "";
        private float _startSimTime;
        private bool _started;
        private bool _finished;
        private int _completions;

        private void Awake()
        {
            if (environment == null) environment = FindAnyObjectByType<DisasterEnvironment>();
            if (planner == null) planner = FindAnyObjectByType<MissionPlanner>();
            if (simulator == null) simulator = FindAnyObjectByType<FleetSimulator>();
        }

        private void OnEnable()
        {
            RescueFeed.Alert += OnAlert;
            RescueFeed.Note += OnNote;
        }

        private void OnDisable()
        {
            // RescueFeed is static, so a handler left behind would outlive this
            // component and write into a closed file on the next run.
            RescueFeed.Alert -= OnAlert;
            RescueFeed.Note -= OnNote;

            if (_writer != null)
                _writer.CloseEventFeed();
        }

        private void Update()
        {
            if (planner == null || environment == null)
                return;

            if (!_started && planner.RunStarted)
                BeginRun();

            if (!_started)
                return;

            // Sampled every frame, because a recharge is a jump upward that leaves no
            // trace in the drone record afterwards. Miss it and the run looks like it
            // spent less charge than it did.
            SampleBatteries();

            // Advanced only while the mission is live. Frozen rather than stopped: on
            // the frame the run completes this has already been set to the elapsed
            // time CompleteRun is about to record, so the two never disagree. Work
            // that reopens the run starts it moving again, for the same reason the
            // reopened run gets a second summary row.
            if (!_finished)
                MissionSeconds = Mathf.Max(0f, SimTime() - _startSimTime);

            // A NewEmergency raised after the last mission finished reopens the run:
            // the planner clears its reported flag and starts dispatching again. The
            // work done after that point is part of the mission and has to be
            // measured, so the recorder arms itself again rather than treating the
            // first quiet moment as the end of the story.
            if (_finished && !planner.RunComplete)
            {
                _finished = false;
                _completions++;
                Record("run reopened by new work; a further summary row will follow");
                return;
            }

            if (!_finished && planner.RunComplete)
                CompleteRun();
        }

        // -----------------------------------------------------------------
        // Run boundaries
        // -----------------------------------------------------------------

        private void BeginRun()
        {
            _started = true;
            _startedAtUtc = DateTime.UtcNow.ToString("yyyy-MM-dd HH:mm:ss", CultureInfo.InvariantCulture);
            _runId = DateTime.Now.ToString("yyyyMMdd_HHmmss", CultureInfo.InvariantCulture);
            _startSimTime = SimTime();
            MissionSeconds = 0f;

            // Ported from Stage 1, which timed its one mission the same way. The
            // wall clock is not the mission's own clock, so it is recorded beside
            // the simulated time rather than instead of it.
            _stopwatch = Stopwatch.StartNew();

            _writer = new MissionLogWriter(outputDirectory, _runId);

            // Starting charge has to be captured now. Nothing stores it, and by the
            // end of the run the only battery figures left are the spent ones.
            _startBattery.Clear();
            _lastBattery.Clear();
            _rechargedBattery.Clear();
            _rechargeEvents = 0;

            foreach (var drone in environment.DroneList)
            {
                _startBattery[drone.id] = drone.batteryPercent;
                _lastBattery[drone.id] = drone.batteryPercent;
                _rechargedBattery[drone.id] = 0f;
            }

            Record("run started  ·  " + environment.DroneList.Count + " drones, "
                   + environment.Patients.Count + " casualties detected  ·  mode " + DispatchMode);
        }

        private void CompleteRun()
        {
            _finished = true;

            if (_stopwatch != null)
                _stopwatch.Stop();

            LastRun = Capture();

            // A reopened run gets its own row rather than overwriting the first, so
            // the file keeps both the state at the original completion and the state
            // after the extra work. Suffixed so the two are never mistaken for two
            // separate runs of the scenario.
            if (_completions > 0)
                LastRun.runId = _runId + "-" + (_completions + 1);

            Debug.Log("[Metrics] MISSION METRICS\n" + LastRun.ToReadableBlock());

            // The battery figures check themselves. They are the one group here that
            // is not a single reading, and the last time they disagreed there was
            // nothing in the output to say so.
            if (Mathf.Abs(LastRun.BatteryResidual) > 0.5f)
                Debug.LogWarning("[Metrics] Battery figures do not balance: start + recharged - spent - remaining = "
                                 + LastRun.BatteryResidual.ToString("F2", CultureInfo.InvariantCulture)
                                 + ", expected 0.");

            if (writeEventFeed && _writer != null)
            {
                _writer.AppendEvent("");
                _writer.AppendEvent(LastRun.ToReadableBlock());
            }

            bool summaryOk = writeSummaryCsv && _writer != null && _writer.AppendSummaryRow(LastRun);
            bool patientsOk = writePatientCsv && _writer != null && WritePatientDetail();

            if (summaryOk)
                Debug.Log("[Metrics] Run appended to " + _writer.SummaryPath);
            if (patientsOk)
                Debug.Log("[Metrics] Per-casualty detail written to " + _writer.PatientCsvPath);
            if (writeEventFeed && _writer != null)
                Debug.Log("[Metrics] Event feed written to " + _writer.EventLogPath);

            if (_writer != null)
                _writer.CloseEventFeed();
        }

        // -----------------------------------------------------------------
        // The eight metrics
        // -----------------------------------------------------------------

        /// <summary>
        /// Computes every Part 3 metric from shared state. Safe to call at any
        /// point; the numbers are simply partial before the run ends, which is what
        /// Phase 8's live dashboard will want.
        /// </summary>
        public MissionMetrics Capture()
        {
            var metrics = new MissionMetrics
            {
                runId = _runId,
                startedAtUtc = _startedAtUtc,
                scenarioName = environment.Config != null ? environment.Config.name : "unknown",
                dispatchMode = DispatchMode,
                droneCount = environment.DroneList.Count,
                patientCount = environment.Patients.Count,
                dispatchCount = planner.DispatchCount,
                reassignmentCount = planner.ReassignCount
            };

            // 1. Mission completion time.
            metrics.missionTimeMs = _stopwatch != null ? _stopwatch.Elapsed.TotalMilliseconds : 0.0;
            metrics.missionTimeSeconds = Mathf.Max(0f, SimTime() - _startSimTime);

            // 2. Average response time: queue entry to drone arrival, per casualty.
            float responseTotal = 0f;
            int responseCount = 0;
            float worstResponse = 0f;

            // 3 and 7. Delivered, and reached regardless of delivery.
            int delivered = 0;
            int reached = 0;

            foreach (var patient in environment.Patients)
            {
                if (patient.state == PatientState.Delivered)
                    delivered++;

                // reachedAtTime is stamped when a drone arrives, and survives a
                // later reassignment, so it is the right test for coverage: it asks
                // whether help got there, not whether the rescue then completed.
                if (patient.reachedAtTime >= 0f)
                {
                    reached++;

                    float response = Mathf.Max(0f, patient.reachedAtTime - patient.queuedAtTime);
                    responseTotal += response;
                    responseCount++;
                    worstResponse = Mathf.Max(worstResponse, response);
                }
            }

            metrics.responseSamples = responseCount;
            metrics.averageResponseSeconds = responseCount > 0 ? responseTotal / responseCount : 0f;
            metrics.worstResponseSeconds = worstResponse;

            metrics.deliveredCount = delivered;
            metrics.reachedCount = reached;
            metrics.rescueSuccessRatePercent = Percent(delivered, metrics.patientCount);
            metrics.coveragePercent = Percent(reached, metrics.patientCount);

            // 4. Battery at mission end.
            //
            // Four figures rather than two, because two do not reconcile on their
            // own. The fleet does not start on a full charge: the scenario authors a
            // different starting percentage per drone, so "23% left" and "64% spent"
            // sum to the fleet's actual starting average and not to 100. Recording
            // that starting average is what makes the pair readable.
            //
            // Spend is derived rather than differenced, for the same reason: with
            // recharging in play, start minus remaining is not what a drone burned.
            // A drone that started at 100, ran down to 3, recharged and finished on
            // 88 spent 109 points, not 12.
            SampleBatteries();

            float startTotal = 0f;
            float remainingTotal = 0f;
            float rechargedTotal = 0f;
            float lowest = float.PositiveInfinity;

            foreach (var drone in environment.DroneList)
            {
                remainingTotal += drone.batteryPercent;
                lowest = Mathf.Min(lowest, drone.batteryPercent);

                float start;
                startTotal += _startBattery.TryGetValue(drone.id, out start) ? start : drone.batteryPercent;

                float recharged;
                rechargedTotal += _rechargedBattery.TryGetValue(drone.id, out recharged) ? recharged : 0f;
            }

            metrics.rechargeCount = _rechargeEvents;

            if (metrics.droneCount > 0)
            {
                metrics.fleetAvgBatteryStartPercent = startTotal / metrics.droneCount;
                metrics.fleetAvgBatteryRemainingPercent = remainingTotal / metrics.droneCount;
                metrics.fleetAvgBatteryRechargedPercent = rechargedTotal / metrics.droneCount;
                metrics.fleetAvgBatteryUsedPercent =
                    (startTotal + rechargedTotal - remainingTotal) / metrics.droneCount;
                metrics.minBatteryRemainingPercent = lowest;
            }

            // 5. Collisions.
            if (simulator != null)
            {
                metrics.collisionCount = simulator.Collisions.CollisionCount;
                metrics.minSeparation = simulator.Collisions.MinSeparation;
            }
            else
            {
                metrics.minSeparation = float.PositiveInfinity;
            }

            // 8. Throughput, per simulated minute rather than per second: per second
            // this scenario reads 0.16 and tells nobody anything.
            metrics.throughputPerMinute = metrics.missionTimeSeconds > 0.001f
                ? delivered / (metrics.missionTimeSeconds / 60f)
                : 0f;

            return metrics;
        }

        /// <summary>
        /// Notes any charge that has appeared in a drone since the last look.
        ///
        /// Battery only ever falls while flying, so a rise can only be a recharge.
        /// The threshold keeps floating-point noise out of the total; a real recharge
        /// arrives as a jump of tens of points.
        /// </summary>
        private void SampleBatteries()
        {
            foreach (var drone in environment.DroneList)
            {
                float previous;
                if (!_lastBattery.TryGetValue(drone.id, out previous))
                {
                    // A drone that appeared after the run began. Counted from here.
                    _startBattery[drone.id] = drone.batteryPercent;
                    _lastBattery[drone.id] = drone.batteryPercent;
                    _rechargedBattery[drone.id] = 0f;
                    continue;
                }

                float gained = drone.batteryPercent - previous;
                if (gained > 0.5f)
                {
                    float already;
                    _rechargedBattery.TryGetValue(drone.id, out already);
                    _rechargedBattery[drone.id] = already + gained;
                    _rechargeEvents++;
                }

                _lastBattery[drone.id] = drone.batteryPercent;
            }
        }

        private bool WritePatientDetail()
        {
            var c = CultureInfo.InvariantCulture;
            var rows = new List<string>();

            foreach (var patient in environment.Patients)
            {
                float response = patient.reachedAtTime >= 0f
                    ? Mathf.Max(0f, patient.reachedAtTime - patient.queuedAtTime)
                    : -1f;

                rows.Add(string.Join(",", new[]
                {
                    _runId,
                    patient.id,
                    patient.priority.ToString(),
                    patient.state.ToString(),
                    string.IsNullOrEmpty(patient.assignedDroneId) ? "" : patient.assignedDroneId,
                    patient.queuedAtTime.ToString("F2", c),
                    patient.reachedAtTime >= 0f ? patient.reachedAtTime.ToString("F2", c) : "",
                    patient.deliveredAtTime >= 0f ? patient.deliveredAtTime.ToString("F2", c) : "",
                    response >= 0f ? response.ToString("F2", c) : ""
                }));
            }

            return _writer.WritePatientRows(
                "RunId,PatientId,Priority,FinalState,LastDroneId,QueuedAtSeconds,ReachedAtSeconds,DeliveredAtSeconds,ResponseSeconds",
                rows);
        }

        // -----------------------------------------------------------------
        // Event feed
        // -----------------------------------------------------------------

        private void OnAlert(AlertKind kind, string message)
        {
            Record("[" + kind.ToString().ToUpperInvariant() + "] " + message);
        }

        private void OnNote(string message)
        {
            Record(message);
        }

        /// <summary>Stamps one line with the simulated clock, keeps it, and streams it to disk.</summary>
        private void Record(string message)
        {
            string line = SimTime().ToString("F1", CultureInfo.InvariantCulture).PadLeft(7) + "s   " + message;
            _feed.Add(line);

            if (writeEventFeed && _writer != null)
                _writer.AppendEvent(line);
        }

        private float SimTime()
        {
            return simulator != null ? simulator.SimulatedTime : Time.timeSinceLevelLoad;
        }

        private static float Percent(int part, int whole) => whole > 0 ? part * 100f / whole : 0f;

#if UNITY_EDITOR
        public void EditorAssign(DisasterEnvironment env, MissionPlanner missionPlanner, FleetSimulator fleet)
        {
            environment = env;
            planner = missionPlanner;
            simulator = fleet;
        }
#endif
    }
}
