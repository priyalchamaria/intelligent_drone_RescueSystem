using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Text;
using UnityEngine;
using DroneRescue.Environment;
using DroneRescue.Fleet;
using DroneRescue.Metrics;
using DroneRescue.Navigation;
using DroneRescue.Planning;
using DroneRescue.Visualization;

namespace DroneRescue.Dashboard
{
    /// <summary>
    /// Publishes the state of a running mission to a JSON file for the web
    /// dashboard to poll.
    ///
    /// READ ONLY, LIKE THE OTHER OBSERVERS. It reads shared state, the planner's
    /// public surface and the recorder's finished metrics, and writes to none of
    /// them. Switching it off changes what a run looks like from outside and nothing
    /// about what the run does. That matters more here than it does for the
    /// on-screen display: a dashboard is the piece most likely to be left running
    /// during a measured comparison, and it must not be able to affect one.
    ///
    /// A FILE, NOT A SOCKET. Part 4 asks for a file the browser polls, and that is
    /// the right shape for this: no server inside Unity, no connection to lose, no
    /// second thing to start in the right order during a demo, and a run leaves a
    /// readable artefact behind either way. The cost is latency of about one publish
    /// interval, which nobody watching a drone cross a map can see.
    ///
    /// ONE DOCUMENT, NOT SEVERAL. Fleet, queue, alerts, the last decision and the
    /// finished metrics all go into the same file. The browser then renders one
    /// consistent snapshot per poll instead of four files that can disagree about
    /// what time it is, and a half-written file can never be read because the
    /// document is written to a temporary path and swapped into place.
    /// </summary>
    [DisallowMultipleComponent]
    public class DashboardStateWriter : MonoBehaviour
    {
        [SerializeField] private DisasterEnvironment environment;
        [SerializeField] private MissionPlanner planner;
        [SerializeField] private FleetSimulator simulator;
        [SerializeField] private MissionRecorder recorder;

        [Header("Output")]
        [Tooltip("Leave empty for a Dashboard/data folder next to Assets in the editor.")]
        [SerializeField] private string outputDirectory = "";

        [Tooltip("Seconds between publishes. Part 4 asks for 200 to 500ms.")]
        [SerializeField, Range(0.05f, 2f)] private float publishInterval = 0.3f;

        [Tooltip("How many feed lines the alerts panel keeps. Older lines fall off the top.")]
        [SerializeField, Min(4)] private int feedLength = 40;

        [Tooltip("Turn off to leave the last published file untouched, e.g. to inspect it after a run.")]
        [SerializeField] private bool publish = true;

        public const string StateFileName = "state.json";

        /// <summary>Where the last publish went. Shown in the console once, so the page is easy to point at.</summary>
        public string StatePath => Path.Combine(Directory, StateFileName);

        public string Directory => string.IsNullOrEmpty(outputDirectory) ? DefaultDirectory() : outputDirectory;

        private static readonly Encoding Utf8NoBom = new UTF8Encoding(false);
        private static readonly CultureInfo Inv = CultureInfo.InvariantCulture;

        private readonly List<FeedLine> _feed = new List<FeedLine>();
        private readonly StringBuilder _json = new StringBuilder(8192);

        private float _nextPublishTime;
        private bool _announced;
        private bool _failed;
        private bool _wasComplete;
        private int _consecutiveFailures;

        /// <summary>Consecutive failed publishes before the writer gives up. About six seconds' worth.</summary>
        private const int FailuresBeforeGivingUp = 20;

        /// <summary>One line of the alerts panel: the four Part 1 events are marked, notes are not.</summary>
        private struct FeedLine
        {
            public float time;
            public string kind;
            public string text;
        }

        private void Awake()
        {
            if (environment == null) environment = FindAnyObjectByType<DisasterEnvironment>();
            if (planner == null) planner = FindAnyObjectByType<MissionPlanner>();
            if (simulator == null) simulator = FindAnyObjectByType<FleetSimulator>();
            if (recorder == null) recorder = FindAnyObjectByType<MissionRecorder>();
        }

        private void OnEnable()
        {
            RescueFeed.Alert += OnAlert;
            RescueFeed.Note += OnNote;
        }

        private void OnDisable()
        {
            // RescueFeed is static: a handler left behind would outlive this component
            // and keep filling a feed nobody reads.
            RescueFeed.Alert -= OnAlert;
            RescueFeed.Note -= OnNote;
        }

        private void OnAlert(AlertKind kind, string message) => Append(kind.ToString(), message);

        private void OnNote(string message) => Append(null, message);

        private void Append(string kind, string message)
        {
            _feed.Add(new FeedLine { time = SimTime(), kind = kind, text = message });

            if (_feed.Count > feedLength)
                _feed.RemoveRange(0, _feed.Count - feedLength);
        }

        private void LateUpdate()
        {
            if (!publish || _failed || environment == null || planner == null)
                return;

            // Published on a fixed interval rather than every frame, and once more the
            // moment a run finishes so the analytics view never has to wait out an
            // interval to show a result that already exists.
            bool complete = planner.RunComplete;
            bool justFinished = complete && !_wasComplete;
            _wasComplete = complete;

            if (!justFinished && Time.time < _nextPublishTime)
                return;

            _nextPublishTime = Time.time + publishInterval;
            Publish();
        }

        // -----------------------------------------------------------------
        // The document
        // -----------------------------------------------------------------

        private void Publish()
        {
            _json.Length = 0;
            _json.Append('{');

            Field("version", 2);
            Comma();
            Field("generatedAt", DateTime.Now.ToString("HH:mm:ss", Inv));
            Comma();
            Field("simTime", SimTime());
            Comma();
            Field("missionSeconds", MissionSeconds());
            Comma();
            Field("playing", Application.isPlaying);
            Comma();
            Field("runStarted", planner.RunStarted);
            Comma();
            Field("runComplete", planner.RunComplete);
            Comma();
            Field("scenario", environment.Config != null ? environment.Config.name : "unknown");
            Comma();
            Field("maxRange", planner.MaxRange);
            Comma();
            Field("dispatches", planner.DispatchCount);
            Comma();
            Field("reassignments", planner.ReassignCount);

            Comma();
            WriteWorld();
            Comma();
            WriteFleet();
            Comma();
            WriteQueue();
            Comma();
            WritePatients();
            Comma();
            WriteFeed();
            Comma();
            WriteExplain();
            Comma();
            WriteAnalytics();

            _json.Append('}');

            Write(_json.ToString());
        }

        /// <summary>
        /// The static map: bounds, the two kinds of site, and every obstacle.
        ///
        /// Republished every time rather than written once, because the obstacle list
        /// is not static after all: a FireSpread event adds to it mid-run, and a
        /// dashboard that drew the map once would keep showing drones detouring
        /// around nothing.
        /// </summary>
        private void WriteWorld()
        {
            var config = environment.Config;

            Key("world");
            _json.Append('{');

            if (config != null)
            {
                Field("originX", config.gridOrigin.x);
                Comma();
                Field("originZ", config.gridOrigin.z);
                Comma();
                Field("width", config.gridWidth * config.cellSize);
                Comma();
                Field("height", config.gridHeight * config.cellSize);
                Comma();

                Key("hospitals");
                _json.Append('[');
                for (int i = 0; i < config.hospitals.Count; i++)
                {
                    if (i > 0) Comma();
                    WritePoint(config.hospitals[i]);
                }
                _json.Append(']');
                Comma();

                Key("chargingStations");
                _json.Append('[');
                for (int i = 0; i < config.chargingStations.Count; i++)
                {
                    if (i > 0) Comma();
                    WritePoint(config.chargingStations[i]);
                }
                _json.Append(']');
                Comma();
            }

            Key("obstacles");
            _json.Append('[');

            var obstacles = environment.Grid != null ? environment.Grid.Obstacles : null;
            if (obstacles != null)
            {
                for (int i = 0; i < obstacles.Count; i++)
                {
                    if (i > 0) Comma();
                    WriteObstacle(obstacles[i]);
                }
            }

            _json.Append(']');
            _json.Append('}');
        }

        private void WriteObstacle(Obstacle obstacle)
        {
            _json.Append('{');
            Field("circle", obstacle.shape == ObstacleShape.Circle);
            Comma();
            Field("x", obstacle.center.x);
            Comma();
            Field("z", obstacle.center.z);
            Comma();
            Field("r", obstacle.radius);
            Comma();
            Field("hx", obstacle.halfExtents.x);
            Comma();
            Field("hz", obstacle.halfExtents.z);
            Comma();
            Field("fire", obstacle.isDynamic);
            _json.Append('}');
        }

        private void WriteFleet()
        {
            Key("fleet");
            _json.Append('[');

            var fleet = environment.DroneList;
            for (int i = 0; i < fleet.Count; i++)
            {
                if (i > 0) Comma();

                var drone = fleet[i];
                var mission = planner.ActiveMissionOf(drone.id);

                _json.Append('{');
                Field("id", drone.id);
                Comma();
                Field("status", drone.status.ToString());
                Comma();
                Field("battery", drone.batteryPercent);
                Comma();
                Field("range", drone.Range(planner.MaxRange));
                Comma();
                Field("x", drone.location.x);
                Comma();
                Field("z", drone.location.z);
                Comma();
                Field("task", mission != null && mission.patient != null ? mission.patient.id : null);
                Comma();
                Field("leg", mission != null ? mission.leg.ToString() : null);
                _json.Append('}');
            }

            _json.Append(']');
        }

        /// <summary>
        /// The priority queue in the order it will actually be served.
        ///
        /// ToOrderedList, not the raw heap array. A binary heap is only partially
        /// ordered, so printing its backing array would show a queue that disagrees
        /// with the order patients are then dispatched in, which is exactly the thing
        /// this panel exists to make visible.
        /// </summary>
        private void WriteQueue()
        {
            Key("queue");
            _json.Append('[');

            var ordered = planner.Queue.ToOrderedList();
            for (int i = 0; i < ordered.Count; i++)
            {
                if (i > 0) Comma();

                var patient = ordered[i];
                _json.Append('{');
                Field("id", patient.id);
                Comma();
                Field("priority", patient.priority.ToString());
                Comma();
                Field("waiting", Mathf.Max(0f, SimTime() - patient.queuedAtTime));
                Comma();
                Field("x", patient.location.x);
                Comma();
                Field("z", patient.location.z);
                _json.Append('}');
            }

            _json.Append(']');
        }

        private void WritePatients()
        {
            Key("patients");
            _json.Append('[');

            var patients = environment.Patients;
            for (int i = 0; i < patients.Count; i++)
            {
                if (i > 0) Comma();

                var patient = patients[i];
                _json.Append('{');
                Field("id", patient.id);
                Comma();
                Field("priority", patient.priority.ToString());
                Comma();
                Field("state", patient.state.ToString());
                Comma();
                Field("x", patient.location.x);
                Comma();
                Field("z", patient.location.z);
                Comma();
                Field("drone", patient.assignedDroneId);
                Comma();
                Field("queuedAt", patient.queuedAtTime);
                Comma();
                Field("reachedAt", patient.reachedAtTime);
                Comma();
                Field("deliveredAt", patient.deliveredAtTime);
                Comma();
                Field("response", patient.reachedAtTime >= 0f
                    ? Mathf.Max(0f, patient.reachedAtTime - patient.queuedAtTime)
                    : -1f);
                _json.Append('}');
            }

            _json.Append(']');
        }

        private void WriteFeed()
        {
            Key("feed");
            _json.Append('[');

            for (int i = 0; i < _feed.Count; i++)
            {
                if (i > 0) Comma();

                _json.Append('{');
                Field("t", _feed[i].time);
                Comma();
                Field("kind", _feed[i].kind);
                Comma();
                Field("text", _feed[i].text);
                _json.Append('}');
            }

            _json.Append(']');
        }

        /// <summary>
        /// The most recent dispatch decision, with every drone that was measured for
        /// it and the three factors behind each score.
        /// </summary>
        private void WriteExplain()
        {
            Key("explain");

            var rows = planner.LastEvaluations;
            if (planner.LastDecisionPatient == null || rows == null || rows.Count == 0)
            {
                _json.Append("null");
                return;
            }

            var winner = planner.LastDecision;

            _json.Append('{');
            Field("patient", planner.LastDecisionPatient.id);
            Comma();
            Field("priority", planner.LastDecisionPatient.priority.ToString());
            Comma();
            Field("at", planner.LastDecisionTime);
            Comma();
            Field("winner", winner.drone != null ? winner.drone.id : null);
            Comma();
            Field("w1", planner.W1Distance);
            Comma();
            Field("w2", planner.W2BatteryUtilization);
            Comma();
            Field("w3", planner.W3Risk);
            Comma();

            Key("rows");
            _json.Append('[');

            for (int i = 0; i < rows.Count; i++)
            {
                if (i > 0) Comma();

                var row = rows[i];
                _json.Append('{');
                Field("id", row.drone != null ? row.drone.id : "?");
                Comma();
                Field("verdict", row.verdict.ToString());
                Comma();
                Field("scored", row.scored);
                Comma();
                Field("winner", winner.drone != null && row.drone != null
                                && ReferenceEquals(row.drone, winner.drone));
                Comma();
                Field("toPatient", row.distanceToPatient);
                Comma();
                Field("toHospital", row.distanceToHospital);
                Comma();
                Field("total", row.totalDistance);
                Comma();
                Field("range", row.range);
                Comma();
                Field("battUtil", row.batteryUtilization);
                Comma();
                Field("risk", row.riskFactor);
                Comma();
                Field("distTerm", row.distanceTerm);
                Comma();
                Field("battTerm", row.batteryTerm);
                Comma();
                Field("riskTerm", row.riskTerm);
                Comma();
                Field("score", row.score);
                _json.Append('}');
            }

            _json.Append(']');
            _json.Append('}');
        }

        /// <summary>
        /// Phase 7's eight metrics, once a run has produced them.
        ///
        /// Taken from the recorder rather than recomputed here. Two places computing
        /// the same eight numbers is two places for them to disagree, and the CSV is
        /// the one that gets handed in.
        /// </summary>
        private void WriteAnalytics()
        {
            Key("analytics");

            var run = recorder != null ? recorder.LastRun : null;
            if (run == null)
            {
                _json.Append("null");
                return;
            }

            _json.Append('{');
            Field("runId", run.runId);
            Comma();
            Field("scenario", run.scenarioName);
            Comma();
            Field("dispatchMode", run.dispatchMode);
            Comma();
            Field("missionSeconds", run.missionTimeSeconds);
            Comma();
            Field("missionMs", (float)run.missionTimeMs);
            Comma();
            Field("avgResponse", run.averageResponseSeconds);
            Comma();
            Field("worstResponse", run.worstResponseSeconds);
            Comma();
            Field("responseSamples", run.responseSamples);
            Comma();
            Field("delivered", run.deliveredCount);
            Comma();
            Field("patients", run.patientCount);
            Comma();
            Field("successRate", run.rescueSuccessRatePercent);
            Comma();
            Field("batteryStart", run.fleetAvgBatteryStartPercent);
            Comma();
            Field("batteryRemaining", run.fleetAvgBatteryRemainingPercent);
            Comma();
            Field("batteryMin", run.minBatteryRemainingPercent);
            Comma();
            Field("batteryUsed", run.fleetAvgBatteryUsedPercent);
            Comma();
            Field("batteryRecharged", run.fleetAvgBatteryRechargedPercent);
            Comma();
            Field("recharges", run.rechargeCount);
            Comma();
            Field("collisions", run.collisionCount);
            Comma();
            Field("minSeparation", float.IsInfinity(run.minSeparation) || float.IsNaN(run.minSeparation)
                ? -1f : run.minSeparation);
            Comma();
            Field("reassignments", run.reassignmentCount);
            Comma();
            Field("dispatches", run.dispatchCount);
            Comma();
            Field("reached", run.reachedCount);
            Comma();
            Field("coverage", run.coveragePercent);
            Comma();
            Field("throughput", run.throughputPerMinute);
            _json.Append('}');
        }

        // -----------------------------------------------------------------
        // Writing it out
        // -----------------------------------------------------------------

        /// <summary>
        /// Writes the document to a temporary file and swaps it into place.
        ///
        /// The browser polls this file several times a second, so a plain overwrite
        /// would eventually be read halfway through and fail to parse. Replacing the
        /// file instead means a reader either gets the previous document or the next
        /// one, never half of either.
        ///
        /// THE SWAP IS ALLOWED TO FAIL. On Windows the replace cannot delete the old
        /// file while the web server has it open, which happens whenever a poll and a
        /// publish land on the same instant. That is not an error, it is two
        /// processes doing exactly what they should, a few times a minute. A failed
        /// swap falls back to writing the destination directly, and a failure at that
        /// too simply skips the tick: another publish is 300ms away, and the reader
        /// keeps the previous document until then. Only a fault that persists for
        /// several seconds is a real one, and only that is reported.
        /// </summary>
        private void Write(string payload)
        {
            string destination = null;

            try
            {
                string directory = Directory;
                if (!System.IO.Directory.Exists(directory))
                    System.IO.Directory.CreateDirectory(directory);

                destination = Path.Combine(directory, StateFileName);
                string temporary = destination + ".tmp";

                File.WriteAllText(temporary, payload, Utf8NoBom);

                try
                {
                    if (File.Exists(destination))
                        File.Replace(temporary, destination, null);
                    else
                        File.Move(temporary, destination);
                }
                catch (IOException)
                {
                    // Locked mid-swap. Writing straight over it gives up atomicity for
                    // this one publish, which costs at worst a single unparsed poll
                    // that the next one corrects.
                    File.WriteAllText(destination, payload, Utf8NoBom);
                }

                _consecutiveFailures = 0;

                if (!_announced)
                {
                    _announced = true;
                    Debug.Log("[Dashboard] Publishing to " + destination
                              + " - serve the Dashboard folder and open index.html");
                }
            }
            catch (Exception ex)
            {
                _consecutiveFailures++;

                // Roughly six seconds of nothing but failure. Below that it is
                // contention and it clears itself; above it, something is actually
                // wrong and publishing three errors a second would bury the console.
                if (_consecutiveFailures >= FailuresBeforeGivingUp)
                {
                    _failed = true;
                    Debug.LogError("[Dashboard] Publishing disabled after "
                                   + _consecutiveFailures + " consecutive failures writing "
                                   + (destination ?? Directory) + ": " + ex.Message);
                }
            }
        }

        private static string DefaultDirectory()
        {
#if UNITY_EDITOR
            var projectRoot = System.IO.Directory.GetParent(Application.dataPath);
            if (projectRoot != null)
                return Path.Combine(Path.Combine(projectRoot.FullName, "Dashboard"), "data");
#endif
            return Path.Combine(Application.persistentDataPath, "DashboardData");
        }

        private float SimTime() => simulator != null ? simulator.SimulatedTime : Time.timeSinceLevelLoad;

        /// <summary>
        /// The mission's own clock, which stops when the mission does.
        ///
        /// Taken from the recorder so the header and the analytics tab show one
        /// number rather than two that drift apart the moment a run ends. Without a
        /// recorder there is nothing keeping that clock, and the session clock is
        /// published instead: wrong after the run finishes, but present, which is the
        /// better of the two failures for a component that is only an observer.
        /// </summary>
        private float MissionSeconds() => recorder != null ? recorder.MissionSeconds : SimTime();

        // -----------------------------------------------------------------
        // A very small JSON writer
        //
        // Hand-written because JsonUtility cannot serialise the shapes this
        // document needs (dictionaries, nulls, arrays of mixed records) and
        // pulling in a JSON library for one file would be a heavier dependency
        // than the thirty lines below.
        // -----------------------------------------------------------------

        private void Comma() => _json.Append(',');

        private void Key(string name)
        {
            _json.Append('"').Append(name).Append("\":");
        }

        private void Field(string name, string value)
        {
            Key(name);
            if (value == null)
                _json.Append("null");
            else
                Escape(value);
        }

        private void Field(string name, bool value)
        {
            Key(name);
            _json.Append(value ? "true" : "false");
        }

        private void Field(string name, int value)
        {
            Key(name);
            _json.Append(value.ToString(Inv));
        }

        private void Field(string name, float value)
        {
            Key(name);

            // Infinity and NaN are not JSON, and a parse failure in the browser would
            // blank the whole page over one unusable number.
            if (float.IsNaN(value) || float.IsInfinity(value))
                _json.Append("null");
            else
                _json.Append(value.ToString("0.###", Inv));
        }

        private void WritePoint(Vector3 point)
        {
            _json.Append('{');
            Field("x", point.x);
            Comma();
            Field("z", point.z);
            _json.Append('}');
        }

        private void Escape(string value)
        {
            _json.Append('"');

            for (int i = 0; i < value.Length; i++)
            {
                char c = value[i];
                switch (c)
                {
                    case '"': _json.Append("\\\""); break;
                    case '\\': _json.Append("\\\\"); break;
                    case '\n': _json.Append("\\n"); break;
                    case '\r': _json.Append("\\r"); break;
                    case '\t': _json.Append("\\t"); break;
                    default:
                        if (c < ' ')
                            _json.Append("\\u").Append(((int)c).ToString("X4", Inv));
                        else
                            _json.Append(c);
                        break;
                }
            }

            _json.Append('"');
        }

#if UNITY_EDITOR
        public void EditorAssign(DisasterEnvironment env, MissionPlanner missionPlanner,
                                 FleetSimulator fleet, MissionRecorder metrics)
        {
            environment = env;
            planner = missionPlanner;
            simulator = fleet;
            recorder = metrics;
        }
#endif
    }
}
