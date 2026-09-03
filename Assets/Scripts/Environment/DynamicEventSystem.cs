using System.Collections.Generic;
using UnityEngine;
using UnityEngine.InputSystem;
using DroneRescue.Fleet;
using DroneRescue.Planning;

namespace DroneRescue.Environment
{
    /// <summary>Which of the four Part 1 dynamic events a scripted entry fires.</summary>
    public enum DynamicEventKind
    {
        BatteryLow = 0,
        FireSpread = 1,
        DroneFailed = 2,
        NewEmergency = 3
    }

    /// <summary>One event fired at a set time during a scripted run.</summary>
    [System.Serializable]
    public class ScriptedEvent
    {
        [Tooltip("Seconds after the run starts.")]
        [Min(0f)] public float atSeconds = 5f;

        public DynamicEventKind kind = DynamicEventKind.FireSpread;

        [Tooltip("BatteryLow and DroneFailed only. Leave empty to pick whichever drone is busiest to lose.")]
        public string droneId = "";

        [Tooltip("FireSpread centre, or NewEmergency location.")]
        public Vector3 location;

        [Tooltip("FireSpread only.")]
        [Min(0.5f)] public float radius = 9f;

        [Tooltip("NewEmergency only.")]
        public PatientPriority priority = PatientPriority.Critical;

        [HideInInspector] public bool fired;
    }

    /// <summary>
    /// The four Part 1 dynamic events, and the ways to fire them.
    ///
    /// NOT AN ALGORITHM. Every method here changes one fact about the world and
    /// then calls straight back into the Mission Planner and Navigation Engine
    /// entry points that a normal dispatch already uses. There is no separate
    /// re-planning path, no per-event scoring rule, and nothing here that the
    /// unattended Phase 5 run does not also go through:
    ///
    ///   BatteryLow(drone)    status Charging, then MissionPlanner.Reassign
    ///   DroneFailed(drone)   status Offline,  then MissionPlanner.Reassign
    ///   NewEmergency(p)      DisasterEnvironment.DetectPatient, then Queue.Push
    ///   FireSpread(zone)     DisasterGrid.AddObstacle, then re-path only
    ///
    /// FireSpread is the odd one out on purpose. It does NOT re-run the planner's
    /// scoring: the assignments are still correct, it is only the routes through
    /// the world that have gone stale, so every active leg is re-planned to the
    /// destination it already had.
    ///
    /// Three ways to fire them, all reaching the same methods: the number keys 1
    /// to 4 while the Game view has focus, the right-click context menu on this
    /// component, and the scripted timeline below.
    /// </summary>
    [DisallowMultipleComponent]
    public class DynamicEventSystem : MonoBehaviour
    {
        [SerializeField] private DisasterEnvironment environment;
        [SerializeField] private MissionPlanner planner;

        [Header("Manual triggers")]
        [Tooltip("Number keys 1 BatteryLow, 2 FireSpread, 3 DroneFailed, 4 NewEmergency. " +
                 "Needs Game view focus. The context menu on this component works either way.")]
        [SerializeField] private bool keyboardTriggers = true;

        [Header("What the manual triggers use")]
        [Tooltip("Leave empty to pick a drone that is actually flying, which is the case worth watching.")]
        [SerializeField] private string manualDroneId = "";

        [SerializeField] private Vector3 manualFireCenter = new Vector3(50f, 0f, 45f);
        [SerializeField, Min(0.5f)] private float manualFireRadius = 9f;
        [SerializeField] private Vector3 manualEmergencyLocation = new Vector3(60f, 0f, 75f);
        [SerializeField] private PatientPriority manualEmergencyPriority = PatientPriority.Critical;

        [Header("All four in one run")]
        [Tooltip("Fires the timeline below automatically. Turn off for the clean Phase 5 run.")]
        [SerializeField] private bool runScriptedScenario = true;

        [SerializeField]
        private List<ScriptedEvent> timeline = new List<ScriptedEvent>
        {
            // Ordered so each event lands on a fleet the previous one has already
            // changed, which is the part worth proving: they compose.
            //
            // Both knockouts hit a drone on its OUTBOUND leg, on purpose. Losing a
            // drone that is already carrying somebody to hospital sends the
            // replacement out from the hospital and back, which on this map costs
            // more range than the survivors have left. That is a true result and
            // the planner reports it honestly, but it makes for a scenario that
            // ends with a patient nobody can reach, which is not what this timeline
            // is meant to demonstrate.
            new ScriptedEvent
            {
                atSeconds = 4f, kind = DynamicEventKind.FireSpread,
                location = new Vector3(50f, 0f, 45f), radius = 9f
            },
            new ScriptedEvent
            {
                atSeconds = 6f, kind = DynamicEventKind.BatteryLow, droneId = "D-02"
            },
            new ScriptedEvent
            {
                atSeconds = 7f, kind = DynamicEventKind.NewEmergency,
                location = new Vector3(60f, 0f, 75f), priority = PatientPriority.Critical
            },
            new ScriptedEvent
            {
                atSeconds = 10f, kind = DynamicEventKind.DroneFailed, droneId = "D-04"
            },
        };

        [Header("Automatic battery watchdog")]
        [Tooltip("Fire BatteryLow by itself when a flying drone drops this low, rather than only " +
                 "on a trigger. OFF by default and not part of the Part 1 event set: with no " +
                 "recharging modelled anywhere, it strands a carried patient the moment no drone " +
                 "is left with the range to take over. Turn it on to watch that failure happen.")]
        [SerializeField] private bool watchBatteries;

        [Tooltip("Charge at which the watchdog fires, and the charge a manual BatteryLow drops a drone to.")]
        [SerializeField, Range(0f, 100f)] private float criticalBatteryPercent = 15f;

        private float _startTime;

        private void Awake()
        {
            if (environment == null)
                environment = FindAnyObjectByType<DisasterEnvironment>();
            if (planner == null)
                planner = FindAnyObjectByType<MissionPlanner>();
        }

        private void Start()
        {
            _startTime = Time.time;

            foreach (var entry in timeline)
                entry.fired = false;
        }

        private void Update()
        {
            if (environment == null || planner == null)
                return;

            if (keyboardTriggers)
                PollKeyboard();

            if (runScriptedScenario)
                AdvanceTimeline();

            if (watchBatteries)
                WatchBatteries();
        }

        // -----------------------------------------------------------------
        // The four events
        // -----------------------------------------------------------------

        /// <summary>
        /// BatteryLow(drone). Drops the drone's charge to the critical level first,
        /// so the state genuinely matches the event rather than the log claiming a
        /// flat battery on a drone sitting at 80 percent.
        /// </summary>
        public void FireBatteryLow(string droneId)
        {
            var drone = ResolveDrone(droneId);
            if (drone == null)
            {
                Debug.LogWarning("[Event] BatteryLow: no drone to apply it to.");
                return;
            }

            if (drone.batteryPercent > criticalBatteryPercent)
                drone.batteryPercent = criticalBatteryPercent;

            planner.OnBatteryLow(drone);
        }

        /// <summary>DroneFailed(drone).</summary>
        public void FireDroneFailed(string droneId)
        {
            var drone = ResolveDrone(droneId);
            if (drone == null)
            {
                Debug.LogWarning("[Event] DroneFailed: no drone to apply it to.");
                return;
            }

            planner.OnDroneFailed(drone);
        }

        /// <summary>
        /// FireSpread(zone). The obstacle map changes, then every active route is
        /// re-planned around it. Nothing is re-scored and no drone changes patient.
        /// </summary>
        public void FireSpread(Vector3 center, float radius)
        {
            Debug.Log("[Event] FIRE SPREAD: new zone radius " + radius.ToString("F1")
                      + "u at " + center + ". Obstacle map rebaked, re-pathing only, no re-scoring.");

            environment.SpreadFire(center, radius);

            int repathed = planner.RepathActiveMissions();
            Debug.Log("[Event] FIRE SPREAD: " + repathed + " active route(s) re-planned around the fire.");
        }

        /// <summary>NewEmergency(patient).</summary>
        public void FireNewEmergency(Vector3 location, PatientPriority priority)
        {
            var patient = environment.DetectPatientAt(location, priority);
            planner.OnNewEmergency(patient);
        }

        // -----------------------------------------------------------------
        // Ways to fire them
        // -----------------------------------------------------------------

        [ContextMenu("Fire event 1: BatteryLow")]
        public void ContextBatteryLow() => FireBatteryLow(manualDroneId);

        [ContextMenu("Fire event 2: FireSpread")]
        public void ContextFireSpread() => FireSpread(manualFireCenter, manualFireRadius);

        [ContextMenu("Fire event 3: DroneFailed")]
        public void ContextDroneFailed() => FireDroneFailed(manualDroneId);

        [ContextMenu("Fire event 4: NewEmergency")]
        public void ContextNewEmergency() => FireNewEmergency(manualEmergencyLocation, manualEmergencyPriority);

        private void PollKeyboard()
        {
            // The project is on the new Input System, so the old Input class throws
            // rather than simply returning false. Keyboard.current is null on a
            // machine with no keyboard, which is not an error worth reporting.
            var keyboard = Keyboard.current;
            if (keyboard == null)
                return;

            if (keyboard.digit1Key.wasPressedThisFrame)
                ContextBatteryLow();
            if (keyboard.digit2Key.wasPressedThisFrame)
                ContextFireSpread();
            if (keyboard.digit3Key.wasPressedThisFrame)
                ContextDroneFailed();
            if (keyboard.digit4Key.wasPressedThisFrame)
                ContextNewEmergency();
        }

        private void AdvanceTimeline()
        {
            float elapsed = Time.time - _startTime;

            for (int i = 0; i < timeline.Count; i++)
            {
                var entry = timeline[i];
                if (entry.fired || elapsed < entry.atSeconds)
                    continue;

                entry.fired = true;

                switch (entry.kind)
                {
                    case DynamicEventKind.BatteryLow:
                        FireBatteryLow(entry.droneId);
                        break;
                    case DynamicEventKind.FireSpread:
                        FireSpread(entry.location, entry.radius);
                        break;
                    case DynamicEventKind.DroneFailed:
                        FireDroneFailed(entry.droneId);
                        break;
                    default:
                        FireNewEmergency(entry.location, entry.priority);
                        break;
                }
            }
        }

        /// <summary>
        /// Fires BatteryLow on its own when a flying drone runs genuinely low, so the
        /// event is not purely a demo button. The handler sets the drone to Charging,
        /// which is what stops this re-firing on the same drone every frame.
        /// </summary>
        private void WatchBatteries()
        {
            var fleet = environment.DroneList;

            for (int i = 0; i < fleet.Count; i++)
            {
                var drone = fleet[i];
                if (drone.status == DroneStatus.Busy && drone.batteryPercent <= criticalBatteryPercent)
                    planner.OnBatteryLow(drone);
            }
        }

        /// <summary>
        /// The drone a manual trigger should hit. A named one when given, otherwise
        /// one that is actually flying: knocking out a parked drone changes nothing
        /// and proves nothing about reassignment.
        /// </summary>
        private Drone ResolveDrone(string droneId)
        {
            var fleet = environment.DroneList;

            if (!string.IsNullOrEmpty(droneId))
            {
                for (int i = 0; i < fleet.Count; i++)
                    if (fleet[i].id == droneId)
                        return fleet[i];

                Debug.LogWarning("[Event] Unknown drone id '" + droneId + "'.");
                return null;
            }

            for (int i = 0; i < fleet.Count; i++)
                if (fleet[i].status == DroneStatus.Busy)
                    return fleet[i];

            for (int i = 0; i < fleet.Count; i++)
                if (fleet[i].status == DroneStatus.Idle)
                    return fleet[i];

            return fleet.Count > 0 ? fleet[0] : null;
        }

#if UNITY_EDITOR
        public void EditorAssign(DisasterEnvironment env, MissionPlanner missionPlanner)
        {
            environment = env;
            planner = missionPlanner;
        }
#endif
    }
}
