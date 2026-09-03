using System.Collections.Generic;
using UnityEngine;
using DroneRescue.Environment;
using DroneRescue.Fleet;

namespace DroneRescue.Planning
{
    /// <summary>
    /// Phase 4 driver. Proves the priority queue orders patients correctly and that
    /// the Part 1 Step 2 hard filter excludes the drones it should.
    ///
    /// Nothing flies. No drone is chosen, nothing is scored and nothing is
    /// dispatched: this phase stops at the point where the planner has a shortlist.
    /// Phase 5 picks the winner off that shortlist and hands it to the navigation
    /// engine.
    ///
    /// The state overrides below exist because the authored scenario starts with
    /// six Idle drones on full-ish batteries, so every drone would pass every filter
    /// and the checkpoint would prove nothing. Forcing a Busy drone, a flat drone
    /// and an Offline drone makes each exclusion path observable in the Console.
    /// </summary>
    [DisallowMultipleComponent]
    public class MissionPlannerFilterDemo : MonoBehaviour
    {
        /// <summary>One forced drone state, applied once at start so a filter path is exercised.</summary>
        [System.Serializable]
        public class StateOverride
        {
            public string droneId = "D-02";
            public bool setStatus = true;
            public DroneStatus status = DroneStatus.Busy;
            public bool setBattery;
            [Range(0f, 100f)] public float batteryPercent = 100f;
            public string note = "";
        }

        [SerializeField] private DisasterEnvironment environment;
        [SerializeField] private MissionPlanner planner;

        [Header("Forced states, so each exclusion path is visible")]
        [SerializeField]
        private List<StateOverride> overrides = new List<StateOverride>
        {
            new StateOverride
            {
                droneId = "D-02", setStatus = true, status = DroneStatus.Busy,
                note = "already on a task, must be excluded by the status filter"
            },
            new StateOverride
            {
                droneId = "D-03", setStatus = false, setBattery = true, batteryPercent = 20f,
                note = "Idle but flat, must be excluded by the range filter"
            },
            new StateOverride
            {
                droneId = "D-06", setStatus = true, status = DroneStatus.Offline,
                note = "failed drone, must be excluded by the status filter"
            },
        };

        [SerializeField] private bool verbose = true;

        private bool _ran;

        private void Awake()
        {
            if (environment == null)
                environment = FindAnyObjectByType<DisasterEnvironment>();
            if (planner == null)
                planner = FindAnyObjectByType<MissionPlanner>();
        }

        private void Update()
        {
            if (_ran || environment == null || planner == null)
                return;

            // Wait for the environment to have registered the fleet. Running in
            // Update rather than Start keeps this independent of script order.
            if (environment.DroneList.Count == 0 || environment.Patients.Count == 0)
                return;

            _ran = true;
            ApplyOverrides();
            ReportQueue();
            ReportFilters();
        }

        private void ApplyOverrides()
        {
            foreach (var item in overrides)
            {
                var drone = FindDrone(item.droneId);
                if (drone == null)
                {
                    Debug.LogError("[Phase4] Unknown drone in overrides: " + item.droneId);
                    continue;
                }

                if (item.setStatus)
                    drone.status = item.status;
                if (item.setBattery)
                    drone.batteryPercent = item.batteryPercent;

                if (verbose)
                    Debug.Log("[Phase4] forced " + drone.id + " -> " + drone.status
                              + ", battery " + drone.batteryPercent.ToString("F0") + "%"
                              + (string.IsNullOrEmpty(item.note) ? "" : " (" + item.note + ")"));
            }
        }

        private void ReportQueue()
        {
            planner.ResetQueue();
            int added = planner.SyncQueueFromEnvironment();

            var text = new System.Text.StringBuilder();
            text.Append("[Phase4] Priority queue holds ").Append(added)
                .Append(" patients, service order:");

            int position = 1;
            foreach (var patient in planner.Queue.ToOrderedList())
                text.Append("\n  ").Append(position++).Append(". ").Append(patient.id)
                    .Append(" [").Append(patient.priority).Append("] at ").Append(patient.location);

            Debug.Log(text.ToString());
        }

        private void ReportFilters()
        {
            // Pop the whole queue so every patient's filter pass is logged. In
            // Phase 5 a pop is followed by scoring and a dispatch; here it is
            // followed by nothing, which is the point.
            int served = 0;
            while (true)
            {
                var patient = planner.PopNextPatient();
                if (patient == null)
                    break;

                served++;
                var feasible = planner.FilterFeasible(patient);

                if (feasible.Count == 0)
                    Debug.LogWarning("[Phase4] " + patient.id
                        + " has NO feasible drone. It would stay queued for a later attempt.");
            }

            Debug.Log("[Phase4] FILTERING COMPLETE. " + served
                      + " patients evaluated against " + environment.DroneList.Count
                      + " drones. Nothing dispatched: scoring is Phase 5.");
        }

        private Drone FindDrone(string id)
        {
            foreach (var drone in environment.DroneList)
                if (drone.id == id)
                    return drone;
            return null;
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
