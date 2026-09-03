using System.Collections.Generic;
using UnityEngine;
using DroneRescue.Fleet;

namespace DroneRescue.Environment
{
    /// <summary>
    /// Phase 3 driver. Launches several drones at once along deliberately crossing
    /// routes, so local avoidance has something real to resolve.
    ///
    /// Still no Mission Planner. The drone-to-patient pairings are named in the
    /// inspector and chosen precisely because their routes conflict, which is the
    /// opposite of what a planner would do. Phase 5 replaces this with scoring and
    /// dispatch.
    /// </summary>
    [DisallowMultipleComponent]
    public class MultiDroneAvoidanceDemo : MonoBehaviour
    {
        [System.Serializable]
        public class Assignment
        {
            public string droneId = "D-01";
            public string patientId = "P-01";
        }

        [SerializeField] private DisasterEnvironment environment;
        [SerializeField] private FleetSimulator simulator;

        [Tooltip("Drone-to-patient pairs launched together. Chosen so the routes cross.")]
        [SerializeField]
        private List<Assignment> assignments = new List<Assignment>
        {
            // These three were picked by measurement, not by eye. With avoidance
            // disabled they produce a genuine body-to-body overlap; with it enabled
            // they stay clear. Pairings whose routes merely look crossed on the map
            // never actually conflict and prove nothing.
            new Assignment { droneId = "D-01", patientId = "P-03" },
            new Assignment { droneId = "D-04", patientId = "P-02" },
            new Assignment { droneId = "D-05", patientId = "P-01" },
        };

        [Tooltip("Continue to the nearest hospital after reaching the patient.")]
        [SerializeField] private bool continueToHospital = true;

        [SerializeField] private bool verbose = true;

        private class Mission
        {
            public DroneAgent Drone;
            public Patient Patient;
            public bool PickupDone;
            public bool Complete;
            public int Slot;
        }

        private readonly List<Mission> _missions = new List<Mission>();
        private bool _launched;
        private bool _summarised;

        private void Awake()
        {
            if (environment == null)
                environment = FindAnyObjectByType<DisasterEnvironment>();
            if (simulator == null)
                simulator = FindAnyObjectByType<FleetSimulator>();
        }

        private void Update()
        {
            if (environment == null || environment.Navigation == null)
                return;

            if (!_launched)
            {
                Launch();
                return;
            }

            AdvanceMissions();
        }

        private void Launch()
        {
            if (environment.DroneList.Count == 0 || environment.Patients.Count == 0)
                return;

            if (simulator != null)
            {
                // Phase 3 is the phase where avoidance goes on.
                simulator.EnableLocalAvoidance = true;
                simulator.ResetRun();
            }

            foreach (var assignment in assignments)
            {
                var drone = FindDrone(assignment.droneId);
                var patient = FindPatient(assignment.patientId);

                if (drone == null || patient == null)
                {
                    Debug.LogError("[Phase3] Unknown drone '" + assignment.droneId
                        + "' or patient '" + assignment.patientId + "'.");
                    continue;
                }

                var route = environment.Navigation.FindSafePath(drone.Follower.Position, patient.location);
                if (route.Count == 0)
                {
                    Debug.LogError("[Phase3] No route " + assignment.droneId + " -> " + assignment.patientId + ".");
                    continue;
                }

                drone.AssignRoute(route);
                drone.Data.status = DroneStatus.Busy;
                patient.state = PatientState.Assigned;
                patient.assignedDroneId = drone.DroneId;

                _missions.Add(new Mission { Drone = drone, Patient = patient, Slot = _missions.Count });

                if (verbose)
                    Debug.Log("[Phase3] launch " + assignment.droneId + " -> " + assignment.patientId
                        + " | waypoints=" + route.Count + " | length=" + RouteLength(route).ToString("F1"));
            }

            _launched = true;

            if (verbose)
                Debug.Log("[Phase3] " + _missions.Count + " drones flying simultaneously, local avoidance ON.");
        }

        private void AdvanceMissions()
        {
            bool allComplete = _missions.Count > 0;

            foreach (var mission in _missions)
            {
                if (mission.Complete)
                    continue;

                allComplete = false;

                if (mission.Drone.Follower == null || !mission.Drone.Follower.Finished)
                    continue;

                if (!mission.PickupDone)
                {
                    mission.PickupDone = true;
                    mission.Patient.state = PatientState.Reached;
                    mission.Patient.reachedAtTime = Time.time;

                    if (verbose)
                        Debug.Log("[Phase3] " + mission.Drone.DroneId + " reached " + mission.Patient.id + ".");

                    if (!continueToHospital)
                    {
                        CompleteMission(mission);
                        continue;
                    }

                    // Each delivery gets its own stand on the hospital ring. Sending
                    // them all to the identical centre point makes the first drone to
                    // land block the pad for everyone behind it.
                    var hospital = environment.NearestHospital(mission.Patient.location);
                    var stand = environment.LandingSlot(hospital, mission.Slot, _missions.Count);
                    var route = environment.Navigation.FindSafePath(mission.Drone.Follower.Position, stand);

                    if (route.Count == 0)
                    {
                        Debug.LogError("[Phase3] No route " + mission.Patient.id + " -> hospital.");
                        CompleteMission(mission);
                        continue;
                    }

                    mission.Drone.AssignRoute(route);
                    continue;
                }

                mission.Patient.state = PatientState.Delivered;
                CompleteMission(mission);
            }

            if (allComplete && !_summarised)
                Summarise();
        }

        private void CompleteMission(Mission mission)
        {
            mission.Complete = true;
            mission.Drone.Data.status = DroneStatus.Idle;

            if (verbose)
                Debug.Log("[Phase3] " + mission.Drone.DroneId + " finished " + mission.Patient.id + ".");
        }

        private void Summarise()
        {
            _summarised = true;

            if (simulator == null)
            {
                Debug.Log("[Phase3] all missions complete.");
                return;
            }

            var tracker = simulator.Collisions;
            Debug.Log("[Phase3] ALL COMPLETE. simulatedTime=" + simulator.SimulatedTime.ToString("F1") + "s"
                + " | collisions=" + tracker.CollisionCount
                + " | closestApproach=" + tracker.MinSeparation.ToString("F2") + " units of clear air between bodies");
        }

        private DroneAgent FindDrone(string id)
        {
            foreach (var agent in environment.DroneAgents)
                if (agent.DroneId == id)
                    return agent;
            return null;
        }

        private Patient FindPatient(string id)
        {
            foreach (var patient in environment.Patients)
                if (patient.id == id)
                    return patient;
            return null;
        }

        private static float RouteLength(List<Vector3> route)
        {
            float total = 0f;
            for (int i = 1; i < route.Count; i++)
                total += Vector3.Distance(route[i - 1], route[i]);
            return total;
        }
    }
}
