using System.Collections.Generic;
using UnityEngine;
using DroneRescue.Fleet;

namespace DroneRescue.Environment
{
    /// <summary>
    /// Phase 2 driver. Sends ONE named drone to ONE named patient and then on to
    /// the nearest hospital, using NavigationEngine.FindSafePath directly.
    ///
    /// There is deliberately no Mission Planner here. Nothing is scored, nothing is
    /// filtered, and no drone is chosen: the pair is named in the inspector. This
    /// exists only to prove the navigation engine drives a real drone along a real
    /// obstacle-clearing route. Phase 5 replaces it with actual dispatch.
    /// </summary>
    [DisallowMultipleComponent]
    public class SingleDroneNavigationDemo : MonoBehaviour
    {
        [SerializeField] private DisasterEnvironment environment;

        [Header("The hardcoded pair")]
        [SerializeField] private string droneId = "D-01";
        [SerializeField] private string patientId = "P-01";

        [Tooltip("Fly patient to hospital after pickup, rather than stopping at the patient.")]
        [SerializeField] private bool continueToHospital = true;

        [Tooltip("Log the plan and each leg as it happens.")]
        [SerializeField] private bool verbose = true;

        private DroneAgent _drone;
        private Patient _patient;
        private bool _launched;
        private bool _pickupDone;
        private bool _reported;

        private void Awake()
        {
            if (environment == null)
                environment = FindAnyObjectByType<DisasterEnvironment>();
        }

        private void Update()
        {
            if (environment == null || environment.Navigation == null)
                return;

            // Launch on the first frame after the environment has registered the
            // fleet. Doing this in Update rather than Start keeps it independent of
            // Unity's script execution order.
            if (!_launched)
            {
                Launch();
                return;
            }

            if (_drone == null || _drone.Follower == null)
                return;

            if (!_drone.Follower.Finished)
                return;

            if (!_pickupDone)
            {
                OnReachedPatient();
                return;
            }

            if (!_reported)
                OnReachedHospital();
        }

        private void Launch()
        {
            if (environment.DroneList.Count == 0 || environment.Patients.Count == 0)
                return;

            _drone = FindDrone(droneId);
            _patient = FindPatient(patientId);

            if (_drone == null || _patient == null)
            {
                Debug.LogError("[Phase2] Could not find drone '" + droneId + "' or patient '" + patientId + "'.");
                enabled = false;
                return;
            }

            var route = environment.Navigation.FindSafePath(_drone.Follower.Position, _patient.location);

            if (route.Count == 0)
            {
                Debug.LogError("[Phase2] No route from " + droneId + " to " + patientId + ".");
                enabled = false;
                return;
            }

            _drone.AssignRoute(route);
            _drone.Data.status = DroneStatus.Busy;
            _patient.state = PatientState.Assigned;
            _patient.assignedDroneId = _drone.DroneId;
            _launched = true;

            if (verbose)
            {
                Debug.Log("[Phase2] " + droneId + " -> " + patientId
                    + " | waypoints=" + route.Count
                    + " | routeLength=" + RouteLength(route).ToString("F1")
                    + " | straightLine=" + Vector3.Distance(_drone.Follower.Position, _patient.location).ToString("F1"));
            }
        }

        private void OnReachedPatient()
        {
            _pickupDone = true;
            _patient.state = PatientState.Reached;
            _patient.reachedAtTime = Time.time;

            if (verbose)
                Debug.Log("[Phase2] " + droneId + " reached " + patientId + " at " + _drone.Follower.Position);

            if (!continueToHospital)
            {
                _reported = true;
                _drone.Data.status = DroneStatus.Idle;
                return;
            }

            var hospital = environment.NearestHospital(_patient.location);
            var route = environment.Navigation.FindSafePath(_drone.Follower.Position, hospital);

            if (route.Count == 0)
            {
                Debug.LogError("[Phase2] No route from " + patientId + " to hospital.");
                _reported = true;
                return;
            }

            _drone.AssignRoute(route);

            if (verbose)
                Debug.Log("[Phase2] " + patientId + " -> hospital | waypoints=" + route.Count
                    + " | routeLength=" + RouteLength(route).ToString("F1"));
        }

        private void OnReachedHospital()
        {
            _reported = true;
            _patient.state = PatientState.Delivered;
            _drone.Data.status = DroneStatus.Idle;

            if (verbose)
                Debug.Log("[Phase2] " + droneId + " delivered " + patientId
                    + " to hospital at " + _drone.Follower.Position + ". Drone back to Idle.");
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
