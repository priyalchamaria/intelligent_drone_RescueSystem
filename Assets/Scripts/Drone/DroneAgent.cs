using System.Collections.Generic;
using UnityEngine;
using DroneRescue.Navigation;

namespace DroneRescue.Fleet
{
    /// <summary>
    /// The scene-side representation of one drone.
    ///
    /// It owns two things: the Drone record the Mission Planner reads, and a
    /// RouteFollower holding the flight state. It does not decide anything. The
    /// FleetSimulator steps every follower each frame, and this component copies
    /// the result onto the transform and republishes the position into the shared
    /// record.
    ///
    /// That republishing is the uplink half of the hub-and-spoke model: state flows
    /// drone to planner through shared data, never drone to drone.
    /// </summary>
    public class DroneAgent : MonoBehaviour
    {
        [SerializeField] private string droneId = "D-01";
        [SerializeField, Range(0f, 100f)] private float startBatteryPercent = 100f;
        [SerializeField] private float bodyRadius = 0.6f;
        [SerializeField] private float maxSpeed = 8f;

        /// <summary>The shared-state record for this drone. Created on Awake.</summary>
        public Drone Data { get; private set; }

        /// <summary>Flight state: the route, the current waypoint, the velocity.</summary>
        public RouteFollower Follower { get; private set; }

        public string DroneId => droneId;
        public float BodyRadius => bodyRadius;
        public float MaxSpeed => maxSpeed;

        public Vector3 Velocity => Follower != null ? Follower.Velocity : Vector3.zero;
        public Vector3 CurrentGoal => Follower != null ? Follower.CurrentWaypoint : transform.position;
        public bool HasRoute => Follower != null && Follower.HasRoute && !Follower.Finished;

        private void Awake()
        {
            EnsureData();
        }

        /// <summary>
        /// Creates this drone's shared-state record and flight state if they do not
        /// exist yet, and returns the record.
        ///
        /// Unity does not define the order in which Awake runs across objects, so
        /// the environment cannot assume this drone has already woken when it
        /// collects the fleet. Both sides call this, and whichever runs first
        /// creates the state.
        /// </summary>
        public Drone EnsureData()
        {
            if (Data == null)
            {
                Data = new Drone(droneId, transform.position, startBatteryPercent, DroneStatus.Idle);
            }

            if (Follower == null)
            {
                Follower = new RouteFollower
                {
                    // Assigned properly by the environment during registration. It
                    // only ever breaks ties between two exactly overlapping drones.
                    Id = 0,
                    Position = transform.position,
                    Altitude = transform.position.y,
                    Radius = bodyRadius,
                    MaxSpeed = maxSpeed
                };
            }

            return Data;
        }

        /// <summary>Called by the editor-time scene builder so the inspector shows scenario values.</summary>
        public void Configure(string id, float batteryPercent)
        {
            droneId = id;
            startBatteryPercent = batteryPercent;
        }

        /// <summary>
        /// Sets the id used to break avoidance ties. The environment assigns these
        /// from registration order so a run is reproducible.
        /// </summary>
        public void SetAgentId(int id)
        {
            EnsureData();
            Follower.Id = id;
        }

        /// <summary>Gives this drone a route to fly. Phase 2 assigns these by hand; Phase 5 dispatches them.</summary>
        public void AssignRoute(IEnumerable<Vector3> waypoints)
        {
            EnsureData();
            Follower.AssignRoute(waypoints);
        }

        /// <summary>Copies flight state onto the transform and into the shared record.</summary>
        public void SyncFromFollower()
        {
            if (Follower == null)
                return;

            transform.position = Follower.Position;

            if (Data != null)
                Data.location = Follower.Position;
        }

        /// <summary>
        /// Snapshot this drone for local avoidance. Neighbours read these snapshots
        /// directly, which represents onboard sensing rather than communication.
        /// </summary>
        public AgentData ToAgentData(int index)
        {
            EnsureData();
            return Follower.Snapshot();
        }
    }
}
