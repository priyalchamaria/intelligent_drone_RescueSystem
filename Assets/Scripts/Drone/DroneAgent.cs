using UnityEngine;
using DroneRescue.Navigation;

namespace DroneRescue.Fleet
{
    /// <summary>
    /// The scene-side placeholder for one drone.
    ///
    /// Its only job in Phase 1 is to own a Drone record and keep that record's
    /// location in step with the transform. That record is what the Mission
    /// Planner reads, which is the uplink half of the hub-and-spoke model: state
    /// flows drone to planner through shared data, never drone to drone.
    ///
    /// No movement here yet. Phase 2 adds path following, Phase 3 adds avoidance.
    /// </summary>
    public class DroneAgent : MonoBehaviour
    {
        [SerializeField] private string droneId = "D-01";
        [SerializeField, Range(0f, 100f)] private float startBatteryPercent = 100f;
        [SerializeField] private float bodyRadius = 0.6f;
        [SerializeField] private float maxSpeed = 8f;

        /// <summary>The shared-state record for this drone. Created on Awake.</summary>
        public Drone Data { get; private set; }

        public string DroneId => droneId;
        public float BodyRadius => bodyRadius;
        public float MaxSpeed => maxSpeed;

        /// <summary>Current velocity. Stays zero through Phase 1.</summary>
        public Vector3 Velocity { get; protected set; }

        /// <summary>Waypoint currently being steered toward. Unused until Phase 2.</summary>
        public Vector3 CurrentGoal { get; protected set; }

        private void Awake()
        {
            EnsureData();
        }

        /// <summary>
        /// Creates this drone's shared-state record if it does not exist yet, and
        /// returns it.
        ///
        /// Unity does not define the order in which Awake runs across objects, so
        /// the environment cannot assume this drone has already woken when it
        /// collects the fleet. Both sides call this, and whichever runs first
        /// creates the record.
        /// </summary>
        public Drone EnsureData()
        {
            if (Data == null)
            {
                Data = new Drone(droneId, transform.position, startBatteryPercent, DroneStatus.Idle);
                CurrentGoal = transform.position;
            }

            return Data;
        }

        /// <summary>Called by the editor-time scene builder so the inspector shows scenario values.</summary>
        public void Configure(string id, float batteryPercent)
        {
            droneId = id;
            startBatteryPercent = batteryPercent;
        }

        private void LateUpdate()
        {
            // Publish live position into shared state. This stands in for a status
            // uplink; it is not a message to any other drone.
            if (Data != null)
                Data.location = transform.position;
        }

        /// <summary>
        /// Snapshot this drone for local avoidance. Neighbours read these snapshots
        /// directly, which represents onboard sensing rather than communication.
        /// </summary>
        public AgentData ToAgentData(int index)
        {
            return new AgentData(index, transform.position, Velocity, bodyRadius, maxSpeed, CurrentGoal);
        }
    }
}
