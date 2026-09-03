using UnityEngine;

namespace DroneRescue.Navigation
{
    /// <summary>
    /// A static or dynamic hazard in the disaster environment.
    /// Footprint is treated as a circle on the XZ plane; Y is ignored because the
    /// simulation is effectively 2.5D (drones fly at a fixed altitude band).
    /// </summary>
    [System.Serializable]
    public struct Obstacle
    {
        public Vector3 center;
        public float radius;

        /// <summary>
        /// Static obstacles (rubble, buildings) are baked into the grid and only
        /// re-baked on a FireSpread event. Dynamic ones (spreading fire fronts) are
        /// additionally fed to local avoidance every frame.
        /// </summary>
        public bool isDynamic;

        public Obstacle(Vector3 center, float radius, bool isDynamic = false)
        {
            this.center = center;
            this.radius = radius;
            this.isDynamic = isDynamic;
        }
    }

    /// <summary>
    /// The per-agent state that local avoidance reads. Deliberately a plain value
    /// type with no reference to a GameObject: a drone publishes a snapshot of
    /// itself each frame, and neighbours are read directly from those snapshots.
    ///
    /// NOTE ON THE COMMUNICATION MODEL: reading a neighbour's AgentData is NOT a
    /// message between drones. It stands in for onboard sensing (radar/vision) of a
    /// nearby drone's position and velocity. Avoidance stays communication-free,
    /// consistent with ORCA's actual design (van den Berg et al., 2011).
    /// </summary>
    public struct AgentData
    {
        public int id;
        public Vector3 position;
        public Vector3 velocity;

        /// <summary>Physical radius used for separation distance.</summary>
        public float radius;

        public float maxSpeed;

        /// <summary>
        /// The waypoint this agent is currently steering toward. The preferred
        /// velocity is derived from it inside ComputeAvoidanceVelocity.
        /// </summary>
        public Vector3 goal;

        public AgentData(int id, Vector3 position, Vector3 velocity, float radius, float maxSpeed, Vector3 goal)
        {
            this.id = id;
            this.position = position;
            this.velocity = velocity;
            this.radius = radius;
            this.maxSpeed = maxSpeed;
            this.goal = goal;
        }
    }
}
