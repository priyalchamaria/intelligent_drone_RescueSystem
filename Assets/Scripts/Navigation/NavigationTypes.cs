using UnityEngine;

namespace DroneRescue.Navigation
{
    /// <summary>Footprint shape of an obstacle on the XZ plane.</summary>
    public enum ObstacleShape
    {
        /// <summary>Fire zones and blast radii.</summary>
        Circle = 0,

        /// <summary>Collapsed buildings and rubble, which are authored as boxes in the scene.</summary>
        Box = 1
    }

    /// <summary>
    /// A static or dynamic hazard in the disaster environment.
    /// Footprints live on the XZ plane; Y is ignored because the simulation is
    /// effectively 2.5D (drones fly in a fixed altitude band).
    /// </summary>
    [System.Serializable]
    public struct Obstacle
    {
        public Vector3 center;
        public ObstacleShape shape;

        /// <summary>Used when shape is Circle.</summary>
        public float radius;

        /// <summary>Used when shape is Box. Only x and z matter.</summary>
        public Vector3 halfExtents;

        /// <summary>
        /// Static obstacles (rubble, buildings) are baked into the grid and only
        /// re-baked on a FireSpread event. Dynamic ones (spreading fire fronts) are
        /// additionally fed to local avoidance every frame.
        /// </summary>
        public bool isDynamic;

        /// <summary>Circular footprint. This is the fire-zone case.</summary>
        public Obstacle(Vector3 center, float radius, bool isDynamic = false)
        {
            this.center = center;
            this.shape = ObstacleShape.Circle;
            this.radius = radius;
            this.halfExtents = new Vector3(radius, 0f, radius);
            this.isDynamic = isDynamic;
        }

        /// <summary>Axis-aligned box footprint. This is the collapsed-building case.</summary>
        public Obstacle(Vector3 center, Vector3 halfExtents, bool isDynamic = false)
        {
            this.center = center;
            this.shape = ObstacleShape.Box;
            this.halfExtents = halfExtents;
            this.radius = Mathf.Sqrt(halfExtents.x * halfExtents.x + halfExtents.z * halfExtents.z);
            this.isDynamic = isDynamic;
        }

        /// <summary>Largest distance from the centre to any point of the footprint. Used for broad-phase culling.</summary>
        public float BoundingRadius => shape == ObstacleShape.Circle
            ? radius
            : Mathf.Sqrt(halfExtents.x * halfExtents.x + halfExtents.z * halfExtents.z);

        /// <summary>The point of this footprint nearest to the given world position, on the XZ plane.</summary>
        public Vector3 ClosestPoint(Vector3 world)
        {
            if (shape == ObstacleShape.Box)
            {
                float x = Mathf.Clamp(world.x, center.x - halfExtents.x, center.x + halfExtents.x);
                float z = Mathf.Clamp(world.z, center.z - halfExtents.z, center.z + halfExtents.z);
                return new Vector3(x, center.y, z);
            }

            Vector3 offset = world - center;
            offset.y = 0f;
            float distance = offset.magnitude;
            if (distance <= radius || distance < 0.0001f)
                return new Vector3(world.x, center.y, world.z);

            offset = offset / distance * radius;
            return new Vector3(center.x + offset.x, center.y, center.z + offset.z);
        }

        /// <summary>True when the world position lies inside the footprint.</summary>
        public bool Contains(Vector3 world)
        {
            if (shape == ObstacleShape.Box)
            {
                return Mathf.Abs(world.x - center.x) <= halfExtents.x
                    && Mathf.Abs(world.z - center.z) <= halfExtents.z;
            }

            float dx = world.x - center.x;
            float dz = world.z - center.z;
            return dx * dx + dz * dz <= radius * radius;
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
