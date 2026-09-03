using System.Collections.Generic;
using UnityEngine;
using DroneRescue.Navigation;

namespace DroneRescue.Fleet
{
    /// <summary>
    /// Follows a waypoint list, one simulation step at a time.
    ///
    /// This is the agent half of Stage 1's RunSimulation loop, lifted out of the
    /// MonoBehaviour so it can be stepped without a GameObject and tested headlessly.
    /// It holds no Unity object references at all.
    ///
    /// The step is deliberately split in two. ComputeVelocity decides where this
    /// agent wants to go without moving anything; Integrate then applies the move.
    /// Stage 1's loop did exactly this, computing every agent's velocity before
    /// moving any of them, which is what makes reciprocal avoidance work: each
    /// agent reacts to where its neighbours were at the start of the step, not to
    /// where earlier agents in the list have already moved to.
    /// </summary>
    public class RouteFollower
    {
        private readonly List<Vector3> _route = new List<Vector3>();
        private int _waypointIndex;

        /// <summary>Identifier used to break ties in avoidance. Stable per drone.</summary>
        public int Id;

        public Vector3 Position;
        public Vector3 Velocity;
        public float Radius = 0.6f;
        public float MaxSpeed = 8f;

        /// <summary>Altitude the drone holds. Routes are planned on the ground plane.</summary>
        public float Altitude;

        public bool HasRoute => _route.Count > 0;
        public bool Finished { get; private set; }
        public int WaypointIndex => _waypointIndex;
        public int WaypointCount => _route.Count;
        public IReadOnlyList<Vector3> Route => _route;

        /// <summary>The waypoint currently being steered toward, or the position when done.</summary>
        public Vector3 CurrentWaypoint =>
            _waypointIndex < _route.Count ? _route[_waypointIndex] : Position;

        /// <summary>Total remaining route length, straight line between waypoints.</summary>
        public float RemainingDistance()
        {
            if (_waypointIndex >= _route.Count)
                return 0f;

            float total = FlatDistance(Position, _route[_waypointIndex]);
            for (int i = _waypointIndex + 1; i < _route.Count; i++)
                total += FlatDistance(_route[i - 1], _route[i]);

            return total;
        }

        /// <summary>Adopts a new route and starts again from its first waypoint.</summary>
        public void AssignRoute(IEnumerable<Vector3> waypoints)
        {
            _route.Clear();
            if (waypoints != null)
                _route.AddRange(waypoints);

            _waypointIndex = 0;
            Finished = _route.Count == 0;
            Velocity = Vector3.zero;
        }

        public void ClearRoute()
        {
            _route.Clear();
            _waypointIndex = 0;
            Finished = true;
            Velocity = Vector3.zero;
        }

        /// <summary>Snapshot of this agent for the avoidance call and for neighbours to sense.</summary>
        public AgentData Snapshot()
        {
            return new AgentData(Id, Position, Velocity, Radius, MaxSpeed, CurrentWaypoint);
        }

        /// <summary>
        /// Decides this step's velocity. Does not move the agent.
        ///
        /// Ordering follows Stage 1's ComputeORCAVelocity exactly: the preferred
        /// direction is taken toward the CURRENT waypoint, and only afterwards is
        /// arrival tested and the index advanced. So the step in which a waypoint is
        /// reached still steers toward the waypoint just reached.
        ///
        /// Pass null for neighbors and obstacles to get pure path following with no
        /// avoidance, which is what Phase 2 uses.
        /// </summary>
        public void ComputeVelocity(NavigationEngine engine, List<AgentData> neighbors, List<Obstacle> obstacles, float dt)
        {
            if (!HasRoute || Finished || _waypointIndex >= _route.Count)
            {
                Finished = true;
                Velocity = Vector3.zero;
                return;
            }

            Velocity = engine.ComputeAvoidanceVelocity(Snapshot(), neighbors, obstacles);

            // ADAPTED: Stage 1 stepped at a fixed dt of 0.07 with a max speed of 3,
            // so one step moved 0.21 and could never overshoot the 0.35 tolerance.
            // Stage 2 steps at a variable frame time with faster drones, so a fixed
            // tolerance would let an agent jump past a waypoint and circle it
            // forever. The tolerance grows to cover one step of travel when needed.
            bool isFinalWaypoint = _waypointIndex == _route.Count - 1;
            float baseTolerance = isFinalWaypoint
                ? engine.Settings.finalApproachTolerance
                : engine.Settings.waypointTolerance;

            float tolerance = Mathf.Max(baseTolerance, Velocity.magnitude * dt * 1.5f);

            if (FlatDistance(Position, _route[_waypointIndex]) < tolerance)
            {
                _waypointIndex++;
                if (_waypointIndex >= _route.Count)
                {
                    Finished = true;
                    Velocity = Vector3.zero;
                }
            }
        }

        /// <summary>Applies the velocity decided by ComputeVelocity. Altitude is held constant.</summary>
        public void Integrate(float dt)
        {
            if (Finished)
                return;

            Position += Velocity * dt;
            Position.y = Altitude;
        }

        private static float FlatDistance(Vector3 a, Vector3 b)
        {
            float dx = a.x - b.x;
            float dz = a.z - b.z;
            return Mathf.Sqrt(dx * dx + dz * dz);
        }
    }
}
