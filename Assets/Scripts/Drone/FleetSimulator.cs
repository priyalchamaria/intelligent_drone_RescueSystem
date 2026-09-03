using System.Collections.Generic;
using UnityEngine;
using DroneRescue.Environment;
using DroneRescue.Navigation;

namespace DroneRescue.Fleet
{
    /// <summary>
    /// Steps every drone once per frame, using Stage 1's two-phase loop: compute
    /// all velocities from one shared snapshot, then move everyone.
    ///
    /// Doing it in two phases is what makes the avoidance reciprocal. If each drone
    /// moved inside its own Update, the second drone in the list would react to the
    /// first drone's already-updated position, and the pair would not resolve a
    /// head-on encounter symmetrically.
    ///
    /// Phase 2 runs with local avoidance off, so this reduces to pure path
    /// following for a single drone. Phase 3 turns the flag on and the same loop
    /// starts feeding neighbours and obstacles into the avoidance call.
    /// </summary>
    [DisallowMultipleComponent]
    public class FleetSimulator : MonoBehaviour
    {
        [SerializeField] private DisasterEnvironment environment;

        [Tooltip("Phase 3 turns this on. Off means each drone flies its route ignoring the others.")]
        [SerializeField] private bool enableLocalAvoidance = false;

        [Tooltip("Multiplies real time. 1 is real time; raise it to run a scenario faster.")]
        [SerializeField, Range(0.1f, 10f)] private float timeScale = 1f;

        [Tooltip("Largest step the simulation will take. Caps the effect of a frame hitch.")]
        [SerializeField] private float maxStep = 0.1f;

        private readonly List<AgentData> _snapshots = new List<AgentData>();
        private List<Obstacle> _dynamicObstacles = new List<Obstacle>();

        /// <summary>Simulated seconds elapsed since the run began. Phase 7 metrics use this.</summary>
        public float SimulatedTime { get; private set; }

        public bool EnableLocalAvoidance
        {
            get => enableLocalAvoidance;
            set => enableLocalAvoidance = value;
        }

        public DisasterEnvironment Environment => environment;

        private void Awake()
        {
            if (environment == null)
                environment = FindAnyObjectByType<DisasterEnvironment>();
        }

        private void Update()
        {
            if (environment == null || environment.Navigation == null)
                return;

            float dt = Mathf.Min(Time.deltaTime * timeScale, maxStep);
            if (dt <= 0f)
                return;

            Step(dt);
        }

        /// <summary>
        /// One simulation step. Public so an editor-time harness can drive the exact
        /// same code path deterministically without entering play mode.
        /// </summary>
        public void Step(float dt)
        {
            var agents = environment.DroneAgents;
            var engine = environment.Navigation;

            // Phase 1 of the step: one snapshot of the whole fleet, taken before
            // anybody moves. Every drone reacts to this same picture.
            _snapshots.Clear();
            for (int i = 0; i < agents.Count; i++)
            {
                agents[i].EnsureData();
                _snapshots.Add(agents[i].Follower.Snapshot());
            }

            // Only dynamic hazards participate in local avoidance. Static rubble is
            // already baked into the grid, so the planned route avoids it and
            // repelling from it again would just fight the route.
            RefreshDynamicObstacles();

            List<AgentData> neighbours = enableLocalAvoidance ? _snapshots : null;
            List<Obstacle> obstacles = enableLocalAvoidance ? _dynamicObstacles : null;

            // Phase 2 of the step: decide velocities. Nothing moves yet.
            for (int i = 0; i < agents.Count; i++)
                agents[i].Follower.ComputeVelocity(engine, neighbours, obstacles, dt);

            // Phase 3 of the step: move, then publish to transforms and shared state.
            for (int i = 0; i < agents.Count; i++)
            {
                agents[i].Follower.Integrate(dt);
                agents[i].SyncFromFollower();
            }

            SimulatedTime += dt;
        }

        private void RefreshDynamicObstacles()
        {
            _dynamicObstacles.Clear();
            if (environment.Grid == null)
                return;

            var all = environment.Grid.Obstacles;
            for (int i = 0; i < all.Count; i++)
            {
                if (all[i].isDynamic)
                    _dynamicObstacles.Add(all[i]);
            }
        }

        /// <summary>Used by the editor-time harness to bind the environment without a scene reference.</summary>
        public void EditorAssign(DisasterEnvironment env)
        {
            environment = env;
        }
    }
}
