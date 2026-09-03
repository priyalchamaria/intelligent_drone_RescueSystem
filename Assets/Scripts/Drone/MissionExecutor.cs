using UnityEngine;
using DroneRescue.Environment;
using DroneRescue.Planning;

namespace DroneRescue.Fleet
{
    /// <summary>
    /// The drone side of the dispatch channel.
    ///
    /// It reads the shared dispatch board, gives each drone the route its
    /// assignment is carrying, and writes back when the drone gets there. That is
    /// the whole job: it decides nothing. Which drone flies where was settled by the
    /// Mission Planner's scoring, and how it flies is the navigation engine's.
    ///
    /// WHY THIS EXISTS AT ALL: it is the piece that keeps the planner away from
    /// GameObjects. Without it the planner would have to reach into a DroneAgent to
    /// hand over a route, and the hub-and-spoke model would only be a comment. With
    /// it, orders go down through shared state and arrivals come back up the same
    /// way, matching how drone status already reaches the planner.
    ///
    /// It runs in LateUpdate so that within one frame the order is always: planner
    /// decides, fleet simulator flies, executor reconciles.
    ///
    /// NOT A COMMUNICATION CHANNEL BETWEEN DRONES. Each assignment is read only by
    /// the drone it names. No drone learns anything about another one here.
    /// </summary>
    [DisallowMultipleComponent]
    public class MissionExecutor : MonoBehaviour
    {
        [SerializeField] private DisasterEnvironment environment;

        [SerializeField] private bool verbose;

        private void Awake()
        {
            if (environment == null)
                environment = FindAnyObjectByType<DisasterEnvironment>();
        }

        private void LateUpdate()
        {
            if (environment == null)
                return;

            var board = environment.Assignments;

            for (int i = 0; i < board.Count; i++)
            {
                var mission = board[i];
                if (mission.IsFinished)
                    continue;

                var agent = FindAgent(mission.droneId);
                if (agent == null)
                    continue;

                // A route on the board means a new order. Take it, clear it, fly it.
                if (mission.pendingRoute != null)
                {
                    agent.AssignRoute(mission.pendingRoute);
                    mission.pendingRoute = null;
                    mission.arrived = false;
                    mission.state = MissionState.EnRoute;

                    if (verbose)
                        Debug.Log("[Executor] " + mission.droneId + " accepted route for leg " + mission.leg + ".");

                    continue;
                }

                // Report arrival exactly once per leg. Only a mission that is
                // actually flying can arrive: a drone with no route reports
                // Finished from the first simulation step, which would otherwise
                // read as an instant arrival.
                if (mission.state == MissionState.EnRoute && !mission.arrived
                    && agent.Follower != null && agent.Follower.Finished)
                {
                    mission.arrived = true;

                    if (verbose)
                        Debug.Log("[Executor] " + mission.droneId + " finished leg " + mission.leg + ".");
                }
            }
        }

        private DroneAgent FindAgent(string droneId)
        {
            var agents = environment.DroneAgents;
            for (int i = 0; i < agents.Count; i++)
                if (agents[i].DroneId == droneId)
                    return agents[i];
            return null;
        }

#if UNITY_EDITOR
        public void EditorAssign(DisasterEnvironment env)
        {
            environment = env;
        }
#endif
    }
}
