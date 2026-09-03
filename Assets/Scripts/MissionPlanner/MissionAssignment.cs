using System.Collections.Generic;
using UnityEngine;
using DroneRescue.Environment;
using DroneRescue.Fleet;

namespace DroneRescue.Planning
{
    /// <summary>Which leg of a rescue a drone is currently flying.</summary>
    public enum MissionLeg
    {
        /// <summary>Drone to patient. Part 1 Step 4, first half.</summary>
        ToPatient = 0,

        /// <summary>Patient to the nearest hospital. Part 1 Step 4, second half.</summary>
        ToHospital = 1,

        /// <summary>Optional third leg: a drone that finished low on charge flies to a pad.</summary>
        ToChargingStation = 2
    }

    public enum MissionState
    {
        /// <summary>The planner has written a route; the drone has not picked it up yet.</summary>
        AwaitingRoute = 0,

        /// <summary>The drone is flying the current leg.</summary>
        EnRoute = 1,

        /// <summary>Every leg finished.</summary>
        Complete = 2,

        /// <summary>Given up on, usually because no route could be found.</summary>
        Aborted = 3
    }

    /// <summary>
    /// One dispatch command, living in shared state.
    ///
    /// THIS IS THE DOWNLINK HALF OF THE HUB-AND-SPOKE MODEL. The Mission Planner
    /// never calls a method on a drone GameObject. It scores, picks a winner, plans
    /// the route, and writes this record into DisasterEnvironment.Assignments. The
    /// drone side (MissionExecutor) reads the board and flies what it finds, then
    /// writes its arrival back into the same record. State goes down and status
    /// comes up through shared data, exactly as the uplink does, and no drone ever
    /// reads another drone's assignment.
    ///
    /// It also gives Phase 6 something concrete to reassign: MissionPlanner.Reassign
    /// takes one of these, and Phase 8's explainability panel reads the decision
    /// stored on it.
    /// </summary>
    public class MissionAssignment
    {
        /// <summary>The drone's shared-state record. Status writes go here, not to a GameObject.</summary>
        public Drone drone;

        /// <summary>Id of that drone. The executor matches on this rather than on an object reference.</summary>
        public string droneId;

        public Patient patient;

        public MissionLeg leg = MissionLeg.ToPatient;
        public MissionState state = MissionState.AwaitingRoute;

        /// <summary>
        /// Route the planner wants flown. The executor consumes it and clears it,
        /// so a non-null value always means "there is a new order waiting".
        /// </summary>
        public List<Vector3> pendingRoute;

        /// <summary>
        /// Where the current leg is headed. Kept separate from the route because a
        /// FireSpread re-path has to aim at the same destination while throwing the
        /// old waypoints away.
        /// </summary>
        public Vector3 legGoal;

        /// <summary>Set by the executor when the current leg's last waypoint is reached.</summary>
        public bool arrived;

        /// <summary>
        /// Tells the drone side to drop what it is flying. Set when a mission is
        /// reassigned out from under a drone: without it the drone would keep
        /// flying to a patient that somebody else is now on the way to.
        /// </summary>
        public bool cancelRoute;

        /// <summary>Simulated time this mission was first dispatched. Feeds Phase 7's response time.</summary>
        public float dispatchedAtTime;

        /// <summary>Which stand on the hospital landing ring this drone is aiming for.</summary>
        public int landingSlot;

        /// <summary>The winning evaluation, kept so the dispatch can be explained after the fact.</summary>
        public DroneEvaluation decision;

        public bool IsFinished => state == MissionState.Complete || state == MissionState.Aborted;

        public override string ToString() =>
            droneId + " -> " + (patient != null ? patient.id : "?") + " [" + leg + "/" + state + "]";
    }
}
