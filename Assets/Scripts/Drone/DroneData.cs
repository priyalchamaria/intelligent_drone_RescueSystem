using UnityEngine;

namespace DroneRescue.Fleet
{
    /// <summary>Lifecycle state of a drone, exactly as specified in Part 1.</summary>
    public enum DroneStatus
    {
        Idle = 0,
        Busy = 1,
        Charging = 2,
        Offline = 3
    }

    /// <summary>
    /// The Drone record from Part 1: id, location, batteryPercent, status.
    ///
    /// This is the drone's entry in the shared state that the Mission Planner
    /// reads. It is a plain class, not a MonoBehaviour: the scene object
    /// (DroneAgent) owns one of these and republishes its live values into it
    /// every frame, which simulates a status uplink. The Mission Planner never
    /// touches the GameObject, only this record.
    /// </summary>
    [System.Serializable]
    public class Drone
    {
        public string id;
        public Vector3 location;
        public float batteryPercent;
        public DroneStatus status;

        public Drone(string id, Vector3 location, float batteryPercent = 100f, DroneStatus status = DroneStatus.Idle)
        {
            this.id = id;
            this.location = location;
            this.batteryPercent = Mathf.Clamp(batteryPercent, 0f, 100f);
            this.status = status;
        }

        /// <summary>
        /// How far this drone can still fly, given its charge.
        /// range = (batteryPercent / 100) * MAX_RANGE, per the Part 1 hard filter.
        /// MAX_RANGE is supplied by the caller so it stays a tunable mission
        /// parameter rather than a constant buried in the data class.
        /// </summary>
        public float Range(float maxRange) => (batteryPercent / 100f) * maxRange;

        public override string ToString() =>
            $"{id} [{status}] battery={batteryPercent:F0}% at {location}";
    }
}
