using DroneRescue.Fleet;

namespace DroneRescue.Planning
{
    /// <summary>Why the hard filter rejected a drone, or that it did not.</summary>
    public enum FilterVerdict
    {
        /// <summary>Passed both hard filters. A candidate for scoring.</summary>
        Feasible = 0,

        /// <summary>Failed drone.status == Idle.</summary>
        NotIdle = 1,

        /// <summary>Failed totalDistance &lt;= range(batteryPercent).</summary>
        OutOfRange = 2
    }

    /// <summary>
    /// One drone measured against one patient.
    ///
    /// Phase 4 fills only the distance and filter fields. The scoring fields stay
    /// at their defaults until Phase 5 adds Step 3; they live here so the record
    /// a drone is judged by is one object rather than two, which is what the
    /// dashboard's explainability panel in Phase 8 needs to display.
    /// </summary>
    public struct DroneEvaluation
    {
        public Drone drone;

        /// <summary>Distance(drone, patient), measured on the ground plane.</summary>
        public float distanceToPatient;

        /// <summary>Distance(patient, nearestHospital), measured on the ground plane.</summary>
        public float distanceToHospital;

        /// <summary>The Part 1 totalDistance: drone to patient, then patient to hospital.</summary>
        public float totalDistance;

        /// <summary>range = (batteryPercent / 100) * MAX_RANGE.</summary>
        public float range;

        public FilterVerdict verdict;

        public bool Feasible => verdict == FilterVerdict.Feasible;

        /// <summary>How much of the remaining range this mission would consume.</summary>
        public float RangeUsedFraction => range > 0f ? totalDistance / range : float.PositiveInfinity;

        /// <summary>Range left over after the mission. Negative means infeasible.</summary>
        public float SpareRange => range - totalDistance;

        public string Explain()
        {
            switch (verdict)
            {
                case FilterVerdict.NotIdle:
                    return drone.id + " EXCLUDED: status is " + drone.status + ", not Idle";
                case FilterVerdict.OutOfRange:
                    return drone.id + " EXCLUDED: needs " + totalDistance.ToString("F1")
                           + "u but battery " + drone.batteryPercent.ToString("F0")
                           + "% gives only " + range.ToString("F1") + "u";
                default:
                    return drone.id + " feasible: needs " + totalDistance.ToString("F1")
                           + "u of " + range.ToString("F1") + "u available, spare "
                           + SpareRange.ToString("F1") + "u";
            }
        }
    }
}
