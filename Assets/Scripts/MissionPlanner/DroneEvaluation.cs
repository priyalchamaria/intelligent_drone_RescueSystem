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
    /// One drone measured against one patient: the Step 2 filter verdict, and for
    /// the survivors the Step 3 score with the three factors that produced it.
    ///
    /// The factors are kept alongside the total rather than being discarded, so a
    /// dispatch can always be explained after the fact. Phase 8's explainability
    /// panel reads exactly these fields.
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

        // -----------------------------------------------------------------
        // Step 3 scoring. Filled only for drones that survived Step 2.
        // -----------------------------------------------------------------

        /// <summary>
        /// totalDistance / range. THIS IS THE BATTERY PENALTY: a continuous ratio,
        /// not a second cutoff. Near 0 means the drone has range to spare. Near 1
        /// means the mission barely fits, and the drone is penalised heavily for it
        /// even though Step 2 already declared it feasible.
        /// </summary>
        public float batteryUtilization;

        /// <summary>RiskFactor(patient.location). Nothing to do with the patient's triage priority.</summary>
        public float riskFactor;

        /// <summary>W1 * totalDistance. Lowest total score wins.</summary>
        public float distanceTerm;

        /// <summary>W2 * batteryUtilization.</summary>
        public float batteryTerm;

        /// <summary>W3 * riskFactor.</summary>
        public float riskTerm;

        /// <summary>The Part 1 Step 3 score. Lowest wins.</summary>
        public float score;

        /// <summary>True once Step 3 has run on this record.</summary>
        public bool scored;

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
                    return scored
                        ? ExplainScore()
                        : drone.id + " feasible: needs " + totalDistance.ToString("F1")
                          + "u of " + range.ToString("F1") + "u available, spare "
                          + SpareRange.ToString("F1") + "u";
            }
        }

        /// <summary>
        /// The score with its three factors spelled out, so a dispatch decision can
        /// be read rather than guessed at. Same numbers Phase 8's explainability
        /// panel will show.
        /// </summary>
        public string ExplainScore()
        {
            return drone.id + " score " + score.ToString("F1")
                   + " = " + distanceTerm.ToString("F1") + " dist(" + totalDistance.ToString("F1") + "u)"
                   + " + " + batteryTerm.ToString("F1") + " batt(" + batteryUtilization.ToString("F2") + ")"
                   + " + " + riskTerm.ToString("F1") + " risk(" + riskFactor.ToString("F2") + ")";
        }
    }
}
