using System.Globalization;
using System.Text;

namespace DroneRescue.Metrics
{
    /// <summary>
    /// The eight Part 3 metrics for one mission run, plus enough context to tell
    /// two runs apart.
    ///
    /// A plain record with no Unity types and no behaviour beyond formatting. It is
    /// produced by MissionRecorder from shared state at the moment a run finishes,
    /// and consumed by the CSV writer and the console summary. Keeping it inert
    /// means the numbers can be computed once and then written to as many places as
    /// Phase 8 turns out to need without recomputing anything.
    ///
    /// EVERY FIELD IS A MEASUREMENT, NOT A JUDGEMENT. Nothing here decides whether
    /// a run went well; it records what happened so that a baseline comparison in
    /// Phase 9 can be made against numbers rather than impressions.
    /// </summary>
    public class MissionMetrics
    {
        // Run identity ----------------------------------------------------
        public string runId = "";
        public string startedAtUtc = "";
        public string scenarioName = "";

        /// <summary>Which dispatch policy produced this run. Phase 9 adds the baseline value.</summary>
        public string dispatchMode = "Scored";

        public int droneCount;
        public int patientCount;
        public int dispatchCount;

        // 1. Mission completion time -------------------------------------

        /// <summary>
        /// Wall-clock milliseconds, measured with a Stopwatch exactly as Stage 1
        /// measured its single run. Kept because it is the figure Stage 1 reported
        /// and the two stages should be comparable.
        /// </summary>
        public double missionTimeMs;

        /// <summary>
        /// Simulated seconds from the first dispatch to the last mission finishing.
        /// This, not the wall clock, is the honest figure for the mission itself:
        /// it does not move if the editor drops frames or the machine is busy.
        /// </summary>
        public float missionTimeSeconds;

        // 2. Average response time ---------------------------------------

        /// <summary>Mean of (drone arrival - queue entry) over every casualty a drone reached.</summary>
        public float averageResponseSeconds;

        /// <summary>How many casualties that mean is over. A mean of one is not a mean.</summary>
        public int responseSamples;

        public float worstResponseSeconds;

        // 3. Rescue success rate -----------------------------------------
        public int deliveredCount;
        public float rescueSuccessRatePercent;

        // 4. Battery at mission end --------------------------------------
        public float fleetAvgBatteryRemainingPercent;
        public float minBatteryRemainingPercent;

        /// <summary>Mean charge spent across the fleet. The complement of the average above.</summary>
        public float fleetAvgBatteryUsedPercent;

        // 5. Collisions ---------------------------------------------------
        public int collisionCount;

        /// <summary>Closest any two drone bodies came, in world units. Zero or less is a collision.</summary>
        public float minSeparation;

        // 6. Reassignments ------------------------------------------------

        /// <summary>Times the planner was re-invoked mid-mission for a task already assigned.</summary>
        public int reassignmentCount;

        // 7. Coverage -----------------------------------------------------
        public int reachedCount;
        public float coveragePercent;

        // 8. Throughput ---------------------------------------------------

        /// <summary>Casualties delivered per minute of simulated time.</summary>
        public float throughputPerMinute;

        private const string Sep = ",";

        public static string CsvHeader()
        {
            return string.Join(Sep, new[]
            {
                "RunId", "StartedAtUtc", "Scenario", "DispatchMode",
                "Drones", "Patients", "Dispatches",
                "MissionTimeMs", "MissionTimeSeconds",
                "AvgResponseSeconds", "ResponseSamples", "WorstResponseSeconds",
                "Delivered", "RescueSuccessRatePercent",
                "FleetAvgBatteryRemainingPercent", "MinBatteryRemainingPercent", "FleetAvgBatteryUsedPercent",
                "Collisions", "MinSeparationUnits",
                "Reassignments",
                "Reached", "CoveragePercent",
                "ThroughputPerMinute"
            });
        }

        /// <summary>
        /// One run as one CSV row.
        ///
        /// InvariantCulture throughout, deliberately. On a machine with a comma
        /// decimal separator the default formatting would write "18,4" and split
        /// one number across two columns, which corrupts the file silently rather
        /// than failing.
        /// </summary>
        public string ToCsvRow()
        {
            var c = CultureInfo.InvariantCulture;

            return string.Join(Sep, new[]
            {
                Escape(runId), Escape(startedAtUtc), Escape(scenarioName), Escape(dispatchMode),
                droneCount.ToString(c), patientCount.ToString(c), dispatchCount.ToString(c),
                missionTimeMs.ToString("F0", c), missionTimeSeconds.ToString("F2", c),
                averageResponseSeconds.ToString("F2", c), responseSamples.ToString(c), worstResponseSeconds.ToString("F2", c),
                deliveredCount.ToString(c), rescueSuccessRatePercent.ToString("F1", c),
                fleetAvgBatteryRemainingPercent.ToString("F1", c), minBatteryRemainingPercent.ToString("F1", c),
                fleetAvgBatteryUsedPercent.ToString("F1", c),
                collisionCount.ToString(c), FormatSeparation(minSeparation),
                reassignmentCount.ToString(c),
                reachedCount.ToString(c), coveragePercent.ToString("F1", c),
                throughputPerMinute.ToString("F2", c)
            });
        }

        /// <summary>
        /// The same eight metrics laid out for a human, one per line, in the order
        /// Part 3 lists them.
        /// </summary>
        public string ToReadableBlock()
        {
            var c = CultureInfo.InvariantCulture;
            var sb = new StringBuilder();

            sb.Append("run ").Append(runId).Append("  |  ").Append(scenarioName)
              .Append("  |  dispatch mode: ").Append(dispatchMode).AppendLine();
            sb.Append("  1. mission completion time     ")
              .Append(missionTimeSeconds.ToString("F1", c)).Append("s simulated (")
              .Append(missionTimeMs.ToString("F0", c)).AppendLine("ms wall clock)");
            sb.Append("  2. average response time       ")
              .Append(averageResponseSeconds.ToString("F1", c)).Append("s over ")
              .Append(responseSamples).Append(" casualties, worst ")
              .Append(worstResponseSeconds.ToString("F1", c)).AppendLine("s");
            sb.Append("  3. rescue success rate         ")
              .Append(rescueSuccessRatePercent.ToString("F0", c)).Append("%  (")
              .Append(deliveredCount).Append(" of ").Append(patientCount).AppendLine(" delivered)");
            sb.Append("  4. battery at mission end      fleet average ")
              .Append(fleetAvgBatteryRemainingPercent.ToString("F0", c)).Append("% remaining, lowest ")
              .Append(minBatteryRemainingPercent.ToString("F0", c)).Append("%, average ")
              .Append(fleetAvgBatteryUsedPercent.ToString("F0", c)).AppendLine("% spent");
            sb.Append("  5. collisions                  ").Append(collisionCount)
              .Append("  (closest approach ").Append(FormatSeparation(minSeparation)).AppendLine(" units)");
            sb.Append("  6. reassignments               ").Append(reassignmentCount)
              .Append("  (of ").Append(dispatchCount).AppendLine(" dispatches)");
            sb.Append("  7. coverage                    ")
              .Append(coveragePercent.ToString("F0", c)).Append("%  (")
              .Append(reachedCount).Append(" of ").Append(patientCount).AppendLine(" reached)");
            sb.Append("  8. system throughput           ")
              .Append(throughputPerMinute.ToString("F2", c)).Append(" casualties per simulated minute");

            return sb.ToString();
        }

        /// <summary>
        /// Separation is infinite when fewer than two drones ever flew at once, and
        /// "Infinity" in a CSV column breaks most spreadsheets. Blank is honest: no
        /// pair was ever observed, so there is no number.
        /// </summary>
        private static string FormatSeparation(float separation)
        {
            return float.IsInfinity(separation) || float.IsNaN(separation)
                ? ""
                : separation.ToString("F2", CultureInfo.InvariantCulture);
        }

        private static string Escape(string value)
        {
            if (string.IsNullOrEmpty(value))
                return "";

            bool needsQuotes = value.IndexOf(',') >= 0 || value.IndexOf('"') >= 0
                               || value.IndexOf('\n') >= 0 || value.IndexOf('\r') >= 0;

            return needsQuotes ? "\"" + value.Replace("\"", "\"\"") + "\"" : value;
        }
    }
}
