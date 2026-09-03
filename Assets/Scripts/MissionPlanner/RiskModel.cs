using UnityEngine;
using DroneRescue.Navigation;

namespace DroneRescue.Planning
{
    /// <summary>
    /// RiskFactor(location) from Part 1 Step 3: how dangerous it is to fly a drone
    /// to a given place.
    ///
    /// DELIBERATELY NOT Patient.priority. Priority is a triage category and it ranks
    /// PATIENTS in the queue. This ranks PLACES, and it is computed from the map
    /// alone: a patient's own triage category never enters this calculation, and a
    /// location's risk never enters the queue ordering. The two stay separate all
    /// the way through, which is the naming rule in Part 1.
    ///
    /// Two things make a location risky here:
    ///
    ///  1. FIRE PROXIMITY. Dynamic obstacles are the fire fronts. Risk is 1 inside
    ///     one and falls off linearly to 0 at fireInfluenceRadius. This is the term
    ///     Part 1 names explicitly, and it is the one Phase 6's FireSpread event
    ///     will start moving.
    ///
    ///  2. CONFINEMENT. The clearance map already knows how much open air surrounds
    ///     every cell, so a spot wedged between collapsed buildings scores higher
    ///     than the same spot out on a clear road. It is capped below the fire term
    ///     by confinementWeight, because rubble is an obstacle whereas fire is a
    ///     hazard.
    ///
    /// The result is always 0 to 1, so W3 alone decides how much risk is worth
    /// relative to the other two score terms.
    /// </summary>
    [System.Serializable]
    public class RiskModel
    {
        [Tooltip("Distance from a fire front at which its risk contribution reaches zero.")]
        [Min(0.01f)] public float fireInfluenceRadius = 20f;

        [Tooltip("Clearance in grid cells at which a location counts as fully open ground.")]
        [Min(1)] public int openClearanceCells = 12;

        [Tooltip("Risk contributed by being hemmed in by rubble, at zero clearance. " +
                 "Below 1 so that being near a fire always outranks being in a tight spot.")]
        [Range(0f, 1f)] public float confinementWeight = 0.5f;

        /// <summary>
        /// Risk of the given world location, 0 (open and clear) to 1 (in a fire).
        /// Returns 0 when there is no grid to measure against, so a missing map
        /// cannot silently bias the scoring.
        /// </summary>
        public float Evaluate(DisasterGrid grid, Vector3 location)
        {
            if (grid == null)
                return 0f;

            return Mathf.Clamp01(Mathf.Max(FireRisk(grid, location), ConfinementRisk(grid, location)));
        }

        /// <summary>Proximity to the nearest fire front, as a 0 to 1 value.</summary>
        public float FireRisk(DisasterGrid grid, Vector3 location)
        {
            if (grid == null)
                return 0f;

            float worst = 0f;
            var obstacles = grid.Obstacles;

            for (int i = 0; i < obstacles.Count; i++)
            {
                // Static rubble is handled by the confinement term. Only the
                // dynamic hazards, the fire fronts, count as fire.
                if (!obstacles[i].isDynamic)
                    continue;

                if (obstacles[i].Contains(location))
                    return 1f;

                float distance = FlatDistance(location, obstacles[i].ClosestPoint(location));
                float risk = 1f - distance / fireInfluenceRadius;

                if (risk > worst)
                    worst = risk;
            }

            return Mathf.Clamp01(worst);
        }

        /// <summary>How enclosed the location is, read straight off the clearance map.</summary>
        public float ConfinementRisk(DisasterGrid grid, Vector3 location)
        {
            if (grid == null)
                return 0f;

            grid.WorldToCell(location, out int x, out int y);
            if (!grid.InBounds(x, y))
                return 0f;

            // A grid with no obstacles at all leaves unreached cells at int.MaxValue,
            // so clamp before dividing rather than trusting the raw cell value.
            int clearance = Mathf.Min(grid.GetClearance(x, y), openClearanceCells);
            float enclosed = 1f - (float)clearance / openClearanceCells;

            return Mathf.Clamp01(enclosed) * confinementWeight;
        }

        private static float FlatDistance(Vector3 a, Vector3 b)
        {
            float dx = a.x - b.x;
            float dz = a.z - b.z;
            return Mathf.Sqrt(dx * dx + dz * dz);
        }
    }
}
