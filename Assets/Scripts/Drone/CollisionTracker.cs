using System.Collections.Generic;
using UnityEngine;
using DroneRescue.Navigation;

namespace DroneRescue.Fleet
{
    /// <summary>
    /// Watches the fleet each step and records how close drones actually came to
    /// each other.
    ///
    /// Stage 1 carried a single collisionFlag it never actually set, so there was no
    /// real collision measurement. This replaces it with something countable, which
    /// Part 3 metric 5 needs.
    ///
    /// A collision is counted once per encounter, not once per frame: a pair that
    /// stays overlapped for 200 steps is one collision, not 200. The pair has to
    /// separate again before it can count a second time.
    /// </summary>
    public class CollisionTracker
    {
        private readonly HashSet<long> _overlapping = new HashSet<long>();

        /// <summary>Distinct overlap events since the last Reset.</summary>
        public int CollisionCount { get; private set; }

        /// <summary>
        /// Smallest surface-to-surface gap observed, in world units. Negative means
        /// the drone bodies actually interpenetrated at some point.
        /// </summary>
        public float MinSeparation { get; private set; } = float.PositiveInfinity;

        /// <summary>Ids of the closest pair seen, for reporting.</summary>
        public int ClosestPairA { get; private set; } = -1;
        public int ClosestPairB { get; private set; } = -1;

        /// <summary>Pairs currently overlapping.</summary>
        public int ActiveOverlaps => _overlapping.Count;

        public void Reset()
        {
            _overlapping.Clear();
            CollisionCount = 0;
            MinSeparation = float.PositiveInfinity;
            ClosestPairA = -1;
            ClosestPairB = -1;
        }

        /// <summary>
        /// Records one step. Only drones that are actually flying are considered:
        /// six parked drones sitting on two charging pads are not a collision, and
        /// counting them as one would make the metric meaningless.
        /// </summary>
        public void Observe(IReadOnlyList<AgentData> agents, IReadOnlyList<bool> isFlying)
        {
            for (int i = 0; i < agents.Count; i++)
            {
                if (isFlying != null && !isFlying[i])
                    continue;

                for (int j = i + 1; j < agents.Count; j++)
                {
                    if (isFlying != null && !isFlying[j])
                        continue;

                    var a = agents[i];
                    var b = agents[j];

                    float dx = a.position.x - b.position.x;
                    float dz = a.position.z - b.position.z;
                    float distance = Mathf.Sqrt(dx * dx + dz * dz);
                    float gap = distance - (a.radius + b.radius);

                    if (gap < MinSeparation)
                    {
                        MinSeparation = gap;
                        ClosestPairA = a.id;
                        ClosestPairB = b.id;
                    }

                    long key = PairKey(a.id, b.id);
                    bool overlapping = gap < 0f;

                    if (overlapping)
                    {
                        // Count the encounter only on the transition into overlap.
                        if (_overlapping.Add(key))
                            CollisionCount++;
                    }
                    else
                    {
                        _overlapping.Remove(key);
                    }
                }
            }
        }

        private static long PairKey(int a, int b)
        {
            int low = Mathf.Min(a, b);
            int high = Mathf.Max(a, b);
            return ((long)low << 32) | (uint)high;
        }
    }
}
