using System.Collections.Generic;
using UnityEngine;

namespace DroneRescue.Navigation
{
    /// <summary>
    /// Tunable knobs for the navigation engine. Kept as a serializable class so a
    /// scene component can expose them in the inspector without the engine itself
    /// depending on MonoBehaviour.
    /// </summary>
    [System.Serializable]
    public class NavigationSettings
    {
        [Header("Global path search")]
        [Tooltip("Multiplier on the clearance penalty. Higher = routes hug the middle of open space more strongly, at the cost of longer paths.")]
        public float clearanceWeight = 1f;

        [Tooltip("Weight on the straight-line heuristic. 0 turns the search into pure Dijkstra.")]
        public float heuristicWeight = 1f;

        [Tooltip("Drop redundant waypoints when the straight line between them is clear.")]
        public bool simplifyPath = true;

        [Tooltip("How far outward to search for a walkable cell when start or goal sits inside an obstacle.")]
        public int nearestWalkableSearchRings = 12;

        [Header("Local avoidance")]
        [Tooltip("Agents and obstacles beyond this world distance are ignored.")]
        public float neighbourRadius = 8f;

        [Tooltip("Strength of the inverse-distance repulsion from other agents.")]
        public float agentRepulsionWeight = 6f;

        [Tooltip("Strength of the inverse-distance repulsion from dynamic obstacles.")]
        public float obstacleRepulsionWeight = 10f;

        [Tooltip("Extra separation kept on top of the two agents' radii.")]
        public float safetyMargin = 1f;

        [Tooltip("Within this distance of the waypoint the preferred speed tapers to zero.")]
        public float arrivalRadius = 1.5f;
    }

    /// <summary>
    /// Stateless-per-call navigation service. Both entry points are safe to call
    /// repeatedly, for arbitrary start/goal pairs and arbitrary agent counts.
    /// There is no visualization, no GameObject reference and no per-agent state
    /// held here — a caller owns its own agent state and asks this engine for
    /// answers.
    /// </summary>
    public class NavigationEngine
    {
        private readonly DisasterGrid _grid;
        private readonly NavigationSettings _settings;

        // Scratch buffers reused across FindSafePath calls to avoid per-dispatch allocation.
        private float[] _gCost;
        private int[] _cameFrom;
        private bool[] _closed;
        private MinHeap _open;

        public NavigationEngine(DisasterGrid grid, NavigationSettings settings = null)
        {
            _grid = grid;
            _settings = settings ?? new NavigationSettings();

            int cells = grid.Width * grid.Height;
            _gCost = new float[cells];
            _cameFrom = new int[cells];
            _closed = new bool[cells];
            _open = new MinHeap(cells);
        }

        public NavigationSettings Settings => _settings;
        public DisasterGrid Grid => _grid;

        // =====================================================================
        // PUBLIC ENTRY POINT 1 — global path
        // =====================================================================

        /// <summary>
        /// Returns a waypoint list from start to goal, or an empty list when no
        /// route exists. Waypoints are world positions on the grid's ground plane.
        /// </summary>
        public List<Vector3> FindSafePath(Vector3 start, Vector3 goal)
        {
            return FindClearanceWeightedPath(start, goal);
        }

        /// <summary>
        /// A* over the occupancy grid where each expansion pays a clearance penalty
        /// on top of its movement cost, so the search is pulled toward cells that
        /// sit far from any obstacle.
        ///
        /// APPROXIMATION NOTE: this approximates Voronoi-style maximum-clearance
        /// routing WITHOUT extracting an explicit Voronoi skeleton or roadmap. There
        /// are no Voronoi edges anywhere in this code. The brushfire distance
        /// transform in DisasterGrid plays the role the Voronoi diagram would play:
        /// cells equidistant from two obstacles hold locally maximal clearance, and
        /// penalising low clearance makes the optimal path prefer them.
        /// </summary>
        public List<Vector3> FindClearanceWeightedPath(Vector3 start, Vector3 goal)
        {
            var path = new List<Vector3>();

            if (!_grid.TryFindNearestWalkable(start, _settings.nearestWalkableSearchRings, out var sx, out var sy))
                return path;
            if (!_grid.TryFindNearestWalkable(goal, _settings.nearestWalkableSearchRings, out var gx, out var gy))
                return path;

            int startIdx = _grid.Index(sx, sy);
            int goalIdx = _grid.Index(gx, gy);

            if (startIdx == goalIdx)
            {
                path.Add(_grid.CellToWorld(gx, gy));
                return path;
            }

            for (int i = 0; i < _gCost.Length; i++)
            {
                _gCost[i] = float.PositiveInfinity;
                _cameFrom[i] = -1;
                _closed[i] = false;
            }

            _open.Clear();
            _gCost[startIdx] = 0f;
            _open.Push(startIdx, Heuristic(sx, sy, gx, gy));

            bool found = false;

            while (_open.Count > 0)
            {
                int current = _open.Pop();
                if (_closed[current])
                    continue;
                _closed[current] = true;

                if (current == goalIdx)
                {
                    found = true;
                    break;
                }

                int cx = current % _grid.Width;
                int cy = current / _grid.Width;

                for (int dy = -1; dy <= 1; dy++)
                {
                    for (int dx = -1; dx <= 1; dx++)
                    {
                        if (dx == 0 && dy == 0)
                            continue;

                        int nx = cx + dx;
                        int ny = cy + dy;
                        if (!_grid.IsWalkable(nx, ny))
                            continue;

                        // Disallow cutting a diagonal between two blocked cells.
                        if (dx != 0 && dy != 0 &&
                            (!_grid.IsWalkable(cx + dx, cy) || !_grid.IsWalkable(cx, cy + dy)))
                            continue;

                        int neighbour = _grid.Index(nx, ny);
                        if (_closed[neighbour])
                            continue;

                        float stepCost = (dx != 0 && dy != 0) ? 1.41421356f : 1f;
                        float tentative = _gCost[current] + stepCost + ClearancePenalty(nx, ny);

                        if (tentative >= _gCost[neighbour])
                            continue;

                        _gCost[neighbour] = tentative;
                        _cameFrom[neighbour] = current;
                        _open.Push(neighbour, tentative + Heuristic(nx, ny, gx, gy));
                    }
                }
            }

            if (!found)
                return path;

            // Walk the parent chain back and reverse.
            for (int idx = goalIdx; idx != -1; idx = _cameFrom[idx])
            {
                path.Add(_grid.CellToWorld(idx % _grid.Width, idx / _grid.Width));
                if (idx == startIdx)
                    break;
            }
            path.Reverse();

            if (_settings.simplifyPath)
                path = SimplifyPath(path);

            return path;
        }

        /// <summary>
        /// Cost added for routing through a cell close to an obstacle.
        /// clearancePenalty = max(0, gridWidth - clearance) * clearanceWeight,
        /// so a cell touching an obstacle pays close to the full grid width while a
        /// cell in the middle of open space pays almost nothing.
        /// </summary>
        private float ClearancePenalty(int x, int y)
        {
            int clearance = _grid.GetClearance(x, y);
            return Mathf.Max(0, _grid.Width - clearance) * _settings.clearanceWeight;
        }

        private float Heuristic(int x, int y, int gx, int gy)
        {
            if (_settings.heuristicWeight <= 0f)
                return 0f;

            float dx = gx - x;
            float dy = gy - y;
            return Mathf.Sqrt(dx * dx + dy * dy) * _settings.heuristicWeight;
        }

        /// <summary>
        /// Removes intermediate waypoints whose predecessor can reach their successor
        /// in a straight line, which turns the staircase of a grid path into a few
        /// long legs.
        ///
        /// The shortcut test is CLEARANCE-AWARE on purpose. A plain line-of-sight
        /// test would happily straighten the route back onto cells that hug an
        /// obstacle, throwing away exactly the safety margin the clearance-weighted
        /// search just paid for. So a shortcut is only taken when the straight line
        /// is at least as clear as the sub-path it replaces.
        /// </summary>
        private List<Vector3> SimplifyPath(List<Vector3> path)
        {
            if (path.Count <= 2)
                return path;

            var result = new List<Vector3> { path[0] };
            int anchor = 0;

            // Lowest clearance seen on the original sub-path since the anchor.
            int subPathMinClearance = ClearanceAt(path[0]);

            for (int i = 1; i < path.Count - 1; i++)
            {
                subPathMinClearance = Mathf.Min(subPathMinClearance, ClearanceAt(path[i]));
                int required = Mathf.Min(subPathMinClearance, ClearanceAt(path[i + 1]));

                if (!IsShortcutSafe(path[anchor], path[i + 1], required))
                {
                    result.Add(path[i]);
                    anchor = i;
                    subPathMinClearance = ClearanceAt(path[i]);
                }
            }

            result.Add(path[path.Count - 1]);
            return result;
        }

        /// <summary>
        /// Samples the segment at half-cell steps and reports whether every sample is
        /// walkable and holds at least the required clearance.
        /// </summary>
        public bool IsShortcutSafe(Vector3 from, Vector3 to, int requiredClearance)
        {
            float distance = Vector3.Distance(from, to);
            int steps = Mathf.Max(1, Mathf.CeilToInt(distance / (_grid.CellSize * 0.5f)));

            for (int i = 0; i <= steps; i++)
            {
                var point = Vector3.Lerp(from, to, i / (float)steps);
                if (!_grid.IsWalkable(point))
                    return false;
                if (ClearanceAt(point) < requiredClearance)
                    return false;
            }

            return true;
        }

        /// <summary>Samples the segment at half-cell steps and reports whether every sample is walkable.</summary>
        public bool HasLineOfSight(Vector3 from, Vector3 to) => IsShortcutSafe(from, to, 0);

        private int ClearanceAt(Vector3 world)
        {
            _grid.WorldToCell(world, out var x, out var y);
            return _grid.GetClearance(x, y);
        }

        // =====================================================================
        // PUBLIC ENTRY POINT 2 — local avoidance
        // =====================================================================

        /// <summary>
        /// Returns the velocity this agent should fly this frame, given what it can
        /// currently sense around it.
        ///
        /// IMPLEMENTATION NOTE: the body is an artificial-potential-field style
        /// reciprocal avoidance heuristic, NOT the literal ORCA velocity-obstacle
        /// linear program. It is ORCA-INSPIRED: each agent assumes its neighbours
        /// are running the same rule and therefore takes only half the corrective
        /// effort for each pairwise conflict, which is the reciprocity idea from
        /// van den Berg et al. (2011) without the half-plane LP.
        ///
        /// This signature is the seam. A true ORCA implementation (e.g. a port of
        /// RVO2) can replace the body without any caller changing, so the swap
        /// stays a one-file change.
        /// </summary>
        public Vector3 ComputeAvoidanceVelocity(AgentData self, List<AgentData> neighbors, List<Obstacle> obstacles)
        {
            Vector3 preferred = ComputePreferredVelocity(self);
            Vector3 repulsion = Vector3.zero;

            float senseRadius = _settings.neighbourRadius;

            if (neighbors != null)
            {
                for (int i = 0; i < neighbors.Count; i++)
                {
                    var other = neighbors[i];
                    if (other.id == self.id)
                        continue;

                    Vector3 away = self.position - other.position;
                    away.y = 0f;
                    float distance = away.magnitude;

                    if (distance > senseRadius)
                        continue;

                    float minSeparation = self.radius + other.radius + _settings.safetyMargin;

                    // Perfectly co-located agents would produce a zero direction, so
                    // break the tie deterministically using the id ordering.
                    if (distance < 0.0001f)
                    {
                        away = new Vector3(self.id < other.id ? 1f : -1f, 0f, 0f);
                        distance = 0.0001f;
                    }

                    Vector3 direction = away / distance;
                    float intrusion = Mathf.Max(0f, senseRadius - distance);
                    float urgency = intrusion / Mathf.Max(0.0001f, distance);

                    // Extra kick once the pair is actually inside its separation distance.
                    if (distance < minSeparation)
                        urgency += (minSeparation - distance) / Mathf.Max(0.0001f, distance);

                    // Half share: the neighbour is expected to take the other half.
                    repulsion += direction * urgency * _settings.agentRepulsionWeight * 0.5f;
                }
            }

            if (obstacles != null)
            {
                for (int i = 0; i < obstacles.Count; i++)
                {
                    var obstacle = obstacles[i];

                    // Measure from the nearest point of the footprint, not the centre,
                    // so a long boxed building repels along its face rather than
                    // pretending to be a circle around its middle.
                    Vector3 closest = obstacle.ClosestPoint(self.position);
                    Vector3 away = self.position - closest;
                    away.y = 0f;
                    float edgeDistance = away.magnitude;
                    float surfaceDistance = edgeDistance - self.radius;

                    if (surfaceDistance > senseRadius)
                        continue;

                    // Already inside the footprint: push straight out from the centre.
                    if (edgeDistance < 0.0001f)
                    {
                        away = self.position - obstacle.center;
                        away.y = 0f;
                        if (away.sqrMagnitude < 0.0001f)
                            away = Vector3.right;
                        edgeDistance = away.magnitude;
                        surfaceDistance = 0f;
                    }

                    Vector3 direction = away / edgeDistance;
                    float effective = Mathf.Max(0.25f, surfaceDistance);
                    float urgency = Mathf.Max(0f, senseRadius - surfaceDistance) / effective;

                    // An obstacle does not move out of the way, so no half share here.
                    repulsion += direction * urgency * _settings.obstacleRepulsionWeight;
                }
            }

            Vector3 result = preferred + repulsion;
            result.y = 0f;

            if (result.sqrMagnitude > self.maxSpeed * self.maxSpeed)
                result = result.normalized * self.maxSpeed;

            return result;
        }

        /// <summary>Straight-line pull toward the current waypoint, tapering off on arrival.</summary>
        private Vector3 ComputePreferredVelocity(AgentData self)
        {
            Vector3 toGoal = self.goal - self.position;
            toGoal.y = 0f;
            float distance = toGoal.magnitude;

            if (distance < 0.0001f)
                return Vector3.zero;

            float speed = self.maxSpeed;
            if (distance < _settings.arrivalRadius)
                speed *= distance / _settings.arrivalRadius;

            return (toGoal / distance) * speed;
        }

        // =====================================================================
        // Binary min-heap keyed by float priority.
        // =====================================================================

        private class MinHeap
        {
            private int[] _items;
            private float[] _priorities;
            private int _count;

            public MinHeap(int capacity)
            {
                capacity = Mathf.Max(16, capacity);
                _items = new int[capacity];
                _priorities = new float[capacity];
                _count = 0;
            }

            public int Count => _count;

            public void Clear() => _count = 0;

            public void Push(int item, float priority)
            {
                if (_count == _items.Length)
                {
                    System.Array.Resize(ref _items, _count * 2);
                    System.Array.Resize(ref _priorities, _count * 2);
                }

                _items[_count] = item;
                _priorities[_count] = priority;

                int child = _count++;
                while (child > 0)
                {
                    int parent = (child - 1) / 2;
                    if (_priorities[parent] <= _priorities[child])
                        break;
                    Swap(parent, child);
                    child = parent;
                }
            }

            public int Pop()
            {
                int result = _items[0];
                _count--;
                _items[0] = _items[_count];
                _priorities[0] = _priorities[_count];

                int parent = 0;
                while (true)
                {
                    int left = parent * 2 + 1;
                    int right = left + 1;
                    int smallest = parent;

                    if (left < _count && _priorities[left] < _priorities[smallest])
                        smallest = left;
                    if (right < _count && _priorities[right] < _priorities[smallest])
                        smallest = right;
                    if (smallest == parent)
                        break;

                    Swap(parent, smallest);
                    parent = smallest;
                }

                return result;
            }

            private void Swap(int a, int b)
            {
                (_items[a], _items[b]) = (_items[b], _items[a]);
                (_priorities[a], _priorities[b]) = (_priorities[b], _priorities[a]);
            }
        }
    }
}
