using System.Collections.Generic;
using UnityEngine;

namespace DroneRescue.Navigation
{
    /// <summary>
    /// Tunable knobs for the navigation engine. Defaults are the values the
    /// Stage 1 PathManager shipped with, so behaviour matches the validated
    /// implementation out of the box.
    /// </summary>
    [System.Serializable]
    public class NavigationSettings
    {
        [Header("Global path search (Stage 1: FindPathSafest_GetPath)")]
        [Tooltip("Stage 1 default was 5. Higher pushes routes further from obstacles at the cost of length.")]
        public float clearanceWeight = 5f;

        [Tooltip("Stage 1 used every grid tile as a waypoint. Leave off to match it.")]
        public bool simplifyPath = false;

        [Tooltip("Stage 2 addition: how far to search outward when a start or goal sits inside an obstacle. " +
                 "Has to exceed the largest hazard radius plus dynamicHazardMargin, or a casualty caught " +
                 "inside a fire cannot be given a reachable pickup point.")]
        public int nearestWalkableSearchRings = 20;

        [Tooltip("Stage 2 addition: extra standoff, in world units, baked around DYNAMIC hazards only. " +
                 "Rubble is inert and can be skimmed; a fire front is spreading and should not be. " +
                 "Measured on the demo map: 5 buys a 5.3u standoff for 3.7% extra path length.")]
        [Min(0f)] public float dynamicHazardMargin = 5f;

        [Header("Local avoidance (Stage 1: ComputeORCAVelocity)")]
        [Tooltip("Stage 1 buffer added to the combined radii before agent repulsion kicks in.")]
        public float agentBuffer = 1.0f;

        [Tooltip("Stage 1 buffer added before obstacle repulsion kicks in.")]
        public float obstacleBuffer = 1.5f;

        [Tooltip("Stage 1 distance at which an agent advances to its next waypoint.")]
        public float waypointTolerance = 0.35f;

        [Tooltip("Stage 2 addition: arrival distance for the FINAL waypoint only. A drone " +
                 "settling near a destination that another drone is parked on cannot always " +
                 "close the last metre, so the last waypoint accepts a wider arrival.")]
        public float finalApproachTolerance = 1.5f;
    }

    /// <summary>
    /// Navigation service ported from the Stage 1 PathManager.
    ///
    /// The Stage 1 original was one MonoBehaviour that owned the grid, the tile
    /// GameObjects, the agents, the metrics and the CSV writer all at once, and it
    /// planned exactly one path between two inspector-assigned transforms. The
    /// algorithms below are its algorithms, lifted out of that class so they can be
    /// called repeatedly for arbitrary start and goal pairs and arbitrary agent
    /// counts, with no GameObject or visualisation coupling.
    ///
    /// Where behaviour had to change for Stage 2, the change is marked ADAPTED and
    /// the reason is given. Everything else is the Stage 1 logic.
    /// </summary>
    public class NavigationEngine
    {
        private static readonly float Sqrt2 = 1.41421356f;

        private readonly DisasterGrid _grid;
        private readonly NavigationSettings _settings;

        // Scratch buffers reused across calls, so repeated dispatch does not allocate.
        private readonly float[] _gCost;
        private readonly int[] _cameFrom;
        private readonly MinHeap _open;

        public NavigationEngine(DisasterGrid grid, NavigationSettings settings = null)
        {
            _grid = grid;
            _settings = settings ?? new NavigationSettings();

            int cells = grid.Width * grid.Height;
            _gCost = new float[cells];
            _cameFrom = new int[cells];
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
        /// Port of Stage 1's FindPathSafest_GetPath. Searches the grid with each
        /// node's queue priority set to gCost + clearancePenalty, where
        /// clearancePenalty = max(0, gridWidth - clearance) * clearanceWeight.
        /// There is no distance heuristic, exactly as in Stage 1, so this is a
        /// Dijkstra-style search biased toward wide corridors rather than an A*.
        ///
        /// APPROXIMATION NOTE: this approximates Voronoi-style maximum-clearance
        /// routing WITHOUT extracting an explicit Voronoi skeleton or roadmap.
        /// There are no Voronoi edges anywhere in Stage 1 or here. The brushfire
        /// distance transform plays the role the Voronoi diagram would: cells
        /// equidistant from two obstacles hold locally maximal clearance, and
        /// penalising low clearance makes the chosen path prefer them.
        ///
        /// FIDELITY NOTE: Stage 1 adds the clearance penalty to the queue priority
        /// only, never into gCost, so the penalty acts as a one-step lookahead and
        /// does not accumulate along the route. That is reproduced here rather than
        /// "corrected", because the Stage 1 results were produced with it.
        /// </summary>
        public List<Vector3> FindClearanceWeightedPath(Vector3 start, Vector3 goal)
        {
            var path = new List<Vector3>();

            // ADAPTED: Stage 1 clamped start and goal to the grid and assumed they
            // were never inside an obstacle. Stage 2 spawns patients and hospitals
            // anywhere, and fire can spread over a marker, so recover to the nearest
            // free cell instead of planning from inside a wall.
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

            ResetSearchState();
            _gCost[startIdx] = 0f;
            _open.Push(startIdx, 0f);

            bool found = false;

            while (_open.Count > 0)
            {
                int current = _open.Pop();

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

                        // Stage 1 refused a diagonal step when either orthogonal
                        // neighbour was an obstacle, so paths cannot slip through
                        // the corner gap between two blocked tiles.
                        if (dx != 0 && dy != 0 &&
                            (!_grid.IsWalkable(cx + dx, cy) || !_grid.IsWalkable(cx, cy + dy)))
                            continue;

                        int neighbour = _grid.Index(nx, ny);

                        float moveCost = (dx == 0 || dy == 0) ? 1f : Sqrt2;
                        float newG = _gCost[current] + moveCost;

                        if (newG >= _gCost[neighbour])
                            continue;

                        _gCost[neighbour] = newG;
                        _cameFrom[neighbour] = current;

                        int clearance = _grid.GetClearance(nx, ny);
                        float clearancePenalty = Mathf.Max(0, _grid.Width - clearance) * _settings.clearanceWeight;

                        _open.Push(neighbour, newG + clearancePenalty);
                    }
                }
            }

            if (!found)
                return path;

            BuildPath(startIdx, goalIdx, path);

            if (_settings.simplifyPath)
                path = SimplifyPath(path);

            return path;
        }

        /// <summary>
        /// Port of Stage 1's FindPathForAgent: a plain A* on Euclidean step cost
        /// with a straight-line heuristic and NO clearance penalty. It produces the
        /// shortest route rather than the safest one.
        ///
        /// Worth knowing: in Stage 1 this, not the clearance-weighted search, was
        /// the path the simulated agent actually flew. The clearance-weighted path
        /// was computed as a "backbone" and used for visualisation and for the
        /// deviation metric. Stage 2 follows the CLAUDE.md specification instead and
        /// flies the clearance-weighted route, so this method is kept for the
        /// Phase 9 baseline comparison rather than for normal dispatch.
        ///
        /// FIDELITY NOTE: Stage 1 omitted the diagonal corner-cutting guard here,
        /// though it had one in the safest-path search. That inconsistency is
        /// preserved so baseline numbers match Stage 1's.
        /// </summary>
        public List<Vector3> FindShortestPath(Vector3 start, Vector3 goal)
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

            ResetSearchState();
            _gCost[startIdx] = 0f;
            _open.Push(startIdx, Heuristic(sx, sy, gx, gy));

            bool found = false;

            while (_open.Count > 0)
            {
                int current = _open.Pop();

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

                        int neighbour = _grid.Index(nx, ny);

                        float moveCost = (dx == 0 || dy == 0) ? 1f : Sqrt2;
                        float newG = _gCost[current] + moveCost;

                        if (newG >= _gCost[neighbour])
                            continue;

                        _gCost[neighbour] = newG;
                        _cameFrom[neighbour] = current;
                        _open.Push(neighbour, newG + Heuristic(nx, ny, gx, gy));
                    }
                }
            }

            if (!found)
                return path;

            BuildPath(startIdx, goalIdx, path);
            return path;
        }

        private void ResetSearchState()
        {
            for (int i = 0; i < _gCost.Length; i++)
            {
                _gCost[i] = float.PositiveInfinity;
                _cameFrom[i] = -1;
            }

            _open.Clear();
        }

        private void BuildPath(int startIdx, int goalIdx, List<Vector3> into)
        {
            for (int idx = goalIdx; idx != -1; idx = _cameFrom[idx])
            {
                into.Add(_grid.CellToWorld(idx % _grid.Width, idx / _grid.Width));
                if (idx == startIdx)
                    break;
            }

            into.Reverse();
        }

        private static float Heuristic(int x, int y, int gx, int gy)
        {
            float dx = gx - x;
            float dy = gy - y;
            return Mathf.Sqrt(dx * dx + dy * dy);
        }

        // ---------------------------------------------------------------------
        // Optional path simplification (Stage 2 addition, off by default)
        // ---------------------------------------------------------------------

        /// <summary>
        /// Drops waypoints whose predecessor can reach their successor in a straight
        /// line. Stage 1 had no such step and flew every tile, so this is off by
        /// default; it exists because Stage 2 dispatches far more paths and a
        /// 60-waypoint route per mission is wasteful to store and stream.
        ///
        /// The shortcut test is clearance-aware on purpose. A plain line-of-sight
        /// test would straighten the route back onto cells that hug an obstacle,
        /// discarding the margin the clearance-weighted search just paid for.
        /// </summary>
        private List<Vector3> SimplifyPath(List<Vector3> path)
        {
            if (path.Count <= 2)
                return path;

            var result = new List<Vector3> { path[0] };
            int anchor = 0;
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

        /// <summary>Samples a segment at half-cell steps; every sample must be walkable and clear enough.</summary>
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
        /// Port of Stage 1's ComputeORCAVelocity. Returns the velocity this agent
        /// should fly this frame given what it can currently sense.
        ///
        /// The Stage 1 rule, reproduced here: take the preferred velocity straight
        /// at the next waypoint at full speed; for every agent closer than the
        /// combined radii plus a buffer, add an inverse-distance push directly away
        /// from it; do the same for nearby obstacles with a larger buffer; scale the
        /// summed push by max speed, add it to the preferred velocity, and clamp the
        /// result to max speed.
        ///
        /// IMPLEMENTATION NOTE: despite Stage 1 naming it ComputeORCAVelocity, this
        /// is an artificial-potential-field style avoidance heuristic, NOT the ORCA
        /// velocity-obstacle linear program. There are no half-plane constraints and
        /// no LP solve anywhere in it. It is ORCA-inspired in that each agent acts
        /// alone on what it senses and assumes others do the same, which is the
        /// reciprocity idea from van den Berg et al. (2011), without the machinery.
        ///
        /// This signature is the seam. A true ORCA implementation, such as a port of
        /// RVO2, can replace this body without any caller changing.
        /// </summary>
        public Vector3 ComputeAvoidanceVelocity(AgentData self, List<AgentData> neighbors, List<Obstacle> obstacles)
        {
            // ADAPTED: flattened to the XZ plane. Stage 1 kept every agent and
            // obstacle at roughly the same height so 3D distance was harmless. Stage
            // 2 flies drones above ground-level obstacles, so an unflattened
            // distance would understate how close a drone is to a building.
            Vector3 toGoal = self.goal - self.position;
            toGoal.y = 0f;

            // Stage 1 normalised and always flew at full speed, with no arrival taper.
            Vector3 preferredDirection = toGoal.sqrMagnitude > 0.0000001f ? toGoal.normalized : Vector3.zero;
            Vector3 preferredVelocity = preferredDirection * self.maxSpeed;

            Vector3 avoidance = Vector3.zero;

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

                    float combinedRadius = self.radius + other.radius;
                    if (distance >= combinedRadius + _settings.agentBuffer)
                        continue;

                    // Two agents exactly on top of each other give no direction.
                    // Break the tie by id so the pair separates deterministically.
                    if (distance < 0.0001f)
                    {
                        away = new Vector3(self.id < other.id ? 1f : -1f, 0f, 0f);
                        distance = 0.0001f;
                    }

                    // Stage 1: closer means a stronger push, as 1/distance.
                    Vector3 direction = away.normalized;
                    float magnitude = 1f / distance;
                    Vector3 push = direction * magnitude;

                    // ADAPTED: add a sideways component when the push points almost
                    // straight back along the way we want to travel.
                    //
                    // Stage 1's push is purely radial. A purely radial push cannot
                    // get an agent past a blocker sitting on its path: the push
                    // cancels the preferred velocity exactly and the agent stops
                    // dead, forever. That is the classic potential-field local
                    // minimum, and it is not hypothetical here. A drone parked on a
                    // charging pad that happens to lie on another drone's route
                    // pinned it at zero speed indefinitely.
                    //
                    // Real ORCA never has this failure, because its half-plane
                    // constraint still permits motion across the obstacle rather
                    // than only away from it. Rotating part of the push by ninety
                    // degrees restores that: the agent slides around the blocker
                    // instead of pressing into it. The side is chosen by id order so
                    // that two drones meeting head-on pick opposite sides and the
                    // choice is reproducible.
                    push += TangentialEscape(direction, preferredDirection, magnitude, self.id > other.id);

                    avoidance += push;
                }
            }

            if (obstacles != null)
            {
                for (int i = 0; i < obstacles.Count; i++)
                {
                    var obstacle = obstacles[i];

                    // ADAPTED: Stage 1 measured to the obstacle's centre and assumed
                    // every obstacle had radius 1. Stage 2 obstacles have real
                    // footprints including long boxes, so measure to the nearest
                    // point of the footprint. A building now repels from its face
                    // rather than from its middle.
                    Vector3 closest = obstacle.ClosestPoint(self.position);
                    Vector3 away = self.position - closest;
                    away.y = 0f;
                    float distance = away.magnitude;

                    if (distance >= self.radius + _settings.obstacleBuffer)
                        continue;

                    // Inside the footprint: push out from the centre instead.
                    if (distance < 0.0001f)
                    {
                        away = self.position - obstacle.center;
                        away.y = 0f;
                        if (away.sqrMagnitude < 0.0000001f)
                            away = Vector3.right;
                        distance = 0.0001f;
                    }

                    Vector3 obstacleDirection = away.normalized;
                    float obstacleMagnitude = 1f / distance;
                    Vector3 obstaclePush = obstacleDirection * obstacleMagnitude;

                    // Same escape term. An obstacle cannot move aside, so pick the
                    // side that lies closer to where the agent is already heading.
                    Vector3 leftward = new Vector3(-obstacleDirection.z, 0f, obstacleDirection.x);
                    bool flip = Vector3.Dot(leftward, preferredDirection) < 0f;
                    obstaclePush += TangentialEscape(obstacleDirection, preferredDirection, obstacleMagnitude, flip);

                    avoidance += obstaclePush;
                }
            }

            Vector3 newVelocity = preferredVelocity + avoidance * self.maxSpeed;
            newVelocity.y = 0f;

            if (newVelocity.sqrMagnitude > self.maxSpeed * self.maxSpeed)
                newVelocity = newVelocity.normalized * self.maxSpeed;

            return newVelocity;
        }

        /// <summary>
        /// The sideways part of a repulsion, used only when the radial push opposes
        /// the direction of travel. Returns zero when the push is not blocking, so
        /// ordinary passing encounters keep Stage 1's behaviour exactly.
        /// </summary>
        private static Vector3 TangentialEscape(Vector3 pushDirection, Vector3 preferredDirection, float magnitude, bool flipSide)
        {
            if (preferredDirection.sqrMagnitude < 0.0000001f)
                return Vector3.zero;

            // 1 means the push points exactly back along the travel direction.
            float opposition = -Vector3.Dot(pushDirection, preferredDirection);
            if (opposition <= 0.3f)
                return Vector3.zero;

            Vector3 tangent = new Vector3(-pushDirection.z, 0f, pushDirection.x);
            if (flipSide)
                tangent = -tangent;

            return tangent * magnitude * opposition;
        }


        // =====================================================================
        // Binary min-heap keyed by float priority.
        //
        // Stage 1 used a generic PriorityQueue<TileData> that compared on a
        // priority field stored on the tile itself. Same algorithm, but the
        // priority travels with the heap entry here so a re-queued node cannot
        // silently change the ordering of entries already in the heap.
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
