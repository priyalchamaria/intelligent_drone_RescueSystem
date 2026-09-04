using System.Collections.Generic;
using UnityEngine;

namespace DroneRescue.Navigation
{
    /// <summary>
    /// Owns the occupancy grid and the clearance (brushfire distance-transform) map
    /// for the disaster area, as PURE DATA. There are no per-tile GameObjects here
    /// and no rendering of any kind — visualization is a separate optional concern.
    ///
    /// Lifecycle: built once at scene load. The obstacle list mutates and the
    /// clearance map is recomputed ONLY when the environment changes (a FireSpread
    /// event), never per dispatch.
    /// </summary>
    public class DisasterGrid
    {
        /// <summary>World position of the centre of cell (0, 0).</summary>
        public Vector3 Origin { get; private set; }

        /// <summary>Edge length of one square cell, in world units.</summary>
        public float CellSize { get; private set; }

        /// <summary>Grid extent in cells along world X.</summary>
        public int Width { get; private set; }

        /// <summary>Grid extent in cells along world Z.</summary>
        public int Height { get; private set; }

        /// <summary>Y plane the grid lives on. Paths are returned at this height.</summary>
        public float GroundY { get; private set; }

        private bool[] _blocked;

        /// <summary>
        /// Distance (in cells) from each cell to the nearest blocked cell.
        /// Blocked cells are 0. Produced by ComputeClearanceMap.
        /// </summary>
        private int[] _clearance;

        private readonly List<Obstacle> _obstacles = new List<Obstacle>();

        /// <summary>Live obstacle list. Mutate via SetObstacles / AddObstacle only.</summary>
        public IReadOnlyList<Obstacle> Obstacles => _obstacles;

        /// <summary>Incremented every time the clearance map is rebuilt, so callers can invalidate cached paths.</summary>
        public int Version { get; private set; }

        /// <summary>
        /// Extra standoff, in world units, baked into the grid around DYNAMIC
        /// hazards only. Set from NavigationSettings before the obstacles are
        /// loaded; changing it afterwards needs a Rebake.
        ///
        /// WHY ONLY THE DYNAMIC ONES. Rubble is inert. A route that skims a
        /// collapsed building is a route that skims a collapsed building, and the
        /// clearance-weighted search already prefers not to. A fire front is not
        /// inert: it is the one obstacle in this simulation that grows, and the
        /// grid it was baked into is out of date the moment it does. Treating both
        /// identically, which is what this class used to do, meant a route could be
        /// planned flush against the edge of an active fire and still be counted
        /// safe. The margin buys back the time the fire takes to reach the path.
        /// </summary>
        public float DynamicHazardMargin { get; set; }

        public DisasterGrid(Vector3 origin, float cellSize, int width, int height, float groundY = 0f)
        {
            Origin = origin;
            CellSize = Mathf.Max(0.01f, cellSize);
            Width = Mathf.Max(1, width);
            Height = Mathf.Max(1, height);
            GroundY = groundY;

            _blocked = new bool[Width * Height];
            _clearance = new int[Width * Height];
            ComputeClearanceMap();
        }

        // ---------------------------------------------------------------------
        // Coordinate conversion
        // ---------------------------------------------------------------------

        public int Index(int x, int y) => y * Width + x;

        public bool InBounds(int x, int y) => x >= 0 && x < Width && y >= 0 && y < Height;

        public void WorldToCell(Vector3 world, out int x, out int y)
        {
            x = Mathf.RoundToInt((world.x - Origin.x) / CellSize);
            y = Mathf.RoundToInt((world.z - Origin.z) / CellSize);
        }

        public Vector3 CellToWorld(int x, int y)
        {
            return new Vector3(Origin.x + x * CellSize, GroundY, Origin.z + y * CellSize);
        }

        /// <summary>True when the cell is inside the grid and not occupied by an obstacle.</summary>
        public bool IsWalkable(int x, int y) => InBounds(x, y) && !_blocked[Index(x, y)];

        public bool IsWalkable(Vector3 world)
        {
            WorldToCell(world, out var x, out var y);
            return IsWalkable(x, y);
        }

        /// <summary>Distance in cells from this cell to the nearest obstacle. 0 on an obstacle.</summary>
        public int GetClearance(int x, int y) => InBounds(x, y) ? _clearance[Index(x, y)] : 0;

        // ---------------------------------------------------------------------
        // Obstacle management
        // ---------------------------------------------------------------------

        /// <summary>Replaces the whole obstacle set and rebuilds the clearance map.</summary>
        public void SetObstacles(IEnumerable<Obstacle> obstacles)
        {
            _obstacles.Clear();
            if (obstacles != null)
                _obstacles.AddRange(obstacles);
            Rebake();
        }

        /// <summary>
        /// Adds one obstacle and rebuilds. This is the FireSpread path: a growing
        /// fire front is pushed in here, which is the only thing that invalidates
        /// the clearance map mid-mission.
        /// </summary>
        public void AddObstacle(Obstacle obstacle)
        {
            _obstacles.Add(obstacle);
            Rebake();
        }

        /// <summary>Rasterises every obstacle into the occupancy grid, then recomputes clearance.</summary>
        public void Rebake()
        {
            System.Array.Clear(_blocked, 0, _blocked.Length);

            for (int i = 0; i < _obstacles.Count; i++)
                Rasterise(_obstacles[i]);

            ComputeClearanceMap();
            Version++;
        }

        /// <summary>
        /// Marks every cell whose centre falls inside the obstacle footprint as
        /// blocked. Works for both circular fire zones and boxed rubble, so the
        /// blocked area matches what is actually drawn in the scene.
        ///
        /// A dynamic hazard is grown by DynamicHazardMargin first, so the standoff
        /// costs nothing extra downstream: the brushfire seeds from the widened
        /// footprint, every free cell's clearance already accounts for it, and the
        /// path search keeps its distance without needing a rule about fire.
        /// </summary>
        private void Rasterise(Obstacle obstacle)
        {
            float margin = obstacle.isDynamic ? Mathf.Max(0f, DynamicHazardMargin) : 0f;

            WorldToCell(obstacle.center, out var cx, out var cy);
            int r = Mathf.CeilToInt((obstacle.BoundingRadius + margin) / CellSize) + 1;

            for (int y = cy - r; y <= cy + r; y++)
            {
                for (int x = cx - r; x <= cx + r; x++)
                {
                    if (!InBounds(x, y))
                        continue;

                    if (obstacle.Contains(CellToWorld(x, y), margin))
                        _blocked[Index(x, y)] = true;
                }
            }
        }

        // ---------------------------------------------------------------------
        // Clearance map (brushfire distance transform)
        // ---------------------------------------------------------------------

        /// <summary>
        /// Multi-source BFS ("brushfire") seeded from every blocked cell, ported
        /// directly from Stage 1's ComputeClearanceMap. Each free cell ends up
        /// holding its distance in cells to the nearest obstacle, which is what lets
        /// the path search prefer wide corridors.
        ///
        /// Two Stage 1 behaviours matter and are preserved:
        ///  - Only obstacles seed the flood. The map border is NOT treated as an
        ///    obstacle, so routes are free to run along the edge of the world.
        ///  - A cell the flood never reaches, which happens when the map has no
        ///    obstacles at all, keeps int.MaxValue. The clearance penalty formula
        ///    handles that: gridWidth minus a huge number goes negative and is
        ///    clamped to zero, meaning no penalty.
        /// </summary>
        public void ComputeClearanceMap()
        {
            const int Unvisited = int.MaxValue;
            for (int i = 0; i < _clearance.Length; i++)
                _clearance[i] = Unvisited;

            var queue = new Queue<int>();

            for (int i = 0; i < _blocked.Length; i++)
            {
                if (_blocked[i])
                {
                    _clearance[i] = 0;
                    queue.Enqueue(i);
                }
            }

            while (queue.Count > 0)
            {
                int idx = queue.Dequeue();
                int cx = idx % Width;
                int cy = idx / Width;
                int next = _clearance[idx] + 1;

                // 4-connected expansion, as in Stage 1.
                TryVisit(cx + 1, cy, next, queue);
                TryVisit(cx - 1, cy, next, queue);
                TryVisit(cx, cy + 1, next, queue);
                TryVisit(cx, cy - 1, next, queue);
            }
        }

        private void TryVisit(int x, int y, int distance, Queue<int> queue)
        {
            if (!InBounds(x, y))
                return;

            int idx = Index(x, y);
            if (_clearance[idx] <= distance)
                return;

            _clearance[idx] = distance;
            queue.Enqueue(idx);
        }

        // ---------------------------------------------------------------------
        // Helpers
        // ---------------------------------------------------------------------

        /// <summary>
        /// Nearest walkable cell to a world position, searched outward in rings.
        /// Used when a patient or hospital happens to sit inside an obstacle
        /// footprint (common once fire spreads over a marker).
        /// </summary>
        public bool TryFindNearestWalkable(Vector3 world, int maxRingSearch, out int outX, out int outY)
        {
            WorldToCell(world, out var sx, out var sy);
            if (IsWalkable(sx, sy))
            {
                outX = sx;
                outY = sy;
                return true;
            }

            for (int r = 1; r <= maxRingSearch; r++)
            {
                for (int dy = -r; dy <= r; dy++)
                {
                    for (int dx = -r; dx <= r; dx++)
                    {
                        // Only walk the ring perimeter.
                        if (Mathf.Abs(dx) != r && Mathf.Abs(dy) != r)
                            continue;

                        int x = sx + dx;
                        int y = sy + dy;
                        if (IsWalkable(x, y))
                        {
                            outX = x;
                            outY = y;
                            return true;
                        }
                    }
                }
            }

            outX = sx;
            outY = sy;
            return false;
        }
    }
}
