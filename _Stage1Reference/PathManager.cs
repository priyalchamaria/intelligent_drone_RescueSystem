// PathManager.cs
// Integrated Voronoi (safest path) + ORCA local avoidance for UAV demo
// Extended Voronoi visualization and full console logging

using UnityEngine;
using UnityEngine.UI;
using System.Collections;
using System.Collections.Generic;
using System.Diagnostics;
using System;
using System.IO;
using Debug = UnityEngine.Debug;

#region PriorityQueue (min-heap)
public class PriorityQueue<T> where T : IComparable<T>
{
    private List<T> data;
    public int Count => data.Count;

    public PriorityQueue() { data = new List<T>(); }

    public void Enqueue(T item)
    {
        data.Add(item);
        int ci = data.Count - 1;
        while (ci > 0)
        {
            int pi = (ci - 1) / 2;
            if (data[ci].CompareTo(data[pi]) >= 0) break;
            T tmp = data[ci]; data[ci] = data[pi]; data[pi] = tmp;
            ci = pi;
        }
    }

    public T Dequeue()
    {
        int li = data.Count - 1;
        T front = data[0];
        data[0] = data[li];
        data.RemoveAt(li);
        li--;
        int pi = 0;
        while (true)
        {
            int ci = pi * 2 + 1;
            if (ci > li) break;
            int rc = ci + 1;
            if (rc <= li && data[rc].CompareTo(data[ci]) < 0) ci = rc;
            if (data[pi].CompareTo(data[ci]) <= 0) break;
            T tmp = data[pi]; data[pi] = data[ci]; data[ci] = tmp;
            pi = ci;
        }
        return front;
    }
}
#endregion

#region TileData & AgentData
[System.Serializable]
public class TileData : IComparable<TileData>
{
    public GameObject tileObject;
    public int x, z;
    public float gCost;
    public float priority;
    public bool isStart, isEnd, isObstacle;
    public TileData parent;

    public void SetColor(Color color)
    {
        if (tileObject == null) return;
        var rend = tileObject.GetComponent<Renderer>();
        if (rend != null) rend.material.color = color;
    }

    public int CompareTo(TileData other)
    {
        return priority.CompareTo(other.priority);
    }
}

public class AgentData
{
    public Vector3 position;
    public Vector3 velocity;
    public Vector3 preferredVelocity;
    public float radius = 0.5f;
    public TileData currentTile;
    public List<Vector3> pathHistory = new List<Vector3>();
    public Vector3 lastVelocity;
    public float sumSquaredAccel = 0f;
    public int smoothSamples = 0;
    public Vector3 lastPosition;
    public float stuckTimer = 0f;
    public float stuckCheckInterval = 0.5f;
    public List<Vector3> waypoints;
    public int waypointIndex = 0;
    public bool finished = false;
}
#endregion

public class PathManager : MonoBehaviour
{
    public GameObject startPoint;
    public GameObject endPoint;
    public GameObject tilePrefab;
    public GameObject startPoint2; // <-- ADD THIS LINE
    public GameObject endPoint2;
    public Text legendText;

    public int gridWidth = 50;
    public int gridHeight = 50;
    public int obstacleCount = 300;
    public float clearanceWeight = 5.0f;

    public float maxSpeed = 3f;
    public float dt = 0.07f;
    public float waypointTolerance = 0.35f;

    private TileData[,] grid;
    private int[,] clearanceMap;
    private List<TileData> voronoiPath;
    private List<Vector3> waypoints;
    private List<AgentData> agents;
private List<GameObject> agentObjects; 
    // Add this with your other private variables
private GameObject[] dynamicObstacles;

    private float backbonePathCost = 0f;
    private float backbonePathLength = 0f;
    private double voronoiBuildAndPlanMs = 0.0;
    private Stopwatch missionStopwatch;
    private int collisionFlag = 0;
    private float minClearanceAlongFlight = float.MaxValue;
    private float sumClearanceAlongFlight = 0f;
    private int clearanceSamples = 0;
    private float sumDeviation = 0f;
    private int deviationSamples = 0;

    private const float tileY = 0f;
    private static readonly float Sqrt2 = 1.41421356f;
    private string csvPath;

    void Start()
    {
        csvPath = Path.Combine(Application.persistentDataPath, $"voronoi_orca_results_{DateTime.Now:yyyyMMdd_HHmmss}.csv");
        StartCoroutine(MainRoutine());
    }

    IEnumerator MainRoutine()
    {
        // In MainRoutine()



        Debug.Log("Voronoi planning started...");
        yield return StartCoroutine(GenerateGrid());
        dynamicObstacles = GameObject.FindGameObjectsWithTag("DynamicObstacle"); // <-- ADD THIS LINE
        clearanceMap = ComputeClearanceMap();

        var swVor = Stopwatch.StartNew();
        voronoiPath = FindPathSafest_GetPath();
        swVor.Stop();
        voronoiBuildAndPlanMs = swVor.Elapsed.TotalMilliseconds;

        if (voronoiPath == null || voronoiPath.Count == 0)
        {
            Debug.LogError("Voronoi/Safest path not found. Aborting.");
            UpdateLegendText("No path");
            yield break;
        }

        backbonePathCost = ComputePathCost(voronoiPath);
        backbonePathLength = ComputeEuclideanLength(voronoiPath);

        waypoints = new List<Vector3>();
        foreach (var t in voronoiPath)
            waypoints.Add(t.tileObject.transform.position + Vector3.up * 0.5f);

        foreach (var t in voronoiPath)
            if (!t.isStart && !t.isEnd) t.SetColor(Color.cyan);

        Debug.Log($"Voronoi planning ended. Metrics: PathCost={backbonePathCost:F2}, PathLength={backbonePathLength:F2}, PlanTime={voronoiBuildAndPlanMs:F3} ms");

        // keep Voronoi visualization for a longer time
        yield return new WaitForSeconds(2f);

        // In your MainRoutine method...
// ... (after Voronoi planning and visualization) ...

// ---- REPLACE the old ORCA block with this ----
Debug.Log("Multi-agent simulation started...");
InitAgents(); // Make sure you are calling the plural InitAgents()
missionStopwatch = Stopwatch.StartNew();
yield return StartCoroutine(RunSimulation()); // <-- Call the new function here

       // CORRECTED LOGGING LINE:
Debug.Log($"Simulation ended. Metrics: CollisionFlag={collisionFlag}, AvgClearance={(clearanceSamples > 0 ? sumClearanceAlongFlight / clearanceSamples : 0f):F2}, MinClearance={minClearanceAlongFlight:F2}, AvgDeviation={(deviationSamples > 0 ? sumDeviation / deviationSamples : 0f):F2}, MeanSqAccel={(agents.Count > 0 && agents[0].smoothSamples > 0 ? agents[0].sumSquaredAccel / agents[0].smoothSamples : 0f):F4}, MissionTime={missionStopwatch.Elapsed.TotalMilliseconds:F0} ms");

        WriteSummaryCSV();
        Debug.Log($"Mission finished. CSV saved to: {csvPath}");
    }

    #region Grid & Voronoi
    IEnumerator GenerateGrid()
    {
        grid = new TileData[gridWidth, gridHeight];
        for (int x = 0; x < gridWidth; x++)
        {
            for (int z = 0; z < gridHeight; z++)
            {
                Vector3 pos = new Vector3(x, tileY, z);
                GameObject tileObj = Instantiate(tilePrefab, pos, Quaternion.identity);
                tileObj.name = $"Tile_{x}_{z}";
                tileObj.transform.parent = this.transform;
                var rend = tileObj.GetComponent<Renderer>();
                if (rend)
                {
                    rend.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
                    rend.receiveShadows = false;
                    rend.material.color = Color.white;
                }

                TileData tile = new TileData { tileObject = tileObj, x = x, z = z, isObstacle = false, isStart = false, isEnd = false };
                grid[x, z] = tile;
            }
            if (x % 10 == 0) yield return null;
        }

        System.Random rand = new System.Random(300);
        int placed = 0;
        while (placed < obstacleCount)
        {
            int ox = rand.Next(0, gridWidth);
            int oz = rand.Next(0, gridHeight);
            if (grid[ox, oz].isObstacle) continue;
            Vector3 sp = startPoint.transform.position;
            Vector3 ep = endPoint.transform.position;
            if ((ox == (int)sp.x && oz == (int)sp.z) || (ox == (int)ep.x && oz == (int)ep.z)) continue;

            grid[ox, oz].isObstacle = true;
            grid[ox, oz].SetColor(Color.black);
            grid[ox, oz].tileObject.transform.localScale = new Vector3(1f, 2f, 1f);
            Vector3 p = grid[ox, oz].tileObject.transform.position;
            p.y = 0.5f;
            grid[ox, oz].tileObject.transform.position = p;
            placed++;
            if (placed % 50 == 0) yield return null;
        }

        int sx = Mathf.Clamp(Mathf.RoundToInt(startPoint.transform.position.x), 0, gridWidth - 1);
        int sz = Mathf.Clamp(Mathf.RoundToInt(startPoint.transform.position.z), 0, gridHeight - 1);
        int gx = Mathf.Clamp(Mathf.RoundToInt(endPoint.transform.position.x), 0, gridWidth - 1);
        int gz = Mathf.Clamp(Mathf.RoundToInt(endPoint.transform.position.z), 0, gridHeight - 1);
        grid[sx, sz].isStart = true; grid[gx, gz].isEnd = true;
        grid[sx, sz].SetColor(Color.green);
        grid[gx, gz].SetColor(Color.red);
        yield return null;
    }

    int[,] ComputeClearanceMap()
    {
        int[,] clearance = new int[gridWidth, gridHeight];
        Queue<(int x, int z, int dist)> queue = new Queue<(int x, int z, int dist)>();
        for (int x = 0; x < gridWidth; x++)
            for (int z = 0; z < gridHeight; z++)
            {
                if (grid[x, z].isObstacle)
                {
                    clearance[x, z] = 0;
                    queue.Enqueue((x, z, 0));
                }
                else clearance[x, z] = int.MaxValue;
            }
        int[] dx = { 0, 0, 1, -1 }; int[] dz = { 1, -1, 0, 0 };
        while (queue.Count > 0)
        {
            var (x, z, dist) = queue.Dequeue();
            for (int i = 0; i < 4; i++)
            {
                int nx = x + dx[i]; int nz = z + dz[i];
                if (nx >= 0 && nx < gridWidth && nz >= 0 && nz < gridHeight)
                {
                    if (clearance[nx, nz] > dist + 1)
                    {
                        clearance[nx, nz] = dist + 1;
                        queue.Enqueue((nx, nz, dist + 1));
                    }
                }
            }
        }
        return clearance;
    }

    List<TileData> FindPathSafest_GetPath()
    {
        int sx = Mathf.Clamp(Mathf.RoundToInt(startPoint.transform.position.x), 0, gridWidth - 1);
        int sz = Mathf.Clamp(Mathf.RoundToInt(startPoint.transform.position.z), 0, gridHeight - 1);
        int ex = Mathf.Clamp(Mathf.RoundToInt(endPoint.transform.position.x), 0, gridWidth - 1);
        int ez = Mathf.Clamp(Mathf.RoundToInt(endPoint.transform.position.z), 0, gridHeight - 1);

        TileData start = grid[sx, sz]; TileData end = grid[ex, ez];
        if (clearanceMap == null) clearanceMap = ComputeClearanceMap();

        PriorityQueue<TileData> openSet = new PriorityQueue<TileData>();
        Dictionary<TileData, float> gCostMap = new Dictionary<TileData, float>();
        foreach (TileData t in grid) { t.parent = null; gCostMap[t] = float.MaxValue; }

        gCostMap[start] = 0f; start.priority = 0f; openSet.Enqueue(start);

        while (openSet.Count > 0)
        {
            TileData current = openSet.Dequeue();
            if (current == end)
            {
                List<TileData> path = new List<TileData>();
                TileData cur = end;
                while (cur != null) { path.Add(cur); cur = cur.parent; }
                path.Reverse(); return path;
            }

            if (!current.isStart && !current.isEnd) current.SetColor(Color.yellow);

            foreach (TileData neighbor in GetNeighbors(current))
            {
                if (neighbor.isObstacle) continue;
                if (Mathf.Abs(neighbor.x - current.x) == 1 && Mathf.Abs(neighbor.z - current.z) == 1)
                {
                    if (grid[current.x, neighbor.z].isObstacle || grid[neighbor.x, current.z].isObstacle) continue;
                }

                float moveCost = (current.x == neighbor.x || current.z == neighbor.z) ? 1f : Sqrt2;
                float newG = gCostMap[current] + moveCost;
                if (newG < gCostMap[neighbor])
                {
                    gCostMap[neighbor] = newG;
                    int c = clearanceMap[neighbor.x, neighbor.z];
                    float clearancePenalty = (Mathf.Max(0, (gridWidth - c)) * clearanceWeight);
                    neighbor.priority = newG + clearancePenalty;
                    neighbor.parent = current;
                    openSet.Enqueue(neighbor);
                }
            }
        }
        return null;
    }

    List<TileData> GetNeighbors(TileData t)
    {
        List<TileData> neighbors = new List<TileData>();
        for (int dx = -1; dx <= 1; dx++)
            for (int dz = -1; dz <= 1; dz++)
            {
                if (dx == 0 && dz == 0) continue;
                int nx = t.x + dx, nz = t.z + dz;
                if (nx >= 0 && nx < gridWidth && nz >= 0 && nz < gridHeight)
                    neighbors.Add(grid[nx, nz]);
            }
        return neighbors;
    }

    float ComputePathCost(List<TileData> path)
    {
        float cost = 0f;
        for (int i = 1; i < path.Count; i++)
            cost += Vector3.Distance(path[i - 1].tileObject.transform.position, path[i].tileObject.transform.position);
        return cost;
    }

    float ComputeEuclideanLength(List<TileData> path)
    {
        return ComputePathCost(path);
    }
    #endregion

    #region ORCA Agent
    // Add these THREE new methods to your PathManager script

void InitAgents()
{
    // Initialize lists
    agents = new List<AgentData>();
    agentObjects = new List<GameObject>();

    // --- Agent 1 Setup ---
    TileData startTile1 = grid[Mathf.RoundToInt(startPoint.transform.position.x), Mathf.RoundToInt(startPoint.transform.position.z)];
    TileData endTile1 = grid[Mathf.RoundToInt(endPoint.transform.position.x), Mathf.RoundToInt(endPoint.transform.position.z)];
    List<TileData> path1 = FindPathForAgent(startTile1, endTile1);
    
    // Create and add the first agent
    AddAgent(startPoint.transform.position, path1, Color.magenta);

    // --- Agent 2 Setup ---
    // Make sure you have created startPoint2 and endPoint2 in the Unity Editor
    // if (startPoint2 != null && endPoint2 != null)
    // {
    //     TileData startTile2 = grid[Mathf.RoundToInt(startPoint2.transform.position.x), Mathf.RoundToInt(startPoint2.transform.position.z)];
    //     TileData endTile2 = grid[Mathf.RoundToInt(endPoint2.transform.position.x), Mathf.RoundToInt(endPoint2.transform.position.z)];
    //     List<TileData> path2 = FindPathForAgent(startTile2, endTile2);

    //     // Create and add the second agent
    //     AddAgent(startPoint2.transform.position, path2, Color.yellow);
    // }
}

void AddAgent(Vector3 startPos, List<TileData> path, Color color)
{
    if (path == null || path.Count == 0)
    {
        Debug.LogError("Path not found for an agent. Agent not created.");
        return;
    }

    AgentData newAgent = new AgentData();
    newAgent.position = startPos + Vector3.up * 0.5f;
    newAgent.lastPosition = newAgent.position;
    
    newAgent.waypoints = new List<Vector3>();
    foreach (var tile in path)
    {
        newAgent.waypoints.Add(tile.tileObject.transform.position + Vector3.up * 0.5f);
    }

    agents.Add(newAgent);

    GameObject agentObj = GameObject.CreatePrimitive(PrimitiveType.Cube);
    agentObj.transform.localScale = Vector3.one * 0.5f;
    agentObj.GetComponent<Renderer>().material.color = color;
    agentObj.transform.position = newAgent.position;
    agentObjects.Add(agentObj);
}

List<TileData> FindPathForAgent(TileData start, TileData end)
{
    PriorityQueue<TileData> openSet = new PriorityQueue<TileData>();
    Dictionary<TileData, float> gCostMap = new Dictionary<TileData, float>();
    
    foreach (TileData t in grid) { t.parent = null; gCostMap[t] = float.MaxValue; }

    gCostMap[start] = 0f;
    start.priority = Vector3.Distance(start.tileObject.transform.position, end.tileObject.transform.position);
    openSet.Enqueue(start);

    while (openSet.Count > 0)
    {
        TileData current = openSet.Dequeue();
        if (current == end)
        {
            List<TileData> path = new List<TileData>();
            TileData cur = end;
            while (cur != null) { path.Add(cur); cur = cur.parent; }
            path.Reverse();
            return path;
        }

        foreach (TileData neighbor in GetNeighbors(current))
        {
            if (neighbor.isObstacle) continue;
            float moveCost = Vector3.Distance(current.tileObject.transform.position, neighbor.tileObject.transform.position);
            float newG = gCostMap[current] + moveCost;

            if (newG < gCostMap[neighbor])
            {
                gCostMap[neighbor] = newG;
                neighbor.priority = newG + Vector3.Distance(neighbor.tileObject.transform.position, end.tileObject.transform.position);
                neighbor.parent = current;
                openSet.Enqueue(neighbor);
            }
        }
    }
    return null;
}

IEnumerator RunSimulation()
{
    int agentsFinished = 0;
    while (agentsFinished < agents.Count)
    {
        // 1. COMPUTE NEW VELOCITIES for all agents based on ORCA
        for (int i = 0; i < agents.Count; i++)
        {
            if (agents[i].finished) continue;
            agents[i].velocity = ComputeORCAVelocity(agents[i], i);
        }

        // 2. UPDATE POSITIONS for all agents
        for (int i = 0; i < agents.Count; i++)
{
    AgentData agent = agents[i];
    if (agent.finished) continue;

    // Move agent based on ORCA velocity
    agent.position += agent.velocity * dt;
    agentObjects[i].transform.position = agent.position;

    // =================================================================
    // METRICS CALCULATION START (This must be INSIDE the loop)
    // =================================================================

    // 1. Calculate Velocity and Acceleration for Smoothness
    // We use agent.velocity, which was just calculated by ORCA
    if(dt > 0)
    {
        Vector3 acceleration = (agent.velocity - agent.lastVelocity) / dt;
        agent.sumSquaredAccel += acceleration.sqrMagnitude;
        agent.smoothSamples++;
    }
    agent.lastVelocity = agent.velocity;
    agent.lastPosition = agent.position;

    // 2. Calculate Clearance from static obstacles
    int agentX = Mathf.Clamp(Mathf.RoundToInt(agent.position.x), 0, gridWidth - 1);
    int agentZ = Mathf.Clamp(Mathf.RoundToInt(agent.position.z), 0, gridHeight - 1);
    if (clearanceMap != null)
    {
        float currentClearance = clearanceMap[agentX, agentZ];
        sumClearanceAlongFlight += currentClearance;
        minClearanceAlongFlight = Mathf.Min(minClearanceAlongFlight, currentClearance);
        clearanceSamples++;
    }

    // 3. Calculate Deviation from the global path backbone
    if(agent.waypointIndex < agent.waypoints.Count)
    {
        Vector3 targetWaypoint = agent.waypoints[agent.waypointIndex];
        // Note: This logic assumes you have a startPoint variable for agent 0.
        // It's a simplification for now but should work for your test case.
        Vector3 prevWaypoint = (agent.waypointIndex == 0) ? startPoint.transform.position + Vector3.up * 0.5f : agent.waypoints[agent.waypointIndex - 1];
        float currentDeviation = FindDistanceToLineSegment(agent.position, prevWaypoint, targetWaypoint);
        sumDeviation += currentDeviation;
        deviationSamples++;
    }
    // =================================================================
    // METRICS CALCULATION END
    // =================================================================
}

        // Check how many agents have finished their path
        agentsFinished = 0;
        foreach(var agent in agents)
        {
            if (agent.finished) agentsFinished++;
        }
        
        yield return new WaitForSeconds(dt);
    }
    
    missionStopwatch.Stop();
    Debug.Log("All agents reached their goals!");
}


Vector3 ComputeORCAVelocity(AgentData currentAgent, int agentIndex)
{
    // Check if agent has reached its goal
    if (currentAgent.waypointIndex >= currentAgent.waypoints.Count)
    {
        currentAgent.finished = true;
        return Vector3.zero;
    }

    // 1. Determine Preferred Velocity (points towards the next waypoint)
    Vector3 target = currentAgent.waypoints[currentAgent.waypointIndex];
    Vector3 preferredDir = (target - currentAgent.position).normalized;
    Vector3 preferredVelocity = preferredDir * maxSpeed;

    // Check if we are close enough to the waypoint to advance to the next one
    if (Vector3.Distance(currentAgent.position, target) < waypointTolerance)
    {
        currentAgent.waypointIndex++;
        if (currentAgent.waypointIndex >= currentAgent.waypoints.Count)
        {
            currentAgent.finished = true;
            return Vector3.zero;
        }
    }
    
    // --- Start of Avoidance Logic ---
    Vector3 avoidanceVector = Vector3.zero;

    // A) Check against other agents
    if (agents != null) // Make sure the agents list exists
    {
        for (int i = 0; i < agents.Count; i++)
        {
            if (i == agentIndex) continue; // Don't check against self
            AgentData other = agents[i];
            float dist = Vector3.Distance(currentAgent.position, other.position);
            float combinedRadius = currentAgent.radius + other.radius;
            if (dist < combinedRadius + 1.0f) // Using a buffer of 1.0f
            {
                Vector3 awayFromOther = (currentAgent.position - other.position).normalized;
                avoidanceVector += awayFromOther / dist; // The closer, the stronger the push
            }
        }
    }

    // B) Check against dynamic obstacles
    if (dynamicObstacles != null) // Make sure the obstacles list exists
    {
        foreach (GameObject obstacle in dynamicObstacles)
        {
            float dist = Vector3.Distance(currentAgent.position, obstacle.transform.position);
            float combinedRadius = currentAgent.radius + 1.0f; // Assuming obstacle radius of 1.0f
            if (dist < combinedRadius + 1.5f) // Using a larger buffer for safety
            {
                Vector3 awayFromObs = (currentAgent.position - obstacle.transform.position).normalized;
                avoidanceVector += awayFromObs / dist;
            }
        }
    }
    
    // --- End of Avoidance Logic ---
    
    // Combine preferred velocity with the avoidance vector
    Vector3 newVelocity = preferredVelocity + avoidanceVector * maxSpeed;

    // Clamp the final velocity to the max speed
    if (newVelocity.sqrMagnitude > maxSpeed * maxSpeed)
    {
        newVelocity = newVelocity.normalized * maxSpeed;
    }

    return newVelocity;
}


    // Add this new helper function anywhere inside the PathManager class
    private float FindDistanceToLineSegment(Vector3 point, Vector3 lineStart, Vector3 lineEnd)
    {
        // Ensure calculations are on the XZ plane
        point.y = 0;
        lineStart.y = 0;
        lineEnd.y = 0;

        Vector3 lineDir = lineEnd - lineStart;
        float lineLengthSqr = lineDir.sqrMagnitude;
        if (lineLengthSqr < 0.0001f)
            return Vector3.Distance(point, lineStart);

        float t = Mathf.Clamp01(Vector3.Dot(point - lineStart, lineDir) / lineLengthSqr);
        Vector3 closestPoint = lineStart + t * lineDir;
        return Vector3.Distance(point, closestPoint);
    }
    #endregion

    void UpdateLegendText(string msg)
    {
        if (legendText != null) legendText.text = msg;
    }

    void WriteSummaryCSV()
    {
        try
        {
            using (StreamWriter sw = new StreamWriter(csvPath))
            {
                sw.WriteLine("Algorithm,PathCost,PathLength,PlanTimeMs,CollisionFlag,AvgClearance,MinClearance,AvgDeviation,MeanSquaredAccel,MissionTimeMs");
                // CORRECTED CSV WRITING LINE:
sw.WriteLine($"Voronoi+ORCA,{backbonePathCost:F2},{backbonePathLength:F2},{voronoiBuildAndPlanMs:F3},{collisionFlag},{(clearanceSamples > 0 ? sumClearanceAlongFlight / clearanceSamples : 0f):F2},{minClearanceAlongFlight:F2},{(deviationSamples > 0 ? sumDeviation / deviationSamples : 0f):F2},{(agents.Count > 0 && agents[0].smoothSamples > 0 ? agents[0].sumSquaredAccel / agents[0].smoothSamples : 0f):F4},{missionStopwatch.Elapsed.TotalMilliseconds:F0}");
            }
        }
        catch (Exception ex)
        {
            Debug.LogError("Error writing CSV: " + ex.Message);
        }
    }
}
