using System.Collections.Generic;
using UnityEngine;
using DroneRescue.Navigation;

namespace DroneRescue.Environment
{
    /// <summary>One authored obstacle in a scenario. Boxes are rubble, circles are fire zones.</summary>
    [System.Serializable]
    public class ObstacleDefinition
    {
        public string label = "Rubble";
        public ObstacleShape shape = ObstacleShape.Box;
        public Vector3 center;

        [Tooltip("Box footprint half-extents. Only x and z are used.")]
        public Vector3 halfExtents = new Vector3(6f, 4f, 6f);

        [Tooltip("Circle footprint radius, used when shape is Circle.")]
        public float radius = 6f;

        [Tooltip("Height of the placeholder box drawn in the scene.")]
        public float visualHeight = 8f;

        public bool isDynamic;

        public Obstacle ToObstacle()
        {
            return shape == ObstacleShape.Box
                ? new Obstacle(center, halfExtents, isDynamic)
                : new Obstacle(center, radius, isDynamic);
        }
    }

    /// <summary>One authored drone in a scenario.</summary>
    [System.Serializable]
    public class DroneDefinition
    {
        public string id = "D-01";
        public Vector3 spawnPosition;
        [Range(0f, 100f)] public float startBatteryPercent = 100f;
    }

    /// <summary>
    /// One authored patient in a scenario. The priority written here is used
    /// directly in Manual triage mode and ignored in WeightedRandom mode.
    /// </summary>
    [System.Serializable]
    public class PatientDefinition
    {
        public string id = "P-01";
        public Vector3 location;
        public PatientPriority priority = PatientPriority.Serious;
    }

    /// <summary>
    /// Everything that defines one runnable disaster scenario. Held as an asset so
    /// several scenarios can exist side by side, which is what Phase 9's baseline
    /// comparison runs will need.
    /// </summary>
    [CreateAssetMenu(fileName = "ScenarioConfig", menuName = "Drone Rescue/Scenario Config")]
    public class ScenarioConfig : ScriptableObject
    {
        [Header("World")]
        [Tooltip("World position of grid cell (0,0).")]
        public Vector3 gridOrigin = Vector3.zero;

        [Min(0.1f)] public float cellSize = 1f;
        [Min(2)] public int gridWidth = 100;
        [Min(2)] public int gridHeight = 100;

        [Tooltip("Altitude the drones fly at. Paths are computed on the ground plane.")]
        public float droneAltitude = 2f;

        [Header("Fixed sites")]
        public List<Vector3> hospitals = new List<Vector3>();
        public List<Vector3> chargingStations = new List<Vector3>();

        [Header("Environment")]
        public List<ObstacleDefinition> obstacles = new List<ObstacleDefinition>();

        [Header("Fleet")]
        public List<DroneDefinition> drones = new List<DroneDefinition>();

        [Tooltip("MAX_RANGE from the Part 1 hard filter: the distance a drone can fly on a full charge.")]
        [Min(1f)] public float maxRange = 250f;

        [Header("Casualties")]
        public List<PatientDefinition> patients = new List<PatientDefinition>();

        [Header("Triage")]
        [Tooltip("Chooses which of the two Part 1 priority-assignment modes is active.")]
        public TriageAssigner triage = new TriageAssigner();

        [Header("Navigation tuning")]
        public NavigationSettings navigation = new NavigationSettings();

        [Header("Debug")]
        [Tooltip("Instantiate one GameObject per grid tile to visualise clearance. Off by default: it does not scale.")]
        public bool visualiseGridTiles = false;

        /// <summary>Nearest hospital to a world position. Used by the Part 1 total-distance calculation.</summary>
        public Vector3 NearestHospital(Vector3 from)
        {
            if (hospitals == null || hospitals.Count == 0)
                return from;

            Vector3 best = hospitals[0];
            float bestSqr = (best - from).sqrMagnitude;

            for (int i = 1; i < hospitals.Count; i++)
            {
                float sqr = (hospitals[i] - from).sqrMagnitude;
                if (sqr < bestSqr)
                {
                    bestSqr = sqr;
                    best = hospitals[i];
                }
            }

            return best;
        }

        public List<Obstacle> BuildObstacles()
        {
            var result = new List<Obstacle>();
            if (obstacles == null)
                return result;

            for (int i = 0; i < obstacles.Count; i++)
                result.Add(obstacles[i].ToObstacle());

            return result;
        }
    }
}
