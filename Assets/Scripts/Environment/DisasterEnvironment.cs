using System.Collections.Generic;
using UnityEngine;
using DroneRescue.Fleet;
using DroneRescue.Navigation;
using DroneRescue.Planning;

namespace DroneRescue.Environment
{
    /// <summary>
    /// The shared state of the simulation, and the single hub of the hub-and-spoke
    /// model. Drones publish their records here; the Mission Planner (Phase 4) will
    /// read this and nothing else. Drones never read each other through this class.
    ///
    /// It also owns the navigation layer, building the DisasterGrid once at scene
    /// load and handing out one NavigationEngine for everyone to share.
    /// </summary>
    [DisallowMultipleComponent]
    public class DisasterEnvironment : MonoBehaviour
    {
        public const string GeneratedRootName = "Generated";

        [SerializeField] private ScenarioConfig config;

        [Header("Materials used by the scene builder")]
        [SerializeField] private Material groundMaterial;
        [SerializeField] private Material rubbleMaterial;
        [SerializeField] private Material fireMaterial;
        [SerializeField] private Material hospitalMaterial;
        [SerializeField] private Material chargingMaterial;
        [SerializeField] private Material droneMaterial;
        [SerializeField] private Material patientCriticalMaterial;
        [SerializeField] private Material patientSeriousMaterial;
        [SerializeField] private Material patientStableMaterial;

        // ---------------------------------------------------------------------
        // Shared state
        // ---------------------------------------------------------------------

        /// <summary>
        /// The live DroneList from Part 1. The Mission Planner reads drone
        /// location, battery and status from here.
        /// </summary>
        public List<Drone> DroneList { get; } = new List<Drone>();

        /// <summary>Every patient currently known to the system, in detection order.</summary>
        public List<Patient> Patients { get; } = new List<Patient>();

        /// <summary>
        /// The dispatch board: every mission the planner has ordered, live.
        ///
        /// This is the downlink half of the hub-and-spoke model, and it sits in
        /// shared state for the same reason DroneList does. The planner writes
        /// orders here and the drone side reads them; neither one calls the other.
        /// </summary>
        public List<MissionAssignment> Assignments { get; } = new List<MissionAssignment>();

        public DisasterGrid Grid { get; private set; }
        public NavigationEngine Navigation { get; private set; }
        public ScenarioConfig Config => config;

        private readonly List<DroneAgent> _droneAgents = new List<DroneAgent>();
        private readonly List<PatientMarker> _patientMarkers = new List<PatientMarker>();
        private int _patientCounter;

        public IReadOnlyList<DroneAgent> DroneAgents => _droneAgents;
        public IReadOnlyList<PatientMarker> PatientMarkers => _patientMarkers;

        private void Awake()
        {
            if (config == null)
            {
                Debug.LogError("[DisasterEnvironment] No ScenarioConfig assigned. Nothing will be built.");
                return;
            }

            BuildNavigation();
        }

        /// <summary>
        /// Fleet registration happens in Start, not Awake. Every drone's Awake is
        /// guaranteed to have run by then, so the drone list cannot come up short
        /// because of Unity's undefined Awake ordering.
        /// </summary>
        private void Start()
        {
            if (config == null)
                return;

            RegisterSceneAgents();
        }

        /// <summary>
        /// Builds the grid and clearance map once, then wraps it in a navigation
        /// engine. The clearance map is not touched again until a FireSpread event
        /// mutates the obstacle set.
        /// </summary>
        public void BuildNavigation()
        {
            Grid = new DisasterGrid(config.gridOrigin, config.cellSize, config.gridWidth, config.gridHeight, 0f);
            Grid.SetObstacles(config.BuildObstacles());
            Navigation = new NavigationEngine(Grid, config.navigation);
        }

        /// <summary>
        /// Collects the drone and patient objects that the scene builder placed, and
        /// puts their records into shared state. Triage runs here, once per patient.
        /// </summary>
        public void RegisterSceneAgents()
        {
            DroneList.Clear();
            Patients.Clear();
            Assignments.Clear();
            _droneAgents.Clear();
            _patientMarkers.Clear();
            _patientCounter = 0;

            config.triage.Reset();

            foreach (var agent in GetComponentsInChildren<DroneAgent>(true))
            {
                agent.SetAgentId(_droneAgents.Count);
                _droneAgents.Add(agent);
                DroneList.Add(agent.EnsureData());
            }

            foreach (var marker in GetComponentsInChildren<PatientMarker>(true))
            {
                _patientMarkers.Add(marker);
                var patient = DetectPatient(marker.PatientId, marker.transform.position, marker.AuthoredPriority);
                marker.Bind(patient);
            }

            Debug.Log("[DisasterEnvironment] Registered " + DroneList.Count + " drones and "
                      + Patients.Count + " patients. Triage mode: " + config.triage.mode + ".");
        }

        // ---------------------------------------------------------------------
        // Patient detection
        // ---------------------------------------------------------------------

        /// <summary>
        /// The single entry point for a patient entering the simulation.
        ///
        /// SIMULATED DETECTION: this constructs a Patient and drops it into shared
        /// state directly. There is no network, no call, no message. In a real
        /// deployment this call site is where a drone's thermal or camera payload
        /// would have autonomously detected a casualty, per the base paper.
        ///
        /// Phase 6's NewEmergency event calls this same method rather than having
        /// its own handling.
        /// </summary>
        public Patient DetectPatient(string id, Vector3 location, PatientPriority authoredPriority)
        {
            if (string.IsNullOrEmpty(id))
            {
                _patientCounter++;
                id = "P-" + _patientCounter.ToString("00");
            }

            // Triage decides the queue priority. This is separate from any
            // RiskFactor(location) used later in drone scoring.
            var priority = config.triage.Assign(authoredPriority);

            var patient = new Patient(id, location, priority, Time.time);
            Patients.Add(patient);
            return patient;
        }

        // ---------------------------------------------------------------------
        // Convenience reads for the Mission Planner
        // ---------------------------------------------------------------------

        public Vector3 NearestHospital(Vector3 from) => config.NearestHospital(from);

        /// <summary>
        /// A distinct landing point on a ring around a site, one per slot index.
        ///
        /// Without this every delivery targets the identical hospital coordinate, and
        /// the first drone to land sits on the goal repelling everyone behind it. The
        /// queue then hovers just outside reach and never completes. Spreading
        /// arrivals onto a ring removes the contention at source.
        /// </summary>
        public Vector3 LandingSlot(Vector3 site, int slotIndex, int slotCount)
        {
            if (slotCount <= 1 || config.landingRingRadius <= 0f)
                return site;

            float angle = (slotIndex % slotCount) * Mathf.PI * 2f / slotCount;
            return site + new Vector3(Mathf.Cos(angle), 0f, Mathf.Sin(angle)) * config.landingRingRadius;
        }

        public Vector3 NearestChargingStation(Vector3 from)
        {
            if (config.chargingStations == null || config.chargingStations.Count == 0)
                return from;

            Vector3 best = config.chargingStations[0];
            float bestSqr = (best - from).sqrMagnitude;

            for (int i = 1; i < config.chargingStations.Count; i++)
            {
                float sqr = (config.chargingStations[i] - from).sqrMagnitude;
                if (sqr < bestSqr)
                {
                    bestSqr = sqr;
                    best = config.chargingStations[i];
                }
            }

            return best;
        }

        /// <summary>Snapshots of every drone, for local avoidance. Phase 3 uses this.</summary>
        public List<AgentData> SnapshotAgents()
        {
            var snapshots = new List<AgentData>(_droneAgents.Count);
            for (int i = 0; i < _droneAgents.Count; i++)
                snapshots.Add(_droneAgents[i].ToAgentData(i));
            return snapshots;
        }

#if UNITY_EDITOR
        // ---------------------------------------------------------------------
        // Editor-time scene builder
        // ---------------------------------------------------------------------

        /// <summary>
        /// Assigns the scenario and the builder's materials from editor tooling.
        /// Exists so a setup script can wire this component up directly instead of
        /// going through SerializedObject.
        /// </summary>
        public void EditorAssign(ScenarioConfig scenario, Material ground, Material rubble, Material fire,
            Material hospital, Material charging, Material drone,
            Material critical, Material serious, Material stable)
        {
            config = scenario;
            groundMaterial = ground;
            rubbleMaterial = rubble;
            fireMaterial = fire;
            hospitalMaterial = hospital;
            chargingMaterial = charging;
            droneMaterial = drone;
            patientCriticalMaterial = critical;
            patientSeriousMaterial = serious;
            patientStableMaterial = stable;
        }

        /// <summary>
        /// Rebuilds every scene object described by the scenario config, so the
        /// visible scene and the navigation grid cannot drift apart. Everything it
        /// creates lives under a single "Generated" child, which is cleared first.
        ///
        /// Runs in the editor only. It is a build step, not gameplay.
        /// </summary>
        [ContextMenu("Rebuild Scene From Scenario")]
        public void RebuildSceneFromScenario()
        {
            if (config == null)
            {
                Debug.LogError("[DisasterEnvironment] Assign a ScenarioConfig before rebuilding.");
                return;
            }

            var existing = transform.Find(GeneratedRootName);
            if (existing != null)
                DestroyImmediate(existing.gameObject);

            var root = new GameObject(GeneratedRootName).transform;
            root.SetParent(transform, false);

            BuildGround(root);
            BuildObstacleVisuals(root);
            BuildSites(root);
            BuildDrones(root);
            BuildPatients(root);

            Debug.Log("[DisasterEnvironment] Scene rebuilt: " + config.drones.Count + " drones, "
                      + config.patients.Count + " patients, " + config.obstacles.Count + " obstacles, "
                      + config.hospitals.Count + " hospitals, " + config.chargingStations.Count + " charging stations.");
        }

        private void BuildGround(Transform root)
        {
            float worldWidth = config.gridWidth * config.cellSize;
            float worldHeight = config.gridHeight * config.cellSize;

            var ground = CreatePrimitive(PrimitiveType.Plane, "Ground", root, groundMaterial);
            // Unity's Plane primitive is 10 by 10 units at unit scale.
            ground.transform.localScale = new Vector3(worldWidth / 10f, 1f, worldHeight / 10f);
            ground.transform.position = new Vector3(
                config.gridOrigin.x + worldWidth * 0.5f,
                config.gridOrigin.y,
                config.gridOrigin.z + worldHeight * 0.5f);
        }

        private void BuildObstacleVisuals(Transform root)
        {
            var group = new GameObject("Obstacles").transform;
            group.SetParent(root, false);

            for (int i = 0; i < config.obstacles.Count; i++)
            {
                var def = config.obstacles[i];
                var material = def.isDynamic ? fireMaterial : rubbleMaterial;
                string label = def.label + "_" + (i + 1).ToString("00");

                if (def.shape == ObstacleShape.Box)
                {
                    var box = CreatePrimitive(PrimitiveType.Cube, label, group, material);
                    box.transform.localScale = new Vector3(def.halfExtents.x * 2f, def.visualHeight, def.halfExtents.z * 2f);
                    box.transform.position = new Vector3(def.center.x, def.visualHeight * 0.5f, def.center.z);
                }
                else
                {
                    var cyl = CreatePrimitive(PrimitiveType.Cylinder, label, group, material);
                    cyl.transform.localScale = new Vector3(def.radius * 2f, def.visualHeight * 0.5f, def.radius * 2f);
                    cyl.transform.position = new Vector3(def.center.x, def.visualHeight * 0.5f, def.center.z);
                }
            }
        }

        private void BuildSites(Transform root)
        {
            var group = new GameObject("Sites").transform;
            group.SetParent(root, false);

            for (int i = 0; i < config.hospitals.Count; i++)
            {
                var pad = CreatePrimitive(PrimitiveType.Cube, "Hospital_" + (i + 1).ToString("00"), group, hospitalMaterial);
                pad.transform.localScale = new Vector3(8f, 0.4f, 8f);
                pad.transform.position = new Vector3(config.hospitals[i].x, 0.2f, config.hospitals[i].z);
            }

            for (int i = 0; i < config.chargingStations.Count; i++)
            {
                var pad = CreatePrimitive(PrimitiveType.Cube, "ChargingStation_" + (i + 1).ToString("00"), group, chargingMaterial);
                pad.transform.localScale = new Vector3(6f, 0.3f, 6f);
                pad.transform.position = new Vector3(config.chargingStations[i].x, 0.15f, config.chargingStations[i].z);
            }
        }

        private void BuildDrones(Transform root)
        {
            var group = new GameObject("Drones").transform;
            group.SetParent(root, false);

            for (int i = 0; i < config.drones.Count; i++)
            {
                var def = config.drones[i];
                var body = CreatePrimitive(PrimitiveType.Sphere, def.id, group, droneMaterial);
                body.transform.localScale = Vector3.one * 1.6f;
                body.transform.position = new Vector3(def.spawnPosition.x, config.droneAltitude, def.spawnPosition.z);

                var agent = body.AddComponent<DroneAgent>();
                agent.Configure(def.id, def.startBatteryPercent);

                // Debug aid only: draws the drone's planned route. Creates its line
                // object at runtime, so nothing extra is saved into the scene.
                body.AddComponent<RouteVisualizer>();
            }
        }

        private void BuildPatients(Transform root)
        {
            var group = new GameObject("Patients").transform;
            group.SetParent(root, false);

            for (int i = 0; i < config.patients.Count; i++)
            {
                var def = config.patients[i];

                Material material;
                if (def.priority == PatientPriority.Critical)
                    material = patientCriticalMaterial;
                else if (def.priority == PatientPriority.Serious)
                    material = patientSeriousMaterial;
                else
                    material = patientStableMaterial;

                var body = CreatePrimitive(PrimitiveType.Capsule, def.id, group, material);
                body.transform.localScale = new Vector3(1.2f, 1.0f, 1.2f);
                body.transform.position = new Vector3(def.location.x, 1.0f, def.location.z);

                var marker = body.AddComponent<PatientMarker>();
                marker.Configure(def.id, def.priority);
            }
        }

        private static GameObject CreatePrimitive(PrimitiveType type, string name, Transform parent, Material material)
        {
            var go = GameObject.CreatePrimitive(type);
            go.name = name;
            go.transform.SetParent(parent, false);

            // No physics in this simulation, so colliders are dead weight.
            var collider = go.GetComponent<Collider>();
            if (collider != null)
                DestroyImmediate(collider);

            if (material != null)
                go.GetComponent<Renderer>().sharedMaterial = material;

            return go;
        }
#endif
    }
}
