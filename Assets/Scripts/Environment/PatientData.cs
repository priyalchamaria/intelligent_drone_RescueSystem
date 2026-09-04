using UnityEngine;

namespace DroneRescue.Environment
{
    /// <summary>
    /// Triage category, inspired by the START triage categories. This orders the
    /// Mission Planner's priority queue: which patient gets served first.
    ///
    /// NOT to be confused with RiskFactor(location), which is a separate quantity
    /// computed from where the patient is (fire proximity and so on) and used
    /// inside drone scoring. Priority ranks patients. Risk ranks locations. They
    /// are deliberately never merged.
    ///
    /// Lower numeric value means higher urgency, so the value doubles as the
    /// priority-queue key.
    /// </summary>
    public enum PatientPriority
    {
        Critical = 0,
        Serious = 1,
        Stable = 2
    }

    /// <summary>How a patient's triage category gets decided at spawn time.</summary>
    public enum TriageMode
    {
        /// <summary>Read the category authored in the scenario. Predictable demos.</summary>
        Manual = 0,

        /// <summary>Draw the category from configured weights. Generated scenarios.</summary>
        WeightedRandom = 1
    }

    /// <summary>Stage a patient has reached in the rescue pipeline. Used for metrics in Phase 7.</summary>
    public enum PatientState
    {
        Waiting = 0,
        Assigned = 1,
        Reached = 2,
        Delivered = 3
    }

    /// <summary>
    /// The Patient record from Part 1: id, location, priority.
    ///
    /// DETECTION MODEL: a patient enters the simulation by being constructed and
    /// pushed straight into the environment's shared state. There is no network,
    /// no call handling and no messaging layer. In a real deployment this instant
    /// would be the moment a drone's thermal or camera payload autonomously
    /// detected a casualty, per the base paper. Here it is simply an object
    /// appearing in shared state.
    /// </summary>
    [System.Serializable]
    public class Patient
    {
        public string id;
        public Vector3 location;
        public PatientPriority priority;

        [Header("Runtime tracking (not part of the Part 1 record)")]
        public PatientState state = PatientState.Waiting;

        /// <summary>Simulated time at which this patient entered the queue. Feeds Average Response Time.</summary>
        public float queuedAtTime;

        /// <summary>Simulated time a drone first arrived. Negative until it happens.</summary>
        public float reachedAtTime = -1f;

        /// <summary>
        /// Simulated time this casualty was handed over at a hospital. Negative
        /// until it happens, so a run that ends with somebody still waiting is
        /// distinguishable from one where everybody was delivered at time zero.
        /// </summary>
        public float deliveredAtTime = -1f;

        /// <summary>Id of the drone currently assigned, or null when unassigned.</summary>
        public string assignedDroneId;

        public Patient(string id, Vector3 location, PatientPriority priority, float queuedAtTime = 0f)
        {
            this.id = id;
            this.location = location;
            this.priority = priority;
            this.queuedAtTime = queuedAtTime;
        }

        public override string ToString() => $"{id} [{priority}] {state} at {location}";
    }

    /// <summary>
    /// Assigns a triage category at the moment of detection.
    ///
    /// Patients do not self-report priority, so something has to decide. This is a
    /// triage-STYLE rule inspired by START categories, deliberately not a medical
    /// simulation. Both modes required by Part 1 are implemented here, and the
    /// active one is chosen by the scenario configuration.
    /// </summary>
    [System.Serializable]
    public class TriageAssigner
    {
        [Tooltip("Manual reads the category authored per patient. WeightedRandom draws from the weights below.")]
        public TriageMode mode = TriageMode.Manual;

        [Header("Weights used only in WeightedRandom mode")]
        [Min(0f)] public float criticalWeight = 0.25f;
        [Min(0f)] public float seriousWeight = 0.35f;
        [Min(0f)] public float stableWeight = 0.40f;

        [Tooltip("Fixed seed keeps generated scenarios reproducible across runs. Set to 0 for a time-based seed.")]
        public int randomSeed = 12345;

        private System.Random _random;

        public void Reset()
        {
            _random = randomSeed == 0
                ? new System.Random()
                : new System.Random(randomSeed);
        }

        /// <summary>
        /// Returns the triage category for a newly detected patient. In Manual mode
        /// the authored value passes through unchanged; in WeightedRandom mode it is
        /// ignored and a category is drawn from the weights.
        /// </summary>
        public PatientPriority Assign(PatientPriority authoredPriority)
        {
            if (mode == TriageMode.Manual)
                return authoredPriority;

            if (_random == null)
                Reset();

            float total = criticalWeight + seriousWeight + stableWeight;
            if (total <= 0f)
                return authoredPriority;

            double roll = _random.NextDouble() * total;

            if (roll < criticalWeight)
                return PatientPriority.Critical;
            if (roll < criticalWeight + seriousWeight)
                return PatientPriority.Serious;
            return PatientPriority.Stable;
        }
    }
}
