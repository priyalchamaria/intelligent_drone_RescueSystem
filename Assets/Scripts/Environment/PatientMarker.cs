using UnityEngine;

namespace DroneRescue.Environment
{
    /// <summary>
    /// The scene-side marker for one patient. Holds the Patient record that the
    /// Mission Planner's priority queue will draw from, and nothing else. Phase 1
    /// patients are static.
    /// </summary>
    public class PatientMarker : MonoBehaviour
    {
        [SerializeField] private string patientId = "P-01";
        [SerializeField] private PatientPriority authoredPriority = PatientPriority.Serious;

        /// <summary>The shared-state record for this patient.</summary>
        public Patient Data { get; private set; }

        public string PatientId => patientId;
        public PatientPriority AuthoredPriority => authoredPriority;

        /// <summary>Called by the editor-time scene builder.</summary>
        public void Configure(string id, PatientPriority priority)
        {
            patientId = id;
            authoredPriority = priority;
        }

        /// <summary>
        /// Binds an already-triaged Patient record to this marker. The environment
        /// creates the record so that triage runs in exactly one place.
        /// </summary>
        public void Bind(Patient patient)
        {
            Data = patient;
            transform.position = new Vector3(patient.location.x, transform.position.y, patient.location.z);
        }
    }
}
