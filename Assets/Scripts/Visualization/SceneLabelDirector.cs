using System.Collections.Generic;
using UnityEngine;
using DroneRescue.Environment;
using DroneRescue.Fleet;

namespace DroneRescue.Visualization
{
    /// <summary>
    /// Puts a floating caption above every drone, every casualty, the hospital and
    /// each charging pad.
    ///
    /// READ ONLY. It looks at shared state and at scene transforms and writes to
    /// neither. Switching this component off changes what a run looks like and
    /// nothing about what it does.
    ///
    /// It labels in LateUpdate rather than once at Start because the world is not
    /// fixed: a NewEmergency event drops a fresh casualty into the scene mid-run,
    /// and that casualty needs a caption on the frame it appears, without the
    /// environment having to know that captions exist.
    /// </summary>
    [DisallowMultipleComponent]
    public class SceneLabelDirector : MonoBehaviour
    {
        [SerializeField] private DisasterEnvironment environment;

        [Header("What to label")]
        [SerializeField] private bool labelDrones = true;
        [SerializeField] private bool labelPatients = true;
        [SerializeField] private bool labelSites = true;

        [Header("Placement")]
        [Tooltip("Height of a drone's caption above the drone body.")]
        [SerializeField] private float droneLabelHeight = 2.6f;

        [SerializeField] private float patientLabelHeight = 3.0f;
        [SerializeField] private float siteLabelHeight = 4.0f;

        [Header("Size")]
        [SerializeField] private float droneFontSize = 30f;
        [SerializeField] private float patientFontSize = 26f;
        [SerializeField] private float siteFontSize = 26f;

        private Transform _root;
        private bool _sitesLabelled;

        private readonly Dictionary<DroneAgent, WorldLabel> _droneLabels = new Dictionary<DroneAgent, WorldLabel>();

        /// <summary>A casualty's caption, plus the state it was last written for.</summary>
        private class PatientLabel
        {
            public WorldLabel label;
            public PatientState state;
        }

        private readonly Dictionary<PatientMarker, PatientLabel> _patientLabels =
            new Dictionary<PatientMarker, PatientLabel>();

        private void Awake()
        {
            if (environment == null)
                environment = FindAnyObjectByType<DisasterEnvironment>();
        }

        private void LateUpdate()
        {
            if (environment == null)
                return;

            EnsureRoot();

            if (labelSites && !_sitesLabelled)
                LabelSites();

            if (labelDrones)
                LabelDrones();

            if (labelPatients)
                LabelPatients();
        }

        /// <summary>
        /// One parent for every caption, marked DontSave so a play session cannot
        /// leave label objects behind in the scene asset.
        /// </summary>
        private void EnsureRoot()
        {
            if (_root != null)
                return;

            var go = new GameObject("SceneLabels");
            go.hideFlags = HideFlags.DontSave;
            _root = go.transform;
        }

        private void LabelDrones()
        {
            var agents = environment.DroneAgents;

            for (int i = 0; i < agents.Count; i++)
            {
                var agent = agents[i];
                if (agent == null || _droneLabels.ContainsKey(agent))
                    continue;

                // The caption takes the same colour as this drone's route line, so
                // a line crossing the map can be traced back to a named drone
                // without counting bodies.
                var color = FleetPalette.RouteColor(agent.DroneId);
                var label = WorldLabel.Attach(agent.transform, _root, agent.DroneId,
                                              color, droneLabelHeight, droneFontSize);
                _droneLabels[agent] = label;
            }
        }

        private void LabelPatients()
        {
            var markers = environment.PatientMarkers;

            for (int i = 0; i < markers.Count; i++)
            {
                var marker = markers[i];
                if (marker == null || marker.Data == null)
                    continue;

                if (_patientLabels.TryGetValue(marker, out var existing))
                {
                    // Rewritten only when the casualty actually moves through the
                    // pipeline, not every frame.
                    if (existing.state != marker.Data.state)
                    {
                        existing.state = marker.Data.state;
                        WritePatientCaption(marker, existing.label);
                    }

                    continue;
                }

                var label = WorldLabel.Attach(marker.transform, _root, marker.Data.id,
                                              FleetPalette.ForPriority(marker.Data.priority),
                                              patientLabelHeight, patientFontSize);

                var record = new PatientLabel { label = label, state = marker.Data.state };
                _patientLabels[marker] = record;
                WritePatientCaption(marker, label);
            }
        }

        /// <summary>
        /// Id and triage priority, as asked for, plus the stage the casualty has
        /// reached. Without the stage a delivered casualty keeps a bright red
        /// CRITICAL caption for the rest of the run, which reads as an outstanding
        /// emergency when it is a completed rescue.
        /// </summary>
        private void WritePatientCaption(PatientMarker marker, WorldLabel label)
        {
            if (label == null)
                return;

            var patient = marker.Data;
            bool done = patient.state == PatientState.Delivered;

            string caption = patient.id + "  " + patient.priority.ToString().ToUpperInvariant();
            if (done)
                caption = patient.id + "  RESCUED";
            else if (patient.state == PatientState.Reached)
                caption += "  (picked up)";
            else if (patient.state == PatientState.Assigned)
                caption += "  (drone inbound)";

            label.SetText(caption, done ? FleetPalette.Rescued : FleetPalette.ForPriority(patient.priority));
        }

        private void LabelSites()
        {
            var config = environment.Config;
            if (config == null)
                return;

            for (int i = 0; i < config.hospitals.Count; i++)
            {
                string caption = config.hospitals.Count > 1 ? "HOSPITAL " + (i + 1) : "HOSPITAL";
                WorldLabel.AttachToPoint(config.hospitals[i], _root, caption,
                                         FleetPalette.Hospital, siteLabelHeight, siteFontSize);
            }

            for (int i = 0; i < config.chargingStations.Count; i++)
            {
                WorldLabel.AttachToPoint(config.chargingStations[i], _root,
                                         "CHARGING " + (i + 1).ToString("00"),
                                         FleetPalette.Charging, siteLabelHeight, siteFontSize);
            }

            _sitesLabelled = true;
        }

        private void OnDisable()
        {
            if (_root != null)
                Destroy(_root.gameObject);

            _root = null;
            _sitesLabelled = false;
            _droneLabels.Clear();
            _patientLabels.Clear();
        }

#if UNITY_EDITOR
        public void EditorAssign(DisasterEnvironment env)
        {
            environment = env;
        }
#endif
    }
}
