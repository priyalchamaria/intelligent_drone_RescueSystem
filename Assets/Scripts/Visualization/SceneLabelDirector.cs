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
    ///
    /// CAPTIONS ARE RANKED, NOT UNIFORM. A caption costs the viewer attention, and
    /// the ones that cost the most are the ones that say least: six drones parked
    /// on the hospital ring with nothing to do, and pad names that have not changed
    /// since the run started. Those are shrunk and faded, and they yield their
    /// screen position to a drone in the air or a casualty still waiting. WorldLabel
    /// does the pushing apart; what matters here is deciding who gives way.
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
        [SerializeField] private float droneLabelHeight = 2.4f;

        [SerializeField] private float patientLabelHeight = 2.8f;
        [SerializeField] private float siteLabelHeight = 3.6f;

        [Header("Size")]
        [Tooltip("Base size for a drone in the air. Parked drones are drawn smaller than this.")]
        [SerializeField] private float droneFontSize = 20f;

        [SerializeField] private float patientFontSize = 18f;
        [SerializeField] private float siteFontSize = 17f;

        [Header("Parked drones and scenery")]
        [Tooltip("Size and opacity of a drone that is idle, charging or offline, relative to one in flight.")]
        [SerializeField, Range(0.3f, 1f)] private float restingScale = 0.78f;

        [SerializeField, Range(0.1f, 1f)] private float restingAlpha = 0.66f;
        [SerializeField, Range(0.3f, 1f)] private float siteScale = 0.85f;
        [SerializeField, Range(0.1f, 1f)] private float siteAlpha = 0.7f;

        // Lower goes first and keeps its natural spot; see WorldLabel.SortPriority.
        //
        // The pads come first even though they matter least, because they are the
        // only captions that never move: pinning them means everything else dodges
        // a fixed obstacle rather than the pad name drifting whenever a drone lands
        // on it. Parked drones come last, so the pile-up on the hospital ring
        // resolves by moving the captions that say the least.
        private const int PrioritySite = 0;
        private const int PriorityOpenCasualty = 1;
        private const int PriorityFlyingDrone = 2;
        private const int PriorityDoneCasualty = 3;
        private const int PriorityParkedDrone = 4;

        private Transform _root;
        private bool _sitesLabelled;

        private class DroneLabel
        {
            public WorldLabel label;
            public bool flying;
            public bool initialised;
        }

        private readonly Dictionary<DroneAgent, DroneLabel> _droneLabels = new Dictionary<DroneAgent, DroneLabel>();

        /// <summary>A casualty's caption, plus the state it was last written for.</summary>
        private class PatientLabel
        {
            public WorldLabel label;
            public PatientState state;
            public bool initialised;
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
                if (agent == null || agent.Data == null)
                    continue;

                if (!_droneLabels.TryGetValue(agent, out var record))
                {
                    // The caption takes the same colour as this drone's route line,
                    // so a line crossing the map can be traced back to a named drone
                    // without counting bodies.
                    var color = FleetPalette.RouteColor(agent.DroneId);
                    var label = WorldLabel.Attach(agent.transform, _root, agent.DroneId,
                                                  color, droneLabelHeight, droneFontSize);

                    record = new DroneLabel { label = label };
                    _droneLabels[agent] = record;
                }

                bool flying = agent.Data.status == DroneStatus.Busy;
                if (record.initialised && record.flying == flying)
                    continue;

                record.flying = flying;
                record.initialised = true;
                ApplyDroneEmphasis(record, flying);
            }
        }

        private void ApplyDroneEmphasis(DroneLabel record, bool flying)
        {
            if (record.label == null)
                return;

            record.label.SortPriority = flying ? PriorityFlyingDrone : PriorityParkedDrone;
            record.label.SetEmphasis(flying ? 1f : restingScale, flying ? 1f : restingAlpha);
        }

        private void LabelPatients()
        {
            var markers = environment.PatientMarkers;

            for (int i = 0; i < markers.Count; i++)
            {
                var marker = markers[i];
                if (marker == null || marker.Data == null)
                    continue;

                if (!_patientLabels.TryGetValue(marker, out var record))
                {
                    var label = WorldLabel.Attach(marker.transform, _root, marker.Data.id,
                                                  FleetPalette.ForPriority(marker.Data.priority),
                                                  patientLabelHeight, patientFontSize);

                    record = new PatientLabel { label = label, state = marker.Data.state };
                    _patientLabels[marker] = record;
                }
                else if (record.initialised && record.state == marker.Data.state)
                {
                    // Rewritten only when the casualty actually moves through the
                    // pipeline, not every frame.
                    continue;
                }

                record.state = marker.Data.state;
                record.initialised = true;
                WritePatientCaption(marker, record.label);
            }
        }

        /// <summary>
        /// Identity and triage priority on one short line.
        ///
        /// The stage is folded into that same line rather than added below it. Two
        /// stacked lines doubled a caption's height for a word, and height is what
        /// makes captions collide. The priority is dropped once the casualty is
        /// delivered, because at that point it is a record of a finished job and no
        /// longer something anyone has to act on.
        /// </summary>
        private void WritePatientCaption(PatientMarker marker, WorldLabel label)
        {
            if (label == null)
                return;

            var patient = marker.Data;
            bool done = patient.state == PatientState.Delivered;

            string caption;
            if (done)
                caption = patient.id + " OK";
            else if (patient.state == PatientState.Reached)
                caption = patient.id + " UP";
            else
                caption = patient.id + " " + ShortPriority(patient.priority);

            label.SortPriority = done ? PriorityDoneCasualty : PriorityOpenCasualty;
            label.SetText(caption, done ? FleetPalette.Rescued : FleetPalette.ForPriority(patient.priority));
            label.SetEmphasis(done ? 0.8f : 1f, done ? 0.6f : 1f);
        }

        private static string ShortPriority(PatientPriority priority)
        {
            if (priority == PatientPriority.Critical) return "CRIT";
            if (priority == PatientPriority.Serious) return "SER";
            return "STAB";
        }

        private void LabelSites()
        {
            var config = environment.Config;
            if (config == null)
                return;

            for (int i = 0; i < config.hospitals.Count; i++)
            {
                string caption = config.hospitals.Count > 1 ? "HOSPITAL " + (i + 1) : "HOSPITAL";
                Dress(WorldLabel.AttachToPoint(config.hospitals[i], _root, caption,
                                               FleetPalette.Hospital, siteLabelHeight, siteFontSize));
            }

            for (int i = 0; i < config.chargingStations.Count; i++)
            {
                Dress(WorldLabel.AttachToPoint(config.chargingStations[i], _root, "CHARGE " + (i + 1),
                                               FleetPalette.Charging, siteLabelHeight, siteFontSize));
            }

            _sitesLabelled = true;
        }

        private void Dress(WorldLabel label)
        {
            if (label == null)
                return;

            label.SortPriority = PrioritySite;
            label.SetEmphasis(siteScale, siteAlpha);
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
