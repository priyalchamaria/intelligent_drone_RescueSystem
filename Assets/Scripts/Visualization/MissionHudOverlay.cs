using System.Collections.Generic;
using UnityEngine;
using DroneRescue.Environment;

namespace DroneRescue.Visualization
{
    /// <summary>
    /// The screen-space layer: event banners, a running mission log, and a legend.
    ///
    /// Drawn with Unity's immediate-mode GUI rather than a Canvas on purpose. A
    /// Canvas would mean new objects, a new event system and new prefabs saved into
    /// the scene asset, all of which are things a later phase would have to keep in
    /// step. This draws from code, saves nothing, and can be switched off with one
    /// checkbox.
    ///
    /// It listens to RescueFeed and holds no reference to the planner, so nothing
    /// it does can reach the decision logic. Three deliberately separate registers:
    ///
    ///   BANNER   compact, brief, centre top. The four Part 1 dynamic events only.
    ///            Reserved for them so that a banner always means the world changed.
    ///   LOG      small, persistent, bottom left. Routine progress, glanceable.
    ///   LEGEND   small, persistent, bottom right. What the colours mean.
    /// </summary>
    [DisallowMultipleComponent]
    public class MissionHudOverlay : MonoBehaviour
    {
        [SerializeField] private DisasterEnvironment environment;

        [Header("What to show")]
        [SerializeField] private bool showBanners = true;
        [SerializeField] private bool showLog = true;
        [SerializeField] private bool showLegend = true;

        [Header("Banner")]
        [Tooltip("How long each event banner stays up. Banners queue rather than overwrite.")]
        [SerializeField, Min(0.5f)] private float bannerSeconds = 2.5f;

        [Header("Mission log")]
        [SerializeField, Min(1)] private int logLines = 9;

        private class Banner
        {
            public AlertKind kind;
            public string message;
        }

        private readonly Queue<Banner> _pending = new Queue<Banner>();
        private readonly List<string> _log = new List<string>();
        private Banner _current;
        private float _bannerShownAt;

        private Texture2D _pixel;
        private GUIStyle _bannerStyle;
        private GUIStyle _bannerKindStyle;
        private GUIStyle _logStyle;
        private GUIStyle _logHeadStyle;
        private GUIStyle _legendStyle;
        private GUIStyle _legendHeadStyle;
        private bool _stylesBuilt;

        private void Awake()
        {
            if (environment == null)
                environment = FindAnyObjectByType<DisasterEnvironment>();
        }

        private void OnEnable()
        {
            RescueFeed.Alert += OnAlert;
            RescueFeed.Note += OnNote;
        }

        private void OnDisable()
        {
            // Unsubscribing matters more than usual here: RescueFeed is static, so a
            // handler left behind would outlive play mode and fire into a destroyed
            // component on the next run.
            RescueFeed.Alert -= OnAlert;
            RescueFeed.Note -= OnNote;

            _pending.Clear();
            _log.Clear();
            _current = null;
        }

        private void OnAlert(AlertKind kind, string message)
        {
            _pending.Enqueue(new Banner { kind = kind, message = message });

            // A backlog would show banners long after the moment they describe. The
            // events themselves are still in the console and the log either way.
            while (_pending.Count > 3)
                _pending.Dequeue();
        }

        private void OnNote(string message)
        {
            _log.Add(Time.timeSinceLevelLoad.ToString("F1") + "s   " + message);

            while (_log.Count > logLines)
                _log.RemoveAt(0);
        }

        private void Update()
        {
            if (_current != null && Time.unscaledTime - _bannerShownAt >= bannerSeconds)
                _current = null;

            if (_current == null && _pending.Count > 0)
            {
                _current = _pending.Dequeue();
                _bannerShownAt = Time.unscaledTime;
            }
        }

        private void OnGUI()
        {
            BuildStyles();

            float s = Mathf.Max(0.8f, Screen.height / 900f);

            if (showBanners && _current != null)
                DrawBanner(s);

            if (showLog && _log.Count > 0)
                DrawLog(s);

            if (showLegend)
                DrawLegend(s);
        }

        // -----------------------------------------------------------------
        // Banner
        // -----------------------------------------------------------------

        private void DrawBanner(float s)
        {
            float age = Time.unscaledTime - _bannerShownAt;

            // Eased in quickly and out slowly, so a banner never simply vanishes
            // between two frames of recorded footage.
            float alpha = Mathf.Min(age / 0.12f, Mathf.Clamp01((bannerSeconds - age) / 0.5f));
            alpha = Mathf.Clamp01(alpha);

            // Sized to be caught out of the corner of the eye and then read, not to
            // take the screen over. The colour bar and the kind line carry the
            // noticing; the box only has to hold one sentence.
            float width = Mathf.Min(Screen.width * 0.52f, 560f * s);
            float height = 44f * s;
            var box = new Rect((Screen.width - width) * 0.5f, 18f * s, width, height);

            var accent = KindColor(_current.kind);

            Fill(box, new Color(0.05f, 0.05f, 0.07f, 0.88f * alpha));
            Fill(new Rect(box.x, box.y, 5f * s, box.height), new Color(accent.r, accent.g, accent.b, alpha));

            var kindRect = new Rect(box.x + 14f * s, box.y + 5f * s, box.width - 22f * s, 13f * s);
            var textRect = new Rect(box.x + 14f * s, box.y + 18f * s, box.width - 22f * s, box.height - 22f * s);

            _bannerKindStyle.fontSize = Mathf.RoundToInt(10f * s);
            _bannerStyle.fontSize = Mathf.RoundToInt(17f * s);

            var prev = GUI.color;
            GUI.color = new Color(accent.r, accent.g, accent.b, alpha);
            GUI.Label(kindRect, KindTitle(_current.kind), _bannerKindStyle);
            GUI.color = new Color(1f, 1f, 1f, alpha);
            GUI.Label(textRect, _current.message, _bannerStyle);
            GUI.color = prev;
        }

        private static string KindTitle(AlertKind kind)
        {
            if (kind == AlertKind.FireSpread) return "FIRE SPREAD";
            if (kind == AlertKind.BatteryLow) return "BATTERY LOW";
            if (kind == AlertKind.DroneFailed) return "DRONE FAILURE";
            return "NEW EMERGENCY";
        }

        private static Color KindColor(AlertKind kind)
        {
            if (kind == AlertKind.FireSpread) return new Color(0.98f, 0.46f, 0.13f);
            if (kind == AlertKind.BatteryLow) return new Color(0.99f, 0.79f, 0.15f);
            if (kind == AlertKind.DroneFailed) return new Color(0.92f, 0.22f, 0.22f);
            return new Color(0.36f, 0.70f, 1.00f);
        }

        // -----------------------------------------------------------------
        // Mission log
        // -----------------------------------------------------------------

        private void DrawLog(float s)
        {
            _logStyle.fontSize = Mathf.RoundToInt(13f * s);
            _logHeadStyle.fontSize = Mathf.RoundToInt(11f * s);

            float lineHeight = 17f * s;
            float width = 330f * s;
            float height = lineHeight * _log.Count + 26f * s;
            var box = new Rect(14f * s, Screen.height - height - 14f * s, width, height);

            Fill(box, new Color(0.04f, 0.05f, 0.06f, 0.66f));

            GUI.Label(new Rect(box.x + 10f * s, box.y + 5f * s, box.width, 14f * s), "MISSION LOG", _logHeadStyle);

            for (int i = 0; i < _log.Count; i++)
            {
                // Oldest at the top, newest at the bottom, so the eye can rest at
                // one edge instead of following a list that reorders itself.
                float fade = Mathf.Lerp(0.55f, 1f, _log.Count == 1 ? 1f : i / (float)(_log.Count - 1));
                var prev = GUI.color;
                GUI.color = new Color(0.88f, 0.92f, 0.95f, fade);
                GUI.Label(new Rect(box.x + 10f * s, box.y + 22f * s + i * lineHeight, box.width - 16f * s, lineHeight),
                          _log[i], _logStyle);
                GUI.color = prev;
            }
        }

        // -----------------------------------------------------------------
        // Legend
        // -----------------------------------------------------------------

        private void DrawLegend(float s)
        {
            _legendStyle.fontSize = Mathf.RoundToInt(12f * s);
            _legendHeadStyle.fontSize = Mathf.RoundToInt(11f * s);

            var fleet = environment != null ? environment.DroneList : null;
            int droneRows = fleet != null ? fleet.Count : 0;

            float lineHeight = 16f * s;
            float width = 172f * s;
            int rows = 4 + 1 + 3 + (droneRows > 0 ? 1 + droneRows : 0);
            float height = lineHeight * rows + 34f * s;

            var box = new Rect(Screen.width - width - 14f * s, Screen.height - height - 14f * s, width, height);
            Fill(box, new Color(0.04f, 0.05f, 0.06f, 0.72f));

            GUI.Label(new Rect(box.x + 10f * s, box.y + 5f * s, box.width, 14f * s), "LEGEND", _legendHeadStyle);

            float y = box.y + 22f * s;
            y = Row(box, y, s, lineHeight, FleetPalette.Critical, "Patient  Critical");
            y = Row(box, y, s, lineHeight, FleetPalette.Serious, "Patient  Serious");
            y = Row(box, y, s, lineHeight, FleetPalette.Stable, "Patient  Stable");
            y = Row(box, y, s, lineHeight, FleetPalette.Rescued, "Patient  Rescued");
            y += lineHeight * 0.35f;

            y = Row(box, y, s, lineHeight, FleetPalette.Hospital, "Hospital pad");
            y = Row(box, y, s, lineHeight, FleetPalette.Charging, "Charging pad");
            y = Row(box, y, s, lineHeight, FleetPalette.Fire, "Fire zone");

            if (droneRows > 0)
            {
                y += lineHeight * 0.35f;
                for (int i = 0; i < fleet.Count; i++)
                    y = Row(box, y, s, lineHeight, FleetPalette.RouteColor(fleet[i].id), fleet[i].id + "  route");
            }
        }

        private float Row(Rect box, float y, float s, float lineHeight, Color swatch, string caption)
        {
            var chip = new Rect(box.x + 10f * s, y + 3f * s, 11f * s, 11f * s);
            Fill(chip, swatch);
            GUI.Label(new Rect(chip.xMax + 8f * s, y, box.width - 34f * s, lineHeight), caption, _legendStyle);
            return y + lineHeight;
        }

        // -----------------------------------------------------------------
        // Drawing helpers
        // -----------------------------------------------------------------

        private void Fill(Rect rect, Color color)
        {
            var prev = GUI.color;
            GUI.color = color;
            GUI.DrawTexture(rect, _pixel);
            GUI.color = prev;
        }

        /// <summary>
        /// Styles are built inside OnGUI because GUI.skin is only valid there, and
        /// only once because building them per frame allocates every frame.
        /// </summary>
        private void BuildStyles()
        {
            if (_stylesBuilt && _pixel != null)
                return;

            _pixel = new Texture2D(1, 1) { hideFlags = HideFlags.DontSave };
            _pixel.SetPixel(0, 0, Color.white);
            _pixel.Apply();

            _bannerStyle = new GUIStyle(GUI.skin.label)
            {
                fontStyle = FontStyle.Bold,
                alignment = TextAnchor.MiddleLeft,
                wordWrap = false
            };

            _bannerKindStyle = new GUIStyle(GUI.skin.label)
            {
                fontStyle = FontStyle.Bold,
                alignment = TextAnchor.MiddleLeft
            };

            _logStyle = new GUIStyle(GUI.skin.label) { alignment = TextAnchor.MiddleLeft, wordWrap = false };
            _logHeadStyle = new GUIStyle(GUI.skin.label) { alignment = TextAnchor.MiddleLeft, fontStyle = FontStyle.Bold };
            _legendStyle = new GUIStyle(GUI.skin.label) { alignment = TextAnchor.MiddleLeft, wordWrap = false };
            _legendHeadStyle = new GUIStyle(GUI.skin.label) { alignment = TextAnchor.MiddleLeft, fontStyle = FontStyle.Bold };

            var head = new Color(0.62f, 0.68f, 0.74f);
            _logHeadStyle.normal.textColor = head;
            _legendHeadStyle.normal.textColor = head;
            _logStyle.normal.textColor = new Color(0.88f, 0.92f, 0.95f);
            _legendStyle.normal.textColor = new Color(0.88f, 0.92f, 0.95f);

            _stylesBuilt = true;
        }

#if UNITY_EDITOR
        public void EditorAssign(DisasterEnvironment env)
        {
            environment = env;
        }
#endif
    }
}
