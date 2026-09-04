using UnityEngine;
using DroneRescue.Visualization;

namespace DroneRescue.Fleet
{
    /// <summary>
    /// Draws a drone's route so the flown path can be compared against the planned
    /// one by eye.
    ///
    /// Purely a debug aid. The line objects and their materials are created at
    /// runtime and never saved into the scene, so nothing here touches the
    /// navigation logic or the scene asset.
    ///
    /// THE ROUTE IS DRAWN IN TWO PIECES, WHICH IS THE POINT OF THIS CLASS. A route
    /// is planned once, from wherever the drone happened to be standing when it was
    /// dispatched, and it does not move afterwards. Drawing that list of waypoints
    /// as one line leaves the near end pinned to a launch point that nothing marks
    /// any more, so on screen the line starts in empty air and the drone appears to
    /// be sitting on top of it rather than flying it.
    ///
    /// So: the part already flown is drawn dark, the part still to fly is drawn
    /// bright, and the two meet at the drone's live position. The bright line is
    /// therefore anchored to a visible object at every instant of the run, and the
    /// dark line still shows where the drone came from. The bright line also tapers
    /// from the drone toward the goal, which gives it a direction, and a short
    /// upright pip marks the goal so the far end lands on something too.
    ///
    /// Each drone gets its own colour from FleetPalette, keyed off its id, so
    /// crossing routes stay separable and match the caption above the drone and the
    /// corner legend.
    /// </summary>
    [RequireComponent(typeof(DroneAgent))]
    public class RouteVisualizer : MonoBehaviour
    {
        [SerializeField] private bool showRoute = true;

        [Tooltip("Leave off to use this drone's colour from FleetPalette, which is what the legend shows.")]
        [SerializeField] private bool overridePaletteColor;

        [SerializeField] private Color customColor = new Color(0.2f, 0.9f, 1f, 1f);

        [Tooltip("Width of the part still to fly. Sized for the camera height this scene is demonstrated from.")]
        [SerializeField] private float lineWidth = 0.8f;

        [Tooltip("Height above the ground plane to draw the line at. Must clear the hospital and charging pads.")]
        [SerializeField] private float lineHeight = 1.2f;

        [Tooltip("Extra height per drone, so two routes crossing resolve as one line over another " +
                 "instead of merging. Small on purpose: it only has to break the depth tie.")]
        [SerializeField, Range(0f, 0.5f)] private float routeLayerSpacing = 0.1f;

        [Tooltip("How far the already-flown part of the route is blended toward the ground colour. 0 keeps it live, 1 hides it.")]
        [SerializeField, Range(0f, 1f)] private float flownFade = 0.55f;

        [Tooltip("Draw a short upright marker where the current leg ends.")]
        [SerializeField] private bool showGoalPip = true;

        private DroneAgent _agent;
        private LineRenderer _ahead;
        private LineRenderer _flown;
        private LineRenderer _pip;
        private Color _color = Color.cyan;
        private float _drawHeight;

        private void Awake()
        {
            _agent = GetComponent<DroneAgent>();
            _color = overridePaletteColor ? customColor : FleetPalette.RouteColor(_agent.DroneId);

            // Each drone draws on its own layer, a few centimetres apart.
            //
            // Two routes crossing at the same height render flush, and the crossing
            // reads as a single line changing colour rather than as two lines. The
            // offset is far too small to see as displacement, roughly a couple of
            // pixels at the camera height this scene is shown from, but it is enough
            // for the depth buffer to order the two consistently, which is all the
            // crossing needs to become legible. The layer comes from the same
            // palette index as the colour, so it is stable across runs.
            _drawHeight = lineHeight + FleetPalette.RouteIndex(_agent.DroneId) * routeLayerSpacing;
        }

        private void LateUpdate()
        {
            var follower = _agent != null ? _agent.Follower : null;

            if (!showRoute || follower == null || !follower.HasRoute || follower.Route.Count < 1)
            {
                Hide();
                return;
            }

            EnsureLines();

            var route = follower.Route;
            int next = Mathf.Clamp(follower.WaypointIndex, 0, route.Count);
            Vector3 here = Flatten(transform.position);

            DrawFlown(route, next, here);
            DrawAhead(route, next, here);
            DrawPip(route);
        }

        /// <summary>Launch point through the last waypoint passed, then on to the drone itself.</summary>
        private void DrawFlown(System.Collections.Generic.IReadOnlyList<Vector3> route, int next, Vector3 here)
        {
            _flown.enabled = true;
            _flown.positionCount = next + 1;

            for (int i = 0; i < next; i++)
                _flown.SetPosition(i, Flatten(route[i]));

            _flown.SetPosition(next, here);
        }

        /// <summary>
        /// The drone itself, then every waypoint still ahead of it. Starting at the
        /// drone rather than at the next waypoint is what keeps the bright line
        /// attached to something visible.
        /// </summary>
        private void DrawAhead(System.Collections.Generic.IReadOnlyList<Vector3> route, int next, Vector3 here)
        {
            int remaining = route.Count - next;
            if (remaining <= 0)
            {
                // Arrived. There is nothing left to fly, so only the trail remains.
                _ahead.enabled = false;
                return;
            }

            _ahead.enabled = true;
            _ahead.positionCount = remaining + 1;
            _ahead.SetPosition(0, here);

            for (int i = 0; i < remaining; i++)
                _ahead.SetPosition(i + 1, Flatten(route[next + i]));
        }

        private void DrawPip(System.Collections.Generic.IReadOnlyList<Vector3> route)
        {
            if (!showGoalPip || _pip == null)
                return;

            var goal = route[route.Count - 1];
            _pip.enabled = _ahead.enabled;
            _pip.positionCount = 2;
            _pip.SetPosition(0, new Vector3(goal.x, 0.5f, goal.z));
            _pip.SetPosition(1, new Vector3(goal.x, 4.0f, goal.z));
        }

        private Vector3 Flatten(Vector3 point) => new Vector3(point.x, _drawHeight, point.z);

        private void Hide()
        {
            if (_ahead != null) _ahead.enabled = false;
            if (_flown != null) _flown.enabled = false;
            if (_pip != null) _pip.enabled = false;
        }

        private void EnsureLines()
        {
            if (_ahead != null)
                return;

            _ahead = CreateLine("_Route", _color, lineWidth);

            // Tapered from the drone toward the goal, so which way the drone is
            // going is readable from a still frame.
            _ahead.widthCurve = new AnimationCurve(new Keyframe(0f, 1f), new Keyframe(1f, 0.45f));
            _ahead.widthMultiplier = lineWidth;

            // Blended toward the ground rather than made transparent: the unlit
            // material these lines use is opaque, so an alpha here would simply be
            // ignored, and blending is what actually lowers the contrast.
            _flown = CreateLine("_RouteFlown", FleetPalette.Receded(_color, flownFade), lineWidth * 0.45f);

            if (showGoalPip)
                _pip = CreateLine("_RouteGoal", _color, lineWidth * 0.7f);
        }

        private LineRenderer CreateLine(string suffix, Color color, float width)
        {
            var go = new GameObject(name + suffix);
            go.transform.SetParent(transform.parent, false);
            go.hideFlags = HideFlags.DontSave;

            var line = go.AddComponent<LineRenderer>();
            line.useWorldSpace = true;
            line.widthMultiplier = width;
            line.numCornerVertices = 2;
            line.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
            line.receiveShadows = false;

            var shader = Shader.Find("Universal Render Pipeline/Unlit");
            if (shader == null)
                shader = Shader.Find("Sprites/Default");

            var mat = new Material(shader) { hideFlags = HideFlags.DontSave };
            if (mat.HasProperty("_BaseColor")) mat.SetColor("_BaseColor", color);
            if (mat.HasProperty("_Color")) mat.SetColor("_Color", color);

            line.material = mat;
            line.startColor = color;
            line.endColor = color;
            return line;
        }

        private void OnDestroy()
        {
            if (_ahead != null) Destroy(_ahead.gameObject);
            if (_flown != null) Destroy(_flown.gameObject);
            if (_pip != null) Destroy(_pip.gameObject);
        }
    }
}
