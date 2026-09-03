using System.Collections.Generic;
using UnityEngine;

namespace DroneRescue.Fleet
{
    /// <summary>
    /// Draws a drone's planned route as a line, so the flown path can be compared
    /// against the planned one by eye.
    ///
    /// Purely a debug aid. The line object and its material are created at runtime
    /// and never saved into the scene, so nothing here touches the navigation
    /// logic or the scene asset.
    /// </summary>
    [RequireComponent(typeof(DroneAgent))]
    public class RouteVisualizer : MonoBehaviour
    {
        [SerializeField] private bool showRoute = true;
        [SerializeField] private Color routeColor = new Color(0.2f, 0.9f, 1f, 1f);
        [SerializeField] private float lineWidth = 0.35f;

        [Tooltip("Height above the ground plane to draw the line at.")]
        [SerializeField] private float lineHeight = 0.6f;

        private DroneAgent _agent;
        private LineRenderer _line;
        private int _lastRouteVersion = -1;

        private void Awake()
        {
            _agent = GetComponent<DroneAgent>();
        }

        private void LateUpdate()
        {
            if (!showRoute || _agent.Follower == null)
                return;

            var route = _agent.Follower.Route;
            if (route == null || route.Count < 2)
            {
                if (_line != null)
                    _line.enabled = false;
                return;
            }

            EnsureLine();
            _line.enabled = true;

            // Redraw only when the route actually changes, not every frame.
            int version = route.Count * 397 ^ _agent.Follower.WaypointCount;
            if (version == _lastRouteVersion)
                return;

            _lastRouteVersion = version;
            _line.positionCount = route.Count;
            for (int i = 0; i < route.Count; i++)
                _line.SetPosition(i, new Vector3(route[i].x, lineHeight, route[i].z));
        }

        private void EnsureLine()
        {
            if (_line != null)
                return;

            var go = new GameObject(name + "_Route");
            go.transform.SetParent(transform.parent, false);
            go.hideFlags = HideFlags.DontSave;

            _line = go.AddComponent<LineRenderer>();
            _line.useWorldSpace = true;
            _line.widthMultiplier = lineWidth;
            _line.numCornerVertices = 2;
            _line.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
            _line.receiveShadows = false;

            var shader = Shader.Find("Universal Render Pipeline/Unlit");
            if (shader == null)
                shader = Shader.Find("Sprites/Default");

            var mat = new Material(shader) { hideFlags = HideFlags.DontSave };
            if (mat.HasProperty("_BaseColor")) mat.SetColor("_BaseColor", routeColor);
            if (mat.HasProperty("_Color")) mat.SetColor("_Color", routeColor);

            _line.material = mat;
            _line.startColor = routeColor;
            _line.endColor = routeColor;
        }

        private void OnDestroy()
        {
            if (_line != null)
                Destroy(_line.gameObject);
        }
    }
}
