using UnityEngine;
using TMPro;

namespace DroneRescue.Visualization
{
    /// <summary>
    /// A floating TextMeshPro caption that hangs above something in the world and
    /// turns to face the camera.
    ///
    /// Created at runtime and flagged DontSave, so labelling the scene adds nothing
    /// to the scene asset and cannot drift out of step with what is actually there.
    ///
    /// Two details worth the code they cost:
    ///
    /// NOT PARENTED TO THE TARGET. A drone body is scaled to 1.6 and a patient
    /// capsule is scaled unevenly, and a child label would inherit that, so the
    /// captions would come out different sizes for no reason the viewer can see.
    /// The label follows the target's position instead and keeps its own scale.
    ///
    /// CONSTANT APPARENT SIZE. Scale tracks distance to the camera, so a caption
    /// occupies the same slice of the screen whether the camera is on the whole map
    /// or pushed in on one drone. A fixed world size would be unreadable at one and
    /// overwhelming at the other.
    /// </summary>
    [DisallowMultipleComponent]
    public class WorldLabel : MonoBehaviour
    {
        private TextMeshPro _text;
        private Transform _target;
        private Camera _camera;
        private float _height;
        private float _screenScale = 0.010f;

        /// <summary>Hangs a new caption above <paramref name="target"/>.</summary>
        public static WorldLabel Attach(Transform target, Transform parent, string content,
                                        Color color, float heightAboveTarget, float fontSize)
        {
            if (target == null)
                return null;

            // RectTransform is added up front rather than relying on TextMeshPro to
            // supply one: a GameObject built in code carries a plain Transform, and
            // TMP lays its glyphs out against a RectTransform it expects to find.
            var go = new GameObject(target.name + "_Label", typeof(RectTransform));
            go.transform.SetParent(parent, false);
            go.hideFlags = HideFlags.DontSave;

            var label = go.AddComponent<WorldLabel>();
            label.Build(target, content, color, heightAboveTarget, fontSize);
            return label;
        }

        /// <summary>A caption pinned to a fixed point rather than to a moving object.</summary>
        public static WorldLabel AttachToPoint(Vector3 position, Transform parent, string content,
                                               Color color, float heightAboveGround, float fontSize)
        {
            var anchor = new GameObject("LabelAnchor");
            anchor.transform.SetParent(parent, false);
            anchor.transform.position = position;
            anchor.hideFlags = HideFlags.DontSave;

            return Attach(anchor.transform, parent, content, color, heightAboveGround, fontSize);
        }

        private void Build(Transform target, string content, Color color, float heightAboveTarget, float fontSize)
        {
            _target = target;
            _height = heightAboveTarget;
            _camera = Camera.main;

            _text = gameObject.AddComponent<TextMeshPro>();
            _text.fontSize = fontSize;
            _text.alignment = TextAlignmentOptions.Center;
            _text.color = Color.white;
            _text.richText = true;

            // Wide enough that a short caption never wraps, which avoids depending
            // on the word-wrap property whose name has moved around between
            // TextMeshPro versions.
            _text.rectTransform.sizeDelta = new Vector2(40f, 5f);

            SetText(content, color);
            FaceCamera();
        }

        /// <summary>
        /// Sets the caption. The dark plate behind the glyphs is a rich-text mark
        /// rather than a second object: an unbacked caption disappears the moment it
        /// crosses a pale hospital pad or a bright route line.
        /// </summary>
        public void SetText(string content, Color color)
        {
            if (_text == null)
                return;

            _text.text = "<mark=#000000B4><color=#" + FleetPalette.ToHex(color) + "> " + content + " </color></mark>";
        }

        private void LateUpdate()
        {
            // The thing this caption names has gone: so should the caption.
            if (_target == null)
            {
                Destroy(gameObject);
                return;
            }

            if (_camera == null)
            {
                _camera = Camera.main;
                if (_camera == null)
                    return;
            }

            transform.position = _target.position + Vector3.up * _height;
            FaceCamera();
        }

        private void FaceCamera()
        {
            if (_camera == null)
                return;

            // Aligned to the camera's own orientation rather than aimed at it, so
            // every caption on screen shares one upright and they read as a set.
            transform.rotation = _camera.transform.rotation;

            float distance = Vector3.Distance(transform.position, _camera.transform.position);
            transform.localScale = Vector3.one * Mathf.Max(0.05f, distance * _screenScale);
        }
    }
}
