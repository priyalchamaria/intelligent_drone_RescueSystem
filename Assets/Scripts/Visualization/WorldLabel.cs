using System.Collections.Generic;
using UnityEngine;
using TMPro;

namespace DroneRescue.Visualization
{
    /// <summary>
    /// A floating TextMeshPro caption that hangs above something in the world, turns
    /// to face the camera, and steps aside when another caption is already occupying
    /// its patch of screen.
    ///
    /// Created at runtime and flagged DontSave, so labelling the scene adds nothing
    /// to the scene asset and cannot drift out of step with what is actually there.
    ///
    /// Three details worth the code they cost:
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
    ///
    /// OVERLAP RESOLVED IN SCREEN SPACE, ACROSS ALL CAPTIONS AT ONCE. Drones end a
    /// mission parked together on the hospital landing ring, and casualties cluster
    /// around a hazard, so several captions routinely want the same few pixels and
    /// stack into an unreadable smear. No caption can see that on its own, so the
    /// layout is done once per frame for every live caption together: see LayoutAll.
    /// </summary>
    [DisallowMultipleComponent]
    public class WorldLabel : MonoBehaviour
    {
        /// <summary>
        /// Every live caption. Static because overlap is a property of the set, not
        /// of any one member.
        /// </summary>
        private static readonly List<WorldLabel> Registry = new List<WorldLabel>();

        private static readonly List<WorldLabel> Visible = new List<WorldLabel>();
        private static int _laidOutFrame = -1;

        /// <summary>
        /// Ties are broken in favour of the lower number, which keeps its natural
        /// position while the others move. Set so that the things a viewer is
        /// actually following stay put: casualties still awaiting rescue first,
        /// then drones in the air, then finished work, then scenery.
        /// </summary>
        public int SortPriority = 2;

        private TextMeshPro _text;
        private Transform _target;
        private Camera _camera;
        private float _height;
        private float _fontSize = 20f;
        private float _sizeMultiplier = 1f;
        private float _alpha = 1f;
        private string _content = "";
        private Color _color = Color.white;

        /// <summary>Fraction of the screen's height one font-size unit occupies. Tuning knob for all captions.</summary>
        private const float ScreenScale = 0.010f;

        /// <summary>TextMeshPro world text is roughly this many world units tall per unit of font size.</summary>
        private const float WorldUnitsPerFontSize = 0.1f;

        private Vector3 _anchor;
        private Vector2 _screen;
        private bool _offScreen;
        private float _pixelOffset;

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
            _fontSize = fontSize;
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
        }

        private void OnEnable()
        {
            if (!Registry.Contains(this))
                Registry.Add(this);
        }

        private void OnDisable()
        {
            Registry.Remove(this);
        }

        /// <summary>
        /// Sets the caption. The dark plate behind the glyphs is a rich-text mark
        /// rather than a second object: an unbacked caption disappears the moment it
        /// crosses a pale hospital pad or a bright route line.
        /// </summary>
        public void SetText(string content, Color color)
        {
            _content = content ?? "";
            _color = color;
            Repaint();
        }

        /// <summary>
        /// Knocks a caption back without hiding it. Used for drones parked at base
        /// and for the site names, which are exactly the captions that pile up and
        /// the ones a viewer least needs to read mid-run.
        /// </summary>
        public void SetEmphasis(float sizeMultiplier, float alpha)
        {
            if (Mathf.Approximately(_sizeMultiplier, sizeMultiplier) && Mathf.Approximately(_alpha, alpha))
                return;

            _sizeMultiplier = Mathf.Max(0.1f, sizeMultiplier);
            _alpha = Mathf.Clamp01(alpha);
            Repaint();
        }

        private void Repaint()
        {
            if (_text == null)
                return;

            byte a = (byte)Mathf.RoundToInt(_alpha * 255f);
            byte plate = (byte)Mathf.RoundToInt(_alpha * 180f);

            _text.text = "<mark=#000000" + plate.ToString("X2") + ">"
                       + "<color=#" + FleetPalette.ToHex(_color) + a.ToString("X2") + "> "
                       + _content + " </color></mark>";
        }

        private void LateUpdate()
        {
            if (_target == null)
            {
                // The thing this caption names has gone: so should the caption.
                Destroy(gameObject);
                return;
            }

            // Layout is a whole-set operation, so whichever caption reaches
            // LateUpdate first does it for everybody and the rest fall through.
            if (_laidOutFrame == Time.frameCount)
                return;

            _laidOutFrame = Time.frameCount;
            LayoutAll();
        }

        /// <summary>
        /// Places every live caption for this frame, then pushes apart the ones that
        /// would land on top of each other.
        ///
        /// The nudge is upward in screen space only. Moving a caption sideways, or
        /// in the world, would break the one thing it has to do: sit unambiguously
        /// above the object it names.
        /// </summary>
        private static void LayoutAll()
        {
            var camera = Camera.main;
            if (camera == null)
                return;

            Visible.Clear();

            for (int i = Registry.Count - 1; i >= 0; i--)
            {
                var label = Registry[i];
                if (label == null || label._target == null)
                {
                    Registry.RemoveAt(i);
                    continue;
                }

                label._camera = camera;
                label._anchor = label._target.position + Vector3.up * label._height;
                label._pixelOffset = 0f;

                var point = camera.WorldToScreenPoint(label._anchor);
                label._offScreen = point.z <= 0f;
                label._screen = new Vector2(point.x, point.y);

                if (!label._offScreen)
                    Visible.Add(label);
            }

            // Highest on screen first within a priority band, so a column of
            // captions resolves upward in a stable order instead of shuffling.
            Visible.Sort(Compare);

            for (int i = 0; i < Visible.Count; i++)
            {
                var label = Visible[i];

                // Each nudge can push this caption into a third one, so keep going
                // until it is clear. The guard caps a pathological pile-up rather
                // than expecting one.
                for (int pass = 0; pass < 24; pass++)
                {
                    bool moved = false;

                    for (int j = 0; j < i; j++)
                    {
                        float clearance = label.Overlap(Visible[j]);
                        if (clearance <= 0f)
                            continue;

                        label._pixelOffset += clearance + 1f;
                        moved = true;
                        break;
                    }

                    if (!moved)
                        break;
                }
            }

            for (int i = 0; i < Registry.Count; i++)
                Registry[i].Apply(camera);
        }

        private static int Compare(WorldLabel a, WorldLabel b)
        {
            if (a.SortPriority != b.SortPriority)
                return a.SortPriority.CompareTo(b.SortPriority);

            return b._screen.y.CompareTo(a._screen.y);
        }

        /// <summary>
        /// How many pixels this caption must rise to clear <paramref name="other"/>,
        /// or zero when the two do not touch.
        /// </summary>
        private float Overlap(WorldLabel other)
        {
            float halfWidths = (PixelWidth() + other.PixelWidth()) * 0.5f;
            if (Mathf.Abs(_screen.x - other._screen.x) >= halfWidths)
                return 0f;

            float halfHeights = (PixelHeight() + other.PixelHeight()) * 0.5f;
            float gap = Mathf.Abs(CurrentY(this) - CurrentY(other));

            return gap >= halfHeights ? 0f : halfHeights - gap;
        }

        private static float CurrentY(WorldLabel label) => label._screen.y + label._pixelOffset;

        /// <summary>
        /// On-screen height of one line of this caption, in pixels.
        ///
        /// Derived rather than measured. Apparent size is pinned to a fraction of
        /// the screen's height by design, so the camera distance cancels out and the
        /// result depends only on the font size, that fraction, and the field of
        /// view. Measuring the rendered mesh instead would cost a layout rebuild per
        /// caption per frame for a number that does not change.
        /// </summary>
        private float PixelHeight()
        {
            if (_camera == null)
                return 12f;

            float halfFov = Mathf.Tan(_camera.fieldOfView * 0.5f * Mathf.Deg2Rad);
            if (halfFov <= 0.0001f)
                return 12f;

            return _fontSize * _sizeMultiplier * WorldUnitsPerFontSize * ScreenScale
                   * Screen.height / (2f * halfFov);
        }

        /// <summary>Rough on-screen width. The two extra characters are the plate's padding spaces.</summary>
        private float PixelWidth() => PixelHeight() * 0.55f * (_content.Length + 2);

        private void Apply(Camera camera)
        {
            if (_text == null)
                return;

            _text.enabled = !_offScreen;
            if (_offScreen)
                return;

            float distance = Vector3.Distance(_anchor, camera.transform.position);

            // Aligned to the camera's own orientation rather than aimed at it, so
            // every caption on screen shares one upright and they read as a set.
            transform.rotation = camera.transform.rotation;
            transform.localScale = Vector3.one * Mathf.Max(0.02f, distance * ScreenScale * _sizeMultiplier);

            transform.position = _anchor + camera.transform.up * (_pixelOffset * WorldPerPixel(camera, distance));
        }

        private static float WorldPerPixel(Camera camera, float distance)
        {
            if (camera.orthographic)
                return camera.orthographicSize * 2f / Mathf.Max(1, Screen.height);

            float halfFov = Mathf.Tan(camera.fieldOfView * 0.5f * Mathf.Deg2Rad);
            return 2f * distance * halfFov / Mathf.Max(1, Screen.height);
        }
    }
}
