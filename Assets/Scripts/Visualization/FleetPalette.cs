using UnityEngine;
using DroneRescue.Environment;

namespace DroneRescue.Visualization
{
    /// <summary>
    /// The one place that decides what colour anything is on screen.
    ///
    /// PURELY PRESENTATIONAL. Nothing here is read by the planner, the navigation
    /// engine or the simulator, and nothing here changes a decision. It exists so
    /// that a route line, the label above a drone and the corner legend cannot
    /// disagree about what colour that drone is, which is the usual way a legend
    /// stops being trustworthy.
    ///
    /// The site and patient colours mirror the scene materials in Assets/Materials
    /// so the legend describes the objects the viewer is actually looking at.
    /// </summary>
    public static class FleetPalette
    {
        /// <summary>
        /// One hue per drone. Six entries because the scenario flies six drones;
        /// beyond that it wraps, and two drones would share a colour.
        ///
        /// Every one of these is deliberately light. The ground material is a dark
        /// grey-green, roughly 0.35 on each channel, and a route line is only a few
        /// pixels wide at the camera height this scene is demonstrated from, so a
        /// colour that merely differs in hue from the ground vanishes into it. What
        /// separates a line from the ground at that size is brightness, not hue, so
        /// each of these keeps at least one channel near full and none of them is
        /// allowed to go dark.
        ///
        /// They are also spread around the wheel far enough to stay apart from each
        /// other, and clear of the patient and pad colours below, so a route
        /// crossing a casualty is still readable.
        /// </summary>
        private static readonly Color[] RouteColors =
        {
            new Color(0.15f, 0.95f, 1.00f), // cyan
            new Color(1.00f, 0.32f, 0.85f), // magenta
            new Color(1.00f, 0.92f, 0.22f), // yellow
            new Color(0.32f, 1.00f, 0.45f), // green
            new Color(0.64f, 0.64f, 1.00f), // periwinkle
            new Color(1.00f, 0.48f, 0.45f), // salmon
        };

        public static readonly Color Critical = new Color(0.86f, 0.14f, 0.14f);
        public static readonly Color Serious  = new Color(0.95f, 0.64f, 0.10f);
        public static readonly Color Stable   = new Color(0.24f, 0.74f, 0.32f);
        public static readonly Color Hospital = new Color(0.92f, 0.95f, 1.00f);
        public static readonly Color Charging = new Color(0.13f, 0.72f, 0.55f);
        public static readonly Color Fire     = new Color(0.90f, 0.33f, 0.10f);
        public static readonly Color Rescued  = new Color(0.62f, 0.66f, 0.62f);

        public static int RouteColorCount => RouteColors.Length;

        public static Color RouteColor(int index) => RouteColors.Length == 0 ? Color.white : RouteColors[Wrap(index)];

        private static int Wrap(int index) =>
            ((index % RouteColors.Length) + RouteColors.Length) % RouteColors.Length;

        /// <summary>
        /// The colour for a drone id such as "D-03".
        ///
        /// Keyed off the trailing number so the mapping is stable no matter what
        /// order the drones registered in: D-03 is the same colour on every run and
        /// in every screenshot.
        /// </summary>
        public static Color RouteColor(string droneId) => RouteColor(RouteIndex(droneId));

        /// <summary>
        /// This drone's slot in the palette, from the trailing number in its id.
        ///
        /// Also used as its drawing layer, so that a drone's colour and the height
        /// its route is drawn at come from the same number and cannot disagree. An
        /// id with no trailing number falls back to a hash, which is stable for that
        /// id but arbitrary.
        /// </summary>
        public static int RouteIndex(string droneId)
        {
            if (string.IsNullOrEmpty(droneId))
                return 0;

            int digitsEnd = droneId.Length;
            int digitsStart = digitsEnd;
            while (digitsStart > 0 && char.IsDigit(droneId[digitsStart - 1]))
                digitsStart--;

            if (digitsStart < digitsEnd
                && int.TryParse(droneId.Substring(digitsStart, digitsEnd - digitsStart), out int number))
            {
                return Wrap(number - 1);
            }

            return Wrap(Mathf.Abs(droneId.GetHashCode()));
        }

        public static Color ForPriority(PatientPriority priority)
        {
            if (priority == PatientPriority.Critical) return Critical;
            if (priority == PatientPriority.Serious) return Serious;
            return Stable;
        }

        /// <summary>The scene's ground material, so a receding colour has something to recede INTO.</summary>
        public static readonly Color Ground = new Color(0.34f, 0.37f, 0.33f);

        /// <summary>
        /// The same hue, pushed back toward the ground it is drawn on.
        ///
        /// Used for the part of a route already flown. Simply darkening the colour
        /// was the obvious thing and it was wrong: a dark line on a mid-grey ground
        /// has MORE contrast than a bright one, not less, so the flown trail came
        /// out as a black smear that drew the eye and no longer read as belonging to
        /// any particular drone. Blending toward the ground colour instead keeps the
        /// hue legible and genuinely lowers the contrast.
        /// </summary>
        public static Color Receded(Color color, float towardGround)
        {
            return Color.Lerp(color, Ground, Mathf.Clamp01(towardGround));
        }

        /// <summary>The colour as "RRGGBB", for TextMeshPro rich text tags.</summary>
        public static string ToHex(Color color) => ColorUtility.ToHtmlStringRGB(color);
    }
}
