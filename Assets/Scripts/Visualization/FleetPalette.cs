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
        /// Chosen to stay apart from each other AND from the patient and pad
        /// colours below, so a route crossing a casualty is still readable.
        /// </summary>
        private static readonly Color[] RouteColors =
        {
            new Color(0.20f, 0.90f, 1.00f), // cyan
            new Color(1.00f, 0.38f, 0.85f), // magenta
            new Color(1.00f, 0.88f, 0.25f), // yellow
            new Color(0.40f, 1.00f, 0.55f), // spring green
            new Color(0.66f, 0.52f, 1.00f), // violet
            new Color(1.00f, 0.58f, 0.36f), // coral
        };

        public static readonly Color Critical = new Color(0.86f, 0.14f, 0.14f);
        public static readonly Color Serious  = new Color(0.95f, 0.64f, 0.10f);
        public static readonly Color Stable   = new Color(0.24f, 0.74f, 0.32f);
        public static readonly Color Hospital = new Color(0.92f, 0.95f, 1.00f);
        public static readonly Color Charging = new Color(0.13f, 0.72f, 0.55f);
        public static readonly Color Fire     = new Color(0.90f, 0.33f, 0.10f);
        public static readonly Color Rescued  = new Color(0.62f, 0.66f, 0.62f);

        public static int RouteColorCount => RouteColors.Length;

        public static Color RouteColor(int index)
        {
            if (RouteColors.Length == 0)
                return Color.white;

            int wrapped = ((index % RouteColors.Length) + RouteColors.Length) % RouteColors.Length;
            return RouteColors[wrapped];
        }

        /// <summary>
        /// The colour for a drone id such as "D-03".
        ///
        /// Keyed off the trailing number so the mapping is stable no matter what
        /// order the drones registered in: D-03 is the same colour on every run and
        /// in every screenshot. Ids with no trailing number fall back to a hash,
        /// which is stable for that id but arbitrary.
        /// </summary>
        public static Color RouteColor(string droneId)
        {
            if (string.IsNullOrEmpty(droneId))
                return Color.white;

            int digitsEnd = droneId.Length;
            int digitsStart = digitsEnd;
            while (digitsStart > 0 && char.IsDigit(droneId[digitsStart - 1]))
                digitsStart--;

            if (digitsStart < digitsEnd
                && int.TryParse(droneId.Substring(digitsStart, digitsEnd - digitsStart), out int number))
            {
                return RouteColor(number - 1);
            }

            return RouteColor(Mathf.Abs(droneId.GetHashCode()));
        }

        public static Color ForPriority(PatientPriority priority)
        {
            if (priority == PatientPriority.Critical) return Critical;
            if (priority == PatientPriority.Serious) return Serious;
            return Stable;
        }

        /// <summary>Same hue, knocked back. Used for the part of a route already flown.</summary>
        public static Color Dimmed(Color color, float brightness, float alpha)
        {
            return new Color(color.r * brightness, color.g * brightness, color.b * brightness, alpha);
        }

        /// <summary>The colour as "RRGGBB", for TextMeshPro rich text tags.</summary>
        public static string ToHex(Color color) => ColorUtility.ToHtmlStringRGB(color);
    }
}
