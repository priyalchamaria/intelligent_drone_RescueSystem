namespace DroneRescue.Visualization
{
    /// <summary>The four Part 1 dynamic events, and nothing else, get a banner.</summary>
    public enum AlertKind
    {
        FireSpread = 0,
        BatteryLow = 1,
        DroneFailed = 2,
        NewEmergency = 3
    }

    /// <summary>
    /// A one-way notice board from the simulation to the screen.
    ///
    /// WHY THIS EXISTS: the on-screen banner and the corner log have to know when
    /// something happened, and the alternative was to have the heads-up display
    /// scrape Unity's console text. Parsing log strings would break silently the
    /// first time a message was reworded, so the callers announce instead.
    ///
    /// STRICTLY ONE WAY, AND STRICTLY OPTIONAL. Nothing here returns a value,
    /// nothing here is awaited, and every call is a no-op when no display is
    /// listening. A raise cannot change what the Mission Planner decides or what
    /// the Navigation Engine plans, so a run with the display switched off behaves
    /// identically to one with it on.
    ///
    /// This is NOT the drone communication channel. Drones do not read it, and
    /// nothing that flows through it reaches another drone.
    /// </summary>
    public static class RescueFeed
    {
        /// <summary>Raised for the four dynamic events. Drives the large banner.</summary>
        public static event System.Action<AlertKind, string> Alert;

        /// <summary>Raised for routine progress. Drives the small corner log.</summary>
        public static event System.Action<string> Note;

        public static void RaiseAlert(AlertKind kind, string message)
        {
            var handler = Alert;
            if (handler != null)
                handler(kind, message);
        }

        public static void RaiseNote(string message)
        {
            var handler = Note;
            if (handler != null)
                handler(message);
        }
    }
}
