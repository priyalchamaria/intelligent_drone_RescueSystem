using System;
using System.IO;
using System.Text;
using UnityEngine;

namespace DroneRescue.Metrics
{
    /// <summary>
    /// All the file writing for a mission run, in one place.
    ///
    /// PORTED FROM STAGE 1's WriteSummaryCSV, and generalised. Stage 1 opened one
    /// StreamWriter at the end of its single run, wrote a header and exactly one
    /// data row into a filename stamped with the time, and wrapped the lot in a
    /// try/catch that logged rather than threw. That last part is the piece worth
    /// keeping verbatim: a locked or unwritable file must never take the simulation
    /// down with it, because the run itself is still valid whether or not it was
    /// recorded.
    ///
    /// What changed for Stage 2, and why:
    ///
    ///   ONE FILE, MANY RUNS. Stage 1 measured one hard-coded flight, so a file per
    ///   run was the same thing as a file per result. Stage 2 runs a scenario over
    ///   and over, and Phase 9 has to put a scored run next to a nearest-idle run,
    ///   so the summary CSV is opened for APPEND under a stable name and the header
    ///   is written only when the file is new. Comparing runs then means opening one
    ///   file rather than collating a directory.
    ///
    ///   THE EVENT FEED IS STREAMED, NOT BUFFERED. It is written as events happen,
    ///   so a run that is stopped halfway still leaves a readable trace of how far
    ///   it got. A run that ends normally would be fine either way; a run that
    ///   ends because something went wrong is exactly when the feed is worth having.
    ///
    ///   OUTPUT LANDS BESIDE THE PROJECT IN THE EDITOR. Stage 1 wrote to
    ///   Application.persistentDataPath, which on Windows is buried under AppData
    ///   and is awkward to find, hand in, or point a browser at. In the editor the
    ///   files go to a MissionLogs folder next to Assets; a built player still uses
    ///   persistentDataPath, because next to Assets does not exist there.
    /// </summary>
    public class MissionLogWriter
    {
        public const string SummaryFileName = "mission_runs.csv";

        /// <summary>
        /// UTF-8 with no byte-order mark.
        ///
        /// The framework default emits one, and a leading mark turns the first CSV
        /// column heading into something no plain parser matches on. Phase 8's
        /// dashboard reads these files, so the default would have cost a bug there
        /// for no benefit here: the content is ASCII apart from the odd separator.
        /// </summary>
        private static readonly Encoding Utf8NoBom = new UTF8Encoding(false);

        private readonly string _directory;
        private readonly string _runStamp;
        private StreamWriter _eventStream;
        private bool _eventStreamFailed;

        public string Directory => _directory;
        public string SummaryPath => Path.Combine(_directory, SummaryFileName);
        public string EventLogPath => Path.Combine(_directory, "run_" + _runStamp + "_events.log");
        public string PatientCsvPath => Path.Combine(_directory, "run_" + _runStamp + "_patients.csv");

        public MissionLogWriter(string directory, string runStamp)
        {
            _directory = string.IsNullOrEmpty(directory) ? DefaultDirectory() : directory;
            _runStamp = runStamp;
        }

        public static string DefaultDirectory()
        {
#if UNITY_EDITOR
            var projectRoot = System.IO.Directory.GetParent(Application.dataPath);
            if (projectRoot != null)
                return Path.Combine(projectRoot.FullName, "MissionLogs");
#endif
            return Path.Combine(Application.persistentDataPath, "MissionLogs");
        }

        private bool EnsureDirectory()
        {
            try
            {
                if (!System.IO.Directory.Exists(_directory))
                    System.IO.Directory.CreateDirectory(_directory);
                return true;
            }
            catch (Exception ex)
            {
                Debug.LogError("[Metrics] Cannot create log directory " + _directory + ": " + ex.Message);
                return false;
            }
        }

        // -----------------------------------------------------------------
        // Human-readable event feed
        // -----------------------------------------------------------------

        /// <summary>
        /// Appends one line to the run's event feed, opening the file on first use.
        ///
        /// A failure disables the stream rather than being reported per line: an
        /// unwritable path fails on every single event, and a run would otherwise
        /// bury the console under hundreds of identical errors.
        /// </summary>
        public void AppendEvent(string line)
        {
            if (_eventStreamFailed)
                return;

            try
            {
                if (_eventStream == null)
                {
                    if (!EnsureDirectory())
                    {
                        _eventStreamFailed = true;
                        return;
                    }

                    // Opened for APPEND, and the banner written only when the file
                    // is new. A late event after the feed was closed, which a
                    // reopened run produces, then extends the file instead of
                    // truncating everything already recorded.
                    bool isNewFile = !File.Exists(EventLogPath) || new FileInfo(EventLogPath).Length == 0;

                    _eventStream = new StreamWriter(EventLogPath, true, Utf8NoBom) { AutoFlush = true };

                    if (isNewFile)
                    {
                        _eventStream.WriteLine("# Mission event feed  ·  " + DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss"));
                        _eventStream.WriteLine("# Simulated time, then what happened.");
                        _eventStream.WriteLine();
                    }
                }

                _eventStream.WriteLine(line);
            }
            catch (Exception ex)
            {
                _eventStreamFailed = true;
                Debug.LogError("[Metrics] Event feed disabled: " + ex.Message);
            }
        }

        public void CloseEventFeed()
        {
            try
            {
                if (_eventStream != null)
                {
                    _eventStream.Flush();
                    _eventStream.Dispose();
                }
            }
            catch (Exception ex)
            {
                Debug.LogError("[Metrics] Error closing event feed: " + ex.Message);
            }
            finally
            {
                _eventStream = null;
            }
        }

        // -----------------------------------------------------------------
        // Summary CSV, one row per run
        // -----------------------------------------------------------------

        /// <summary>Appends this run's row, writing the header first if the file is new.</summary>
        public bool AppendSummaryRow(MissionMetrics metrics)
        {
            if (metrics == null || !EnsureDirectory())
                return false;

            try
            {
                RetireMismatchedSummary();

                bool isNewFile = !File.Exists(SummaryPath) || new FileInfo(SummaryPath).Length == 0;

                using (var writer = new StreamWriter(SummaryPath, true, Utf8NoBom))
                {
                    if (isNewFile)
                        writer.WriteLine(MissionMetrics.CsvHeader());

                    writer.WriteLine(metrics.ToCsvRow());
                }

                return true;
            }
            catch (Exception ex)
            {
                Debug.LogError("[Metrics] Error writing " + SummaryPath + ": " + ex.Message);
                return false;
            }
        }

        /// <summary>
        /// Moves an existing summary file aside when its header no longer matches the
        /// columns being written.
        ///
        /// Appending a wider row under a narrower header does not fail; it produces a
        /// file where a column heading and the values under it are different
        /// quantities, and nothing complains until somebody reads a battery figure as
        /// a collision count. Renaming the old file keeps both sets of runs and lets
        /// the new one start from a header that describes it.
        /// </summary>
        private void RetireMismatchedSummary()
        {
            try
            {
                if (!File.Exists(SummaryPath) || new FileInfo(SummaryPath).Length == 0)
                    return;

                string existingHeader;
                using (var reader = new StreamReader(SummaryPath, Utf8NoBom))
                    existingHeader = reader.ReadLine();

                if (existingHeader == MissionMetrics.CsvHeader())
                    return;

                string retired = Path.Combine(_directory,
                    Path.GetFileNameWithoutExtension(SummaryFileName)
                    + "_before_" + DateTime.Now.ToString("yyyyMMdd_HHmmss") + ".csv");

                File.Move(SummaryPath, retired);
                Debug.Log("[Metrics] Summary columns changed. Earlier runs kept as " + retired);
            }
            catch (Exception ex)
            {
                Debug.LogWarning("[Metrics] Could not retire the old summary file: " + ex.Message);
            }
        }

        // -----------------------------------------------------------------
        // Per-casualty detail
        // -----------------------------------------------------------------

        /// <summary>
        /// Writes the per-casualty rows the run summary is averaged from.
        ///
        /// Not required by Part 3, and worth the twenty lines anyway: an average
        /// response time with nothing behind it cannot be checked, and Phase 8's
        /// analytics view needs the individual numbers to draw anything other than
        /// a single bar.
        /// </summary>
        public bool WritePatientRows(string header, System.Collections.Generic.IEnumerable<string> rows)
        {
            if (rows == null || !EnsureDirectory())
                return false;

            try
            {
                using (var writer = new StreamWriter(PatientCsvPath, false, Utf8NoBom))
                {
                    writer.WriteLine(header);
                    foreach (var row in rows)
                        writer.WriteLine(row);
                }

                return true;
            }
            catch (Exception ex)
            {
                Debug.LogError("[Metrics] Error writing " + PatientCsvPath + ": " + ex.Message);
                return false;
            }
        }
    }
}
