using System.Collections.Generic;
using DroneRescue.Environment;

namespace DroneRescue.Planning
{
    /// <summary>
    /// The Mission Planner's patient priority queue, from Part 1 Step 1.
    ///
    /// Ordering, most significant first:
    ///   1. triage priority   — Critical before Serious before Stable
    ///   2. time of detection — older patients before newer ones of equal priority
    ///   3. patient id        — a stable tiebreak so a run is reproducible
    ///
    /// The second rule matters: without it two Critical patients would come out in
    /// whatever order the heap happened to leave them, and a patient detected early
    /// could sit behind one detected much later. That would quietly wreck the
    /// Average Response Time metric in Part 3.
    ///
    /// This is a binary min-heap rather than a sorted list because Phase 6 pushes
    /// new emergencies into a queue that is already being drained, and a heap keeps
    /// both push and pop cheap.
    /// </summary>
    public class PatientQueue
    {
        private readonly List<Patient> _heap = new List<Patient>();

        public int Count => _heap.Count;
        public bool IsEmpty => _heap.Count == 0;

        /// <summary>Everything still queued, in heap order. For inspection only, not for dispatch.</summary>
        public IReadOnlyList<Patient> Items => _heap;

        public void Clear() => _heap.Clear();

        /// <summary>
        /// Adds a patient to the queue. Phase 6's NewEmergency event calls exactly
        /// this, rather than having a queueing path of its own.
        /// </summary>
        public void Push(Patient patient)
        {
            if (patient == null)
                return;

            _heap.Add(patient);
            SiftUp(_heap.Count - 1);
        }

        /// <summary>The next patient to be served, without removing them.</summary>
        public Patient Peek() => _heap.Count == 0 ? null : _heap[0];

        /// <summary>Removes and returns the highest-priority patient. Null when empty.</summary>
        public Patient Pop()
        {
            if (_heap.Count == 0)
                return null;

            var top = _heap[0];
            int last = _heap.Count - 1;
            _heap[0] = _heap[last];
            _heap.RemoveAt(last);

            if (_heap.Count > 0)
                SiftDown(0);

            return top;
        }

        /// <summary>
        /// Removes a specific patient wherever they sit in the queue. Needed when a
        /// patient is served or withdrawn by something other than a Pop.
        /// </summary>
        public bool Remove(Patient patient)
        {
            int index = _heap.IndexOf(patient);
            if (index < 0)
                return false;

            int last = _heap.Count - 1;
            _heap[index] = _heap[last];
            _heap.RemoveAt(last);

            if (index < _heap.Count)
            {
                SiftUp(index);
                SiftDown(index);
            }

            return true;
        }

        public bool Contains(Patient patient) => _heap.Contains(patient);

        /// <summary>
        /// The queue drained into a list, highest priority first. Non-destructive:
        /// used for logging and, later, for the dashboard's queue panel.
        /// </summary>
        public List<Patient> ToOrderedList()
        {
            var copy = new PatientQueue();
            for (int i = 0; i < _heap.Count; i++)
                copy.Push(_heap[i]);

            var ordered = new List<Patient>(copy.Count);
            while (!copy.IsEmpty)
                ordered.Add(copy.Pop());

            return ordered;
        }

        /// <summary>Negative when a comes first.</summary>
        private static int Compare(Patient a, Patient b)
        {
            // PatientPriority is numbered so that lower means more urgent.
            int byPriority = ((int)a.priority).CompareTo((int)b.priority);
            if (byPriority != 0)
                return byPriority;

            int byTime = a.queuedAtTime.CompareTo(b.queuedAtTime);
            if (byTime != 0)
                return byTime;

            return string.CompareOrdinal(a.id, b.id);
        }

        private void SiftUp(int index)
        {
            while (index > 0)
            {
                int parent = (index - 1) / 2;
                if (Compare(_heap[index], _heap[parent]) >= 0)
                    return;

                Swap(index, parent);
                index = parent;
            }
        }

        private void SiftDown(int index)
        {
            while (true)
            {
                int left = index * 2 + 1;
                int right = left + 1;
                int smallest = index;

                if (left < _heap.Count && Compare(_heap[left], _heap[smallest]) < 0)
                    smallest = left;
                if (right < _heap.Count && Compare(_heap[right], _heap[smallest]) < 0)
                    smallest = right;

                if (smallest == index)
                    return;

                Swap(index, smallest);
                index = smallest;
            }
        }

        private void Swap(int a, int b)
        {
            var tmp = _heap[a];
            _heap[a] = _heap[b];
            _heap[b] = tmp;
        }
    }
}
