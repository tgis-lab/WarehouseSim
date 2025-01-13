using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace PathPlanning
{
    public class PriorityQueue<TElement, TPriority> where TPriority : IComparable<TPriority>
    {
        private List<(TElement Element, TPriority Priority)> _elements = new List<(TElement, TPriority)>();

        public int Count => _elements.Count;

        public void Enqueue(TElement element, TPriority priority)
        {
            _elements.Add((element, priority));
            int ci = _elements.Count - 1;
            while (ci > 0)
            {
                int pi = (ci - 1) / 2;
                if (_elements[ci].Priority.CompareTo(_elements[pi].Priority) >= 0)
                    break;
                var tmp = _elements[ci];
                _elements[ci] = _elements[pi];
                _elements[pi] = tmp;
                ci = pi;
            }
        }

        public bool TryDequeue(out TElement element, out TPriority priority)
        {
            if (_elements.Count == 0)
            {
                element = default;
                priority = default;
                return false;
            }

            element = _elements[0].Element;
            priority = _elements[0].Priority;

            int li = _elements.Count - 1;
            _elements[0] = _elements[li];
            _elements.RemoveAt(li);

            li--;
            int pi = 0;
            while (true)
            {
                int ci = pi * 2 + 1;
                if (ci > li) break;
                int rc = ci + 1;
                if (rc <= li && _elements[rc].Priority.CompareTo(_elements[ci].Priority) < 0)
                    ci = rc;
                if (_elements[pi].Priority.CompareTo(_elements[ci].Priority) <= 0)
                    break;
                var tmp = _elements[ci];
                _elements[ci] = _elements[pi];
                _elements[pi] = tmp;
                pi = ci;
            }

            return true;
        }
    }
}