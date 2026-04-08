using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using Map;

namespace PathPlanning
{
    public class PathChecker
    {
        private NavigationGraph graph;

        public PathChecker(NavigationGraph graph)
        {
            this.graph = graph;
        }

        public bool ValidatePath(string startId, string endId)
        {
            var start = graph.GetNode(startId);
            var end = graph.GetNode(endId);

            if (start == null || end == null)
            {
                Debug.LogError($"Start point {startId} or end point {endId} not found in graph.");
                return false;
            }

            Debug.Log($"Start point: {startId}, Position: {start.Position}");
            Debug.Log($"End point: {endId}, Position: {end.Position}");

            HashSet<string> visited = new HashSet<string>();
            Queue<string> queue = new Queue<string>();
            Dictionary<string, string> previous = new Dictionary<string, string>();

            queue.Enqueue(start.Id);
            visited.Add(start.Id);

            while (queue.Count > 0)
            {
                var currentId = queue.Dequeue();
                if (currentId == endId)
                {
                    List<string> path = new List<string>();
                    string pathId = currentId;
                    while (pathId != null)
                    {
                        path.Add(pathId);
                        previous.TryGetValue(pathId, out pathId);
                    }
                    path.Reverse();
                    Debug.Log($"Path found: {string.Join(" -> ", path)}");
                    return true;
                }

                foreach (var neighbor in graph.GetNeighbors(currentId))
                {
                    if (!visited.Contains(neighbor.Id))
                    {
                        visited.Add(neighbor.Id);
                        queue.Enqueue(neighbor.Id);
                        previous[neighbor.Id] = currentId;
                    }
                }
            }

            Debug.LogWarning($"No path found between {startId} and {endId}");
            return false;
        }

        public void AnalyzeFailedPath(string startId, string endId)
        {
            var start = graph.GetNode(startId);
            var end = graph.GetNode(endId);

            if (start == null || end == null)
            {
                Debug.LogError($"Start point {startId} or end point {endId} not found in graph.");
                return;
            }

            Debug.Log($"Analyzing failed path from {startId} to {endId}");
            Debug.Log($"Start point: {startId}, Position: {start.Position}");
            Debug.Log($"End point: {endId}, Position: {end.Position}");

            Debug.Log("Outgoing connections from start point:");
            foreach (var neighbor in graph.GetNeighbors(startId))
            {
                Debug.Log($"  -> {neighbor.Id}");
            }

            Debug.Log("Incoming connections to end point:");
            var incomingConnections = graph.GetInEdges(endId)
                .Select(e => e.FromId);
            foreach (var incoming in incomingConnections)
            {
                Debug.Log($"  {incoming} ->");
            }
        }
    }
}
