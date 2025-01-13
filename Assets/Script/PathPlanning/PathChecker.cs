using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using Map;

namespace PathPlanning
{
    public class PathChecker
    {
        private Graph graph;

        public PathChecker(Graph graph)
        {
            this.graph = graph;
        }

        public bool ValidatePath(string startId, string endId)
        {
            var start = graph.RoutePoints.FirstOrDefault(rp => rp.ConnectionPoint.Id == startId);
            var end = graph.RoutePoints.FirstOrDefault(rp => rp.ConnectionPoint.Id == endId);

            if (start == null || end == null)
            {
                Debug.LogError($"Start point {startId} or end point {endId} not found in graph.");
                return false;
            }

            Debug.Log($"Start point: {startId}, Position: {start.ConnectionPoint.Point}");
            Debug.Log($"End point: {endId}, Position: {end.ConnectionPoint.Point}");

            HashSet<RoutePoint> visited = new HashSet<RoutePoint>();
            Queue<RoutePoint> queue = new Queue<RoutePoint>();
            Dictionary<RoutePoint, RoutePoint> previous = new Dictionary<RoutePoint, RoutePoint>();

            queue.Enqueue(start);
            visited.Add(start);

            while (queue.Count > 0)
            {
                var current = queue.Dequeue();
                if (current.ConnectionPoint.Id == endId)
                {
                    List<string> path = new List<string>();
                    RoutePoint pathPoint = current;
                    while (pathPoint != null)
                    {
                        path.Add(pathPoint.ConnectionPoint.Id);
                        previous.TryGetValue(pathPoint, out pathPoint);
                    }
                    path.Reverse();
                    Debug.Log($"Path found: {string.Join(" -> ", path)}");
                    return true;
                }

                // Debug.Log($"Checking connections from {current.ConnectionPoint.Id}:");
                foreach (var child in current.Children)
                {
                    var childRoutePoint = graph.RoutePoints.FirstOrDefault(rp => rp.ConnectionPoint.Id == child.Id);
                    if (childRoutePoint != null && !visited.Contains(childRoutePoint))
                    {
                        visited.Add(childRoutePoint);
                        queue.Enqueue(childRoutePoint);
                        previous[childRoutePoint] = current;
                    }
                }
            }

            Debug.LogWarning($"No path found between {startId} and {endId}");
            return false;
        }

        public void AnalyzeFailedPath(string startId, string endId)
        {
            var start = graph.RoutePoints.FirstOrDefault(rp => rp.ConnectionPoint.Id == startId);
            var end = graph.RoutePoints.FirstOrDefault(rp => rp.ConnectionPoint.Id == endId);

            if (start == null || end == null)
            {
                Debug.LogError($"Start point {startId} or end point {endId} not found in graph.");
                return;
            }

            Debug.Log($"Analyzing failed path from {startId} to {endId}");
            Debug.Log($"Start point: {startId}, Position: {start.ConnectionPoint.Point}");
            Debug.Log($"End point: {endId}, Position: {end.ConnectionPoint.Point}");

            Debug.Log("Outgoing connections from start point:");
            foreach (var child in start.Children)
            {
                Debug.Log($"  -> {child.Id}");
            }

            Debug.Log("Incoming connections to end point:");
            var incomingConnections = graph.RoutePoints
                .Where(rp => rp.Children.Any(c => c.Id == endId))
                .Select(rp => rp.ConnectionPoint.Id);
            foreach (var incoming in incomingConnections)
            {
                Debug.Log($"  {incoming} ->");
            }
        }
    }
}