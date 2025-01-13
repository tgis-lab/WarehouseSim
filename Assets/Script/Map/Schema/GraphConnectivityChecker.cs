using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace Map {
    public class GraphConnectivityChecker
    {
        private Graph graph;

        public GraphConnectivityChecker(Graph graph)
        {
            this.graph = graph;
        }

        public bool ValidateGraphConnectivity()
        {
            if (graph.RoutePoints.Count == 0)
            {
                Debug.LogError("Graph is empty");
                return false;
            }

            HashSet<RoutePoint> visited = new HashSet<RoutePoint>();
            Queue<RoutePoint> queue = new Queue<RoutePoint>();
            Dictionary<string, int> inDegree = new Dictionary<string, int>();
            HashSet<string> reportedEdges = new HashSet<string>();

            foreach (var point in graph.RoutePoints)
            {
                inDegree[point.ConnectionPoint.Id] = 0;
            }

            foreach (var point in graph.RoutePoints)
            {
                foreach (var child in point.Children)
                {
                    var childPoint = graph.GetRoutePointFormConnectionPoint(child);
                    if (childPoint != null)
                    {
                        inDegree[childPoint.ConnectionPoint.Id]++;
                    }
                }
            }

            RoutePoint startPoint = graph.RoutePoints[0];
            queue.Enqueue(startPoint);
            visited.Add(startPoint);

            while (queue.Count > 0)
            {
                var current = queue.Dequeue();
                foreach (var child in current.Children)
                {
                    var childRoutePoint = graph.GetRoutePointFormConnectionPoint(child);
                    if (childRoutePoint != null)
                    {
                        string edgeKey = $"{current.ConnectionPoint.Id}->{childRoutePoint.ConnectionPoint.Id}";
                        if (!visited.Contains(childRoutePoint))
                        {
                            visited.Add(childRoutePoint);
                            queue.Enqueue(childRoutePoint);
                            // Debug.Log($"Visiting new point: {childRoutePoint.ConnectionPoint.Id}");
                        }
                        else if (!reportedEdges.Contains(edgeKey))
                        {
                            // Debug.Log($"Revisiting point: {childRoutePoint.ConnectionPoint.Id} from {current.ConnectionPoint.Id}");
                            reportedEdges.Add(edgeKey);
                        }
                    }
                    else
                    {
                        Debug.LogError($"Invalid connection: {current.ConnectionPoint.Id} -> null");
                    }
                }
            }

            bool isFullyConnected = visited.Count == graph.RoutePoints.Count;
            Debug.Log($"Graph connectivity check: {(isFullyConnected ? "Fully connected" : "Disconnected")}. " +
                    $"Visited: {visited.Count}, Total: {graph.RoutePoints.Count}");

            var unreachedPoints = graph.RoutePoints.Where(rp => !visited.Contains(rp)).ToList();
            Debug.Log($"Unreached points: {unreachedPoints.Count}");
            foreach (var point in unreachedPoints)
            {
                Debug.Log($"Unreached point: {point.ConnectionPoint.Id}, " +
                        $"Position: {point.ConnectionPoint.Point}, " +
                        $"In-degree: {inDegree[point.ConnectionPoint.Id]}");
            }

            var zeroInDegreePoints = graph.RoutePoints.Where(rp => inDegree[rp.ConnectionPoint.Id] == 0).ToList();
            Debug.Log($"Points with zero in-degree: {zeroInDegreePoints.Count}");
            foreach (var point in zeroInDegreePoints)
            {
                Debug.Log($"Zero in-degree point: {point.ConnectionPoint.Id}, Position: {point.ConnectionPoint.Point}");
            }

            // Debug.Log($"Total edges reported: {reportedEdges.Count}");
            // Debug.Log($"Average in-degree: {inDegree.Values.Average():F2}");

            return isFullyConnected;
        }
    }
}