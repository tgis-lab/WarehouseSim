using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using Map;

namespace PathPlanning
{
    public class BFS
    {
        public static (List<ConnectionPoint> path, List<float> speeds) BFSAlgorithm(
            Graph graph, 
            RoutePoint startPosition, 
            RoutePoint endPosition, 
            float defaultSpeed, 
            SearchMethod searchMethod)
        {
            if (graph == null || startPosition == null || endPosition == null)
            {
                Debug.LogError("Invalid input parameters for BFS algorithm");
                return (new List<ConnectionPoint>(), new List<float>());
            }

            Debug.Log($"Starting BFS algorithm from {startPosition.ConnectionPoint.Id} to {endPosition.ConnectionPoint.Id}");

            HashSet<RoutePoint> visited = new HashSet<RoutePoint>();
            Queue<RoutePoint> queue = new Queue<RoutePoint>();
            Dictionary<RoutePoint, RoutePoint> previous = new Dictionary<RoutePoint, RoutePoint>();
            Dictionary<RoutePoint, float> speedTo = new Dictionary<RoutePoint, float>();

            queue.Enqueue(startPosition);
            visited.Add(startPosition);
            speedTo[startPosition] = defaultSpeed;

            while (queue.Count > 0)
            {
                var current = queue.Dequeue();
                if (current.ConnectionPoint.Id == endPosition.ConnectionPoint.Id)
                {
                    List<ConnectionPoint> path = new List<ConnectionPoint>();
                    List<float> speeds = new List<float>();
                    RoutePoint pathPoint = current;
                    float speed = speedTo[current];

                    while (pathPoint != null)
                    {
                        path.Add(pathPoint.ConnectionPoint);
                        speeds.Add(speed);
                        
                        RoutePoint nextPathPoint;
                        if (previous.TryGetValue(pathPoint, out nextPathPoint))
                        {
                            float nextSpeed;
                            if (speedTo.TryGetValue(nextPathPoint, out nextSpeed))
                            {
                                speed = nextSpeed;
                            }
                            pathPoint = nextPathPoint;
                        }
                        else
                        {
                            pathPoint = null;
                        }
                    }
                    path.Reverse();
                    speeds.Reverse();
                    return (path, speeds);
                }

                foreach (var neighborConnection in current.Children)
                {
                    RoutePoint neighbor = graph.GetRoutePointFormConnectionPoint(neighborConnection);
                    // var neighbor = graph.RoutePoints.FirstOrDefault(rp => rp.ConnectionPoint.Id == neighborConnection.Id);
                    if (neighbor != null && !visited.Contains(neighbor))
                    {
                        float currentSpeed = ModifySpeed(graph, current.ConnectionPoint, neighborConnection, defaultSpeed, speedTo[current]);
                        speedTo[neighbor] = currentSpeed;
                        queue.Enqueue(neighbor);
                        visited.Add(neighbor);
                        previous[neighbor] = current;
                    }
                }
            }
            
            Debug.LogWarning($"No path found between {startPosition.ConnectionPoint.Id} and {endPosition.ConnectionPoint.Id}");
            return (null, null);
        }

        private static (List<ConnectionPoint> path, List<float> speeds) ReconstructPath(
            Dictionary<RoutePoint, RoutePoint> previous, 
            Dictionary<RoutePoint, float> speedTo, 
            RoutePoint current)
        {
            var path = new List<ConnectionPoint>();
            var speeds = new List<float>();

            while (previous.ContainsKey(current))
            {
                path.Add(current.ConnectionPoint);
                speeds.Add(speedTo[current]);
                current = previous[current];
            }

            path.Add(current.ConnectionPoint);
            speeds.Add(speedTo[current]);

            path.Reverse();
            speeds.Reverse();

            return (path, speeds);
        }

        private static float ModifySpeed(Graph graph, ConnectionPoint current, ConnectionPoint neighbor, float defaultSpeed, float lastSpeed)
        {
            return defaultSpeed;
        }
    }
}
