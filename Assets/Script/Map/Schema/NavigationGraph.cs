using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace Map
{
    /// <summary>
    /// Unified directed graph with inbound/outbound adjacency and node lookup.
    /// Replaces both Graph (CP-backed) and CellSpaceGraph (centroid-backed).
    /// </summary>
    public sealed class NavigationGraph
    {
        private readonly Dictionary<string, Waypoint> nodes = new Dictionary<string, Waypoint>();
        private readonly Dictionary<string, List<NavigationEdge>> outEdges = new Dictionary<string, List<NavigationEdge>>();
        private readonly Dictionary<string, List<NavigationEdge>> inEdges = new Dictionary<string, List<NavigationEdge>>();

        public void AddNode(Waypoint node)
        {
            nodes[node.Id] = node;
            if (!outEdges.ContainsKey(node.Id))
                outEdges[node.Id] = new List<NavigationEdge>();
            if (!inEdges.ContainsKey(node.Id))
                inEdges[node.Id] = new List<NavigationEdge>();
        }

        public void AddEdge(NavigationEdge edge)
        {
            if (!outEdges.ContainsKey(edge.FromId))
                outEdges[edge.FromId] = new List<NavigationEdge>();
            if (!inEdges.ContainsKey(edge.ToId))
                inEdges[edge.ToId] = new List<NavigationEdge>();

            outEdges[edge.FromId].Add(edge);
            inEdges[edge.ToId].Add(edge);
        }

        /// <summary>
        /// Convenience: add edge from two node IDs, deriving cell metadata from Waypoint properties.
        /// </summary>
        public void AddEdge(string fromId, string toId)
        {
            var from = nodes[fromId];
            var to = nodes[toId];
            string enterCell = from.Kind == WaypointKind.CellSpace ? from.Id : from.SecondaryCellId;
            string exitCell = to.Kind == WaypointKind.CellSpace ? to.Id : to.PrimaryCellId;
            bool isVertical = from.IsVertical || to.IsVertical;
            AddEdge(new NavigationEdge(fromId, toId, enterCell, exitCell, isVertical, null));
        }

        public Waypoint GetNode(string id)
        {
            return nodes.TryGetValue(id, out var n) ? n : null;
        }

        public List<Waypoint> GetNeighbors(string nodeId)
        {
            if (!outEdges.TryGetValue(nodeId, out var edges))
                return new List<Waypoint>();
            return edges.Select(e => nodes[e.ToId]).ToList();
        }

        public List<NavigationEdge> GetOutEdges(string nodeId)
        {
            return outEdges.TryGetValue(nodeId, out var edges) ? edges : new List<NavigationEdge>();
        }

        public List<NavigationEdge> GetInEdges(string nodeId)
        {
            return inEdges.TryGetValue(nodeId, out var edges) ? edges : new List<NavigationEdge>();
        }

        public bool HasEdge(string fromId, string toId)
        {
            return outEdges.TryGetValue(fromId, out var edges) && edges.Any(e => e.ToId == toId);
        }

        public IEnumerable<Waypoint> GetAllNodes()
        {
            return nodes.Values;
        }

        public int NodeCount => nodes.Count;

        public int EdgeCount => outEdges.Values.Sum(e => e.Count);

        /// <summary>
        /// Computes the shortest path distance between two nodes using A* (no occupancy).
        /// Returns -1 if no path exists.
        /// </summary>
        public float ComputeShortestDistance(string startId, string endId)
        {
            if (startId == endId) return 0f;
            if (!nodes.ContainsKey(startId) || !nodes.ContainsKey(endId)) return -1f;

            var endPos = nodes[endId].Position;
            var openSet = new List<string> { startId };
            var closedSet = new HashSet<string>();
            var gScore = new Dictionary<string, float> { { startId, 0f } };
            var fScore = new Dictionary<string, float>
                { { startId, Vector3.Distance(nodes[startId].Position, endPos) } };

            while (openSet.Count > 0)
            {
                // Find node with lowest fScore
                string current = openSet[0];
                float bestF = fScore.TryGetValue(current, out float f0) ? f0 : float.PositiveInfinity;
                for (int i = 1; i < openSet.Count; i++)
                {
                    float fi = fScore.TryGetValue(openSet[i], out float fv) ? fv : float.PositiveInfinity;
                    if (fi < bestF) { bestF = fi; current = openSet[i]; }
                }

                if (current == endId)
                    return gScore[endId];

                openSet.Remove(current);
                closedSet.Add(current);

                if (!outEdges.TryGetValue(current, out var edges)) continue;
                float currentG = gScore[current];

                foreach (var edge in edges)
                {
                    if (closedSet.Contains(edge.ToId)) continue;

                    float edgeCost = Vector3.Distance(nodes[current].Position, nodes[edge.ToId].Position);
                    float tentativeG = currentG + edgeCost;

                    if (!openSet.Contains(edge.ToId))
                        openSet.Add(edge.ToId);
                    else if (tentativeG >= (gScore.TryGetValue(edge.ToId, out float g) ? g : float.PositiveInfinity))
                        continue;

                    gScore[edge.ToId] = tentativeG;
                    fScore[edge.ToId] = tentativeG + Vector3.Distance(nodes[edge.ToId].Position, endPos);
                }
            }

            return -1f; // No path
        }
    }
}
