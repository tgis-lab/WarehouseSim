using System.Linq;

namespace Map
{
    /// <summary>
    /// Builds NavigationGraph from either Hypergraph (CP graph) or IndoorSpace (CellSpace graph).
    /// </summary>
    public static class NavigationGraphBuilder
    {
        /// <summary>
        /// Builds a CP-backed NavigationGraph from the Hypergraph.
        /// Nodes are ConnectionPoint waypoints; edges come from Container/RLineGroup InnerEdges.
        /// </summary>
        public static NavigationGraph FromHypergraph(Hypergraph hypergraph, IndoorSpace indoorSpace,
            float floorHeight, float baseHeight)
        {
            var graph = new NavigationGraph();

            foreach (var cp in hypergraph.HyperNodes)
                graph.AddNode(Waypoint.FromConnectionPoint(cp, indoorSpace, floorHeight, baseHeight));

            foreach (var group in hypergraph.HyperEdges)
            {
                foreach (var edge in group.InnerEdges)
                {
                    if (edge.Count < 2) continue;
                    string sourceId = edge[0];
                    string targetId = edge[1];
                    if (graph.GetNode(sourceId) != null && graph.GetNode(targetId) != null)
                        graph.AddEdge(sourceId, targetId);
                }
            }

            return graph;
        }

        /// <summary>
        /// Builds a CellSpace-backed NavigationGraph from IndoorSpace.
        /// Nodes are CellSpace centroids; edges follow CellBoundary source→target direction.
        /// No synthetic reverse edges — CellBoundary direction is authoritative.
        /// </summary>
        public static NavigationGraph FromCellSpaces(IndoorSpace indoorSpace,
            float floorHeight, float baseHeight)
        {
            var graph = new NavigationGraph();

            foreach (var cellSpace in indoorSpace.CellSpaces.Where(cs => cs.IsNavigable()))
                graph.AddNode(Waypoint.FromCellSpace(cellSpace, floorHeight, baseHeight));

            foreach (var boundary in indoorSpace.CellBoundaries)
            {
                var sourceCell = indoorSpace.GetCellSpaceFromId(boundary.Source);
                var targetCell = indoorSpace.GetCellSpaceFromId(boundary.Target);
                if (sourceCell != null && sourceCell.IsNavigable() &&
                    targetCell != null && targetCell.IsNavigable())
                {
                    graph.AddEdge(boundary.Source, boundary.Target);
                }
            }

            return graph;
        }
    }
}
