using System.Linq;
using Map;
using NUnit.Framework;

namespace EditModeTests
{
    public class NavigationGraphTests
    {
        // =====================================================================
        //  CellSpace Graph (directed edges from CellBoundary source/target)
        // =====================================================================

        [Test]
        public void BuildCellSpaceGraph_UsesDirectedCellBoundaryEdgesWithoutSyntheticReverseEdge()
        {
            var indoorSpace = TestIndoorSpaceFactory.BuildDirectedCorridor();
            var graph = NavigationGraphBuilder.FromCellSpaces(indoorSpace, 3.0f, 0.05f);

            Assert.IsTrue(graph.HasEdge("c1", "c2"),
                "Edge c1→c2 should exist (cb12: source=c1, target=c2)");
            Assert.IsFalse(graph.HasEdge("c2", "c1"),
                "Reverse edge c2→c1 should NOT exist — CellBoundary is one-way");
            Assert.IsTrue(graph.HasEdge("c2", "c3"),
                "Edge c2→c3 should exist (cb23: source=c2, target=c3)");
            Assert.IsFalse(graph.HasEdge("c3", "c2"),
                "Reverse edge c3→c2 should NOT exist");
        }

        [Test]
        public void BuildCellSpaceGraph_CorrectNodeCount()
        {
            var indoorSpace = TestIndoorSpaceFactory.BuildDirectedCorridor();
            var graph = NavigationGraphBuilder.FromCellSpaces(indoorSpace, 3.0f, 0.05f);

            Assert.AreEqual(3, graph.NodeCount, "Corridor has 3 navigable cells");
        }

        [Test]
        public void BuildCellSpaceGraph_NodesAreCellSpaceKind()
        {
            var indoorSpace = TestIndoorSpaceFactory.BuildDirectedCorridor();
            var graph = NavigationGraphBuilder.FromCellSpaces(indoorSpace, 3.0f, 0.05f);

            var node = graph.GetNode("c1");
            Assert.IsNotNull(node);
            Assert.AreEqual(WaypointKind.CellSpace, node.Kind);
        }

        // =====================================================================
        //  CP Graph (edges from Hypergraph/Container routing)
        // =====================================================================

        [Test]
        public void BuildCpGraph_CreatesNodesFromConnectionPoints()
        {
            var indoorSpace = TestIndoorSpaceFactory.BuildDirectedCorridor();
            var hypergraph = indoorSpace.GetHypergraph();
            var graph = NavigationGraphBuilder.FromHypergraph(hypergraph, indoorSpace, 3.0f, 0.05f);

            Assert.AreEqual(2, graph.NodeCount,
                "Corridor has 2 CellBoundaries → 2 ConnectionPoints");
            Assert.IsNotNull(graph.GetNode("cb12"));
            Assert.IsNotNull(graph.GetNode("cb23"));
        }

        [Test]
        public void BuildCpGraph_NodesAreConnectionPointKind()
        {
            var indoorSpace = TestIndoorSpaceFactory.BuildDirectedCorridor();
            var hypergraph = indoorSpace.GetHypergraph();
            var graph = NavigationGraphBuilder.FromHypergraph(hypergraph, indoorSpace, 3.0f, 0.05f);

            var node = graph.GetNode("cb12");
            Assert.IsNotNull(node);
            Assert.AreEqual(WaypointKind.ConnectionPoint, node.Kind);
        }

        // =====================================================================
        //  Two-floor elevator
        // =====================================================================

        [Test]
        public void BuildCellSpaceGraph_TwoFloor_IncludesVerticalEdge()
        {
            var indoorSpace = TestIndoorSpaceFactory.BuildTwoFloorElevator();
            var graph = NavigationGraphBuilder.FromCellSpaces(indoorSpace, 3.0f, 0.05f);

            Assert.AreEqual(4, graph.NodeCount);
            Assert.IsTrue(graph.HasEdge("EV1", "EV2"),
                "Vertical edge EV1→EV2 should exist from cb_vert");
            Assert.IsTrue(graph.HasEdge("c1F1", "EV1"),
                "Horizontal edge c1F1→EV1 should exist");
            Assert.IsTrue(graph.HasEdge("EV2", "c1F2"),
                "Horizontal edge EV2→c1F2 should exist");
        }

        // =====================================================================
        //  Graph accessors
        // =====================================================================

        [Test]
        public void GetNeighbors_ReturnsOutboundNeighborsOnly()
        {
            var indoorSpace = TestIndoorSpaceFactory.BuildDirectedCorridor();
            var graph = NavigationGraphBuilder.FromCellSpaces(indoorSpace, 3.0f, 0.05f);

            var neighborsOfC1 = graph.GetNeighbors("c1");
            Assert.AreEqual(1, neighborsOfC1.Count);
            Assert.AreEqual("c2", neighborsOfC1[0].Id);

            var neighborsOfC3 = graph.GetNeighbors("c3");
            Assert.AreEqual(0, neighborsOfC3.Count,
                "c3 has no outbound edges — it is only a target");
        }

        [Test]
        public void GetInEdges_ReturnsInboundEdges()
        {
            var indoorSpace = TestIndoorSpaceFactory.BuildDirectedCorridor();
            var graph = NavigationGraphBuilder.FromCellSpaces(indoorSpace, 3.0f, 0.05f);

            var inEdges = graph.GetInEdges("c2");
            Assert.AreEqual(1, inEdges.Count);
            Assert.AreEqual("c1", inEdges[0].FromId);
        }

        [Test]
        public void GetNode_NotFound_ReturnsNull()
        {
            var indoorSpace = TestIndoorSpaceFactory.BuildDirectedCorridor();
            var graph = NavigationGraphBuilder.FromCellSpaces(indoorSpace, 3.0f, 0.05f);

            Assert.IsNull(graph.GetNode("nonexistent"));
        }

        [Test]
        public void EdgeCount_MatchesBoundaryCount()
        {
            var indoorSpace = TestIndoorSpaceFactory.BuildDirectedCorridor();
            var graph = NavigationGraphBuilder.FromCellSpaces(indoorSpace, 3.0f, 0.05f);

            Assert.AreEqual(2, graph.EdgeCount,
                "2 directed boundaries → 2 directed edges");
        }
    }
}
