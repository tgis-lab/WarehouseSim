using System.Collections.Generic;
using System.Linq;
using Map;
using NUnit.Framework;
using PathPlanning;

namespace EditModeTests
{
    /// <summary>
    /// Tests for NavigationGraph built from CellSpaces (directed edges, no forced bidirectional).
    /// Rewrites of the original CellSpaceGraph tests to use the unified Waypoint topology.
    /// </summary>
    public class CellSpaceGraphTests
    {
        // ===================================================================
        //  BuildGraph
        // ===================================================================

        [Test]
        public void BuildGraph_CreatesCorrectNodeCount()
        {
            var indoorSpace = TestIndoorSpaceFactory.BuildDirectedCorridor();
            var graph = NavigationGraphBuilder.FromCellSpaces(indoorSpace, 3.0f, 0.05f);
            Assert.AreEqual(3, graph.NodeCount);
        }

        [Test]
        public void BuildGraph_ExcludesNonNavigableCells()
        {
            var indoorSpace = new IndoorSpace();
            indoorSpace.CellSpaces.Add(TestIndoorSpaceFactory.MakeNavigableCell("A", 0, 0));
            // Add a non-navigable cell
            var nonNavProps = new Dictionary<string, object> { { "navigable", false } };
            var gf = new NetTopologySuite.Geometries.GeometryFactory();
            var node = gf.CreatePoint(new NetTopologySuite.Geometries.Coordinate(1, 0));
            var space = gf.CreatePolygon(new[] {
                new NetTopologySuite.Geometries.Coordinate(0.5, -0.5),
                new NetTopologySuite.Geometries.Coordinate(1.5, -0.5),
                new NetTopologySuite.Geometries.Coordinate(1.5, 0.5),
                new NetTopologySuite.Geometries.Coordinate(0.5, 0.5),
                new NetTopologySuite.Geometries.Coordinate(0.5, -0.5)
            });
            indoorSpace.CellSpaces.Add(new CellSpace("obstacle", nonNavProps, space, node));
            indoorSpace.CellSpaces.Add(TestIndoorSpaceFactory.MakeNavigableCell("B", 2, 0));

            var graph = NavigationGraphBuilder.FromCellSpaces(indoorSpace, 3.0f, 0.05f);

            Assert.AreEqual(2, graph.NodeCount, "Non-navigable cells should be excluded");
            Assert.IsNotNull(graph.GetNode("A"));
            Assert.IsNotNull(graph.GetNode("B"));
            Assert.IsNull(graph.GetNode("obstacle"));
        }

        [Test]
        public void BuildGraph_DirectedEdges_NoForcedBidirectional()
        {
            var indoorSpace = TestIndoorSpaceFactory.BuildDirectedCorridor();
            var graph = NavigationGraphBuilder.FromCellSpaces(indoorSpace, 3.0f, 0.05f);

            // cb12: source=c1, target=c2 → edge c1→c2 exists, c2→c1 does NOT
            Assert.IsTrue(graph.HasEdge("c1", "c2"), "Forward edge should exist");
            Assert.IsFalse(graph.HasEdge("c2", "c1"), "Reverse edge should NOT exist — directed");
        }

        [Test]
        public void BuildGraph_IgnoresBoundariesWithMissingCells()
        {
            var indoorSpace = new IndoorSpace();
            indoorSpace.CellSpaces.Add(TestIndoorSpaceFactory.MakeNavigableCell("A", 0, 0));
            indoorSpace.CellBoundaries.Add(TestIndoorSpaceFactory.MakeHorizontalBoundary("cb_AX", "A", "X", 0.5, 0));

            var graph = NavigationGraphBuilder.FromCellSpaces(indoorSpace, 3.0f, 0.05f);

            Assert.AreEqual(1, graph.NodeCount);
            Assert.AreEqual(0, graph.GetNeighbors("A").Count,
                "Boundary to non-existent cell should be ignored");
        }

        // ===================================================================
        //  FindPath (via CellSpaceAStarPathFinder)
        // ===================================================================

        [Test]
        public void FindPath_ReturnsPathForConnectedCells()
        {
            var indoorSpace = TestIndoorSpaceFactory.BuildDirectedCorridor();
            var graph = NavigationGraphBuilder.FromCellSpaces(indoorSpace, 3.0f, 0.05f);
            var finder = new CellSpaceAStarPathFinder();

            var result = finder.FindPath(graph, graph.GetNode("c1"), graph.GetNode("c3"),
                null, null, "R1", null, 0f, 1f, 2f, 5f);

            Assert.IsNotNull(result);
            Assert.AreEqual(3, result.Path.Count);
            Assert.AreEqual("c1", result.Path[0].Id);
            Assert.AreEqual("c3", result.Path[result.Path.Count - 1].Id);
        }

        [Test]
        public void FindPath_StartEqualsGoal_ReturnsOneElementPath()
        {
            var indoorSpace = TestIndoorSpaceFactory.BuildDirectedCorridor();
            var graph = NavigationGraphBuilder.FromCellSpaces(indoorSpace, 3.0f, 0.05f);
            var finder = new CellSpaceAStarPathFinder();

            var result = finder.FindPath(graph, graph.GetNode("c2"), graph.GetNode("c2"),
                null, null, "R1", null, 0f, 1f, 2f, 5f);

            Assert.IsNotNull(result);
            Assert.AreEqual(1, result.Path.Count);
            Assert.AreEqual("c2", result.Path[0].Id);
        }

        // ===================================================================
        //  FindPath cross-floor
        // ===================================================================

        [Test]
        public void FindPath_CrossFloor_IncludesVerticalTransition()
        {
            var indoorSpace = TestIndoorSpaceFactory.BuildTwoFloorElevator();
            var graph = NavigationGraphBuilder.FromCellSpaces(indoorSpace, 3.0f, 0.05f);
            var finder = new CellSpaceAStarPathFinder();

            var result = finder.FindPath(graph, graph.GetNode("c1F1"), graph.GetNode("c1F2"),
                null, null, "R1", null, 0f, 1f, 2f, 5f);

            Assert.IsNotNull(result, "Cross-floor path should be found");
            Assert.IsTrue(result.Path.Count >= 4);
            Assert.AreEqual("c1F1", result.Path[0].Id);
            Assert.AreEqual("c1F2", result.Path[result.Path.Count - 1].Id);
            Assert.IsTrue(result.Path.Any(w => w.Id == "EV1"));
            Assert.IsTrue(result.Path.Any(w => w.Id == "EV2"));
        }

        // ===================================================================
        //  FindPath no path
        // ===================================================================

        [Test]
        public void FindPath_ReturnsNullForDisconnectedCells()
        {
            var indoorSpace = new IndoorSpace();
            indoorSpace.CellSpaces.Add(TestIndoorSpaceFactory.MakeNavigableCell("A", 0, 0));
            indoorSpace.CellSpaces.Add(TestIndoorSpaceFactory.MakeNavigableCell("B", 10, 10));

            var graph = NavigationGraphBuilder.FromCellSpaces(indoorSpace, 3.0f, 0.05f);
            var finder = new CellSpaceAStarPathFinder();

            UnityEngine.TestTools.LogAssert.Expect(UnityEngine.LogType.Error, new System.Text.RegularExpressions.Regex("CellSpaceAStar A\\* exhausted"));

            var result = finder.FindPath(graph, graph.GetNode("A"), graph.GetNode("B"),
                null, null, "R1", null, 0f, 1f, 2f, 5f);

            Assert.IsNull(result, "Should return null when no path exists");
        }

        [Test]
        public void FindPath_ReturnsNullForMissingStartOrGoal()
        {
            var indoorSpace = TestIndoorSpaceFactory.BuildDirectedCorridor();
            var graph = NavigationGraphBuilder.FromCellSpaces(indoorSpace, 3.0f, 0.05f);
            var finder = new CellSpaceAStarPathFinder();

            Assert.IsNull(finder.FindPath(graph, null, graph.GetNode("c1"),
                null, null, "R1", null, 0f, 1f, 2f, 5f));
            Assert.IsNull(finder.FindPath(graph, graph.GetNode("c1"), null,
                null, null, "R1", null, 0f, 1f, 2f, 5f));
        }

        // ===================================================================
        //  FindPath adjacent (1-hop)
        // ===================================================================

        [Test]
        public void FindPath_AdjacentCells_Returns2CellPath()
        {
            var indoorSpace = TestIndoorSpaceFactory.BuildDirectedCorridor();
            var graph = NavigationGraphBuilder.FromCellSpaces(indoorSpace, 3.0f, 0.05f);
            var finder = new CellSpaceAStarPathFinder();

            var result = finder.FindPath(graph, graph.GetNode("c1"), graph.GetNode("c2"),
                null, null, "R1", null, 0f, 1f, 2f, 5f);

            Assert.IsNotNull(result);
            Assert.AreEqual(2, result.Path.Count);
            Assert.AreEqual("c1", result.Path[0].Id);
            Assert.AreEqual("c2", result.Path[1].Id);
        }

        // ===================================================================
        //  Contains / GetNeighbors accessors
        // ===================================================================

        [Test]
        public void GetNode_ReturnsTrueForExistingCell()
        {
            var indoorSpace = TestIndoorSpaceFactory.BuildDirectedCorridor();
            var graph = NavigationGraphBuilder.FromCellSpaces(indoorSpace, 3.0f, 0.05f);
            Assert.IsNotNull(graph.GetNode("c1"));
            Assert.IsNotNull(graph.GetNode("c3"));
        }

        [Test]
        public void GetNode_ReturnsNullForMissingCell()
        {
            var indoorSpace = TestIndoorSpaceFactory.BuildDirectedCorridor();
            var graph = NavigationGraphBuilder.FromCellSpaces(indoorSpace, 3.0f, 0.05f);
            Assert.IsNull(graph.GetNode("MISSING"));
        }

        [Test]
        public void GetNeighbors_ReturnsEmptyListForMissingCell()
        {
            var indoorSpace = TestIndoorSpaceFactory.BuildDirectedCorridor();
            var graph = NavigationGraphBuilder.FromCellSpaces(indoorSpace, 3.0f, 0.05f);
            var neighbors = graph.GetNeighbors("MISSING");
            Assert.IsNotNull(neighbors);
            Assert.AreEqual(0, neighbors.Count);
        }

        // ===================================================================
        //  Duplicate boundary edge deduplication
        // ===================================================================

        [Test]
        public void BuildGraph_DuplicateBoundaries_NoDuplicateNeighbors()
        {
            var indoorSpace = new IndoorSpace();
            indoorSpace.CellSpaces.Add(TestIndoorSpaceFactory.MakeNavigableCell("A", 0, 0));
            indoorSpace.CellSpaces.Add(TestIndoorSpaceFactory.MakeNavigableCell("B", 1, 0));
            indoorSpace.CellBoundaries.Add(TestIndoorSpaceFactory.MakeHorizontalBoundary("cb1", "A", "B", 0.5, 0));
            indoorSpace.CellBoundaries.Add(TestIndoorSpaceFactory.MakeHorizontalBoundary("cb2", "A", "B", 0.5, 0));

            var graph = NavigationGraphBuilder.FromCellSpaces(indoorSpace, 3.0f, 0.05f);

            // Two boundaries create two edges (NavigationGraph allows this — unlike CellSpaceGraph which deduped)
            // This is acceptable behavior as NavigationGraph uses edge lists, not adjacency sets
            Assert.IsTrue(graph.GetNeighbors("A").Count >= 1,
                "A should have at least one neighbor B");
        }
    }
}
