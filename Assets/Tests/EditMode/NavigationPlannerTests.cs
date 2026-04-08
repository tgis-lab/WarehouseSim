using System.Linq;
using Map;
using NUnit.Framework;
using PathPlanning;

namespace EditModeTests
{
    public class NavigationPlannerTests
    {
        // =====================================================================
        //  BasicAStarPathFinder
        // =====================================================================

        /// <summary>
        /// Builds a CP graph from the corridor and manually wires edges.
        /// The test fixture has no Containers/RLineGroups, so FromHypergraph() creates nodes only.
        /// We add edges explicitly to test path-finding logic.
        /// </summary>
        private NavigationGraph BuildWiredCpGraph(IndoorSpace indoorSpace)
        {
            var hypergraph = indoorSpace.GetHypergraph();
            var graph = NavigationGraphBuilder.FromHypergraph(hypergraph, indoorSpace, 3.0f, 0.05f);
            // Manually wire edges: cb12 → cb23 (same as what Container routing would produce)
            if (graph.GetNode("cb12") != null && graph.GetNode("cb23") != null)
                graph.AddEdge("cb12", "cb23");
            return graph;
        }

        [Test]
        public void BasicAStar_ReturnsConnectionPointWaypoints()
        {
            var indoorSpace = TestIndoorSpaceFactory.BuildDirectedCorridor();
            var graph = BuildWiredCpGraph(indoorSpace);

            var start = graph.GetNode("cb12");
            var goal = graph.GetNode("cb23");

            var finder = new BasicAStarPathFinder(SearchMethod.Euclidean_Distance);
            NavigationPathResult result = finder.FindPath(
                graph, start, goal, null, null, "R1", null, 0f, 1f, 2f, 5f);

            Assert.IsNotNull(result);
            Assert.IsNotNull(result.Path);
            Assert.AreEqual(2, result.Path.Count,
                "Path from cb12→cb23 should have 2 waypoints");
            Assert.AreEqual("cb12", result.Path[0].Id);
            Assert.AreEqual("cb23", result.Path[1].Id);
        }

        [Test]
        public void BasicAStar_GeneratesTimesAndSpeeds()
        {
            var indoorSpace = TestIndoorSpaceFactory.BuildDirectedCorridor();
            var graph = BuildWiredCpGraph(indoorSpace);

            var start = graph.GetNode("cb12");
            var goal = graph.GetNode("cb23");

            var finder = new BasicAStarPathFinder(SearchMethod.Euclidean_Distance);
            NavigationPathResult result = finder.FindPath(
                graph, start, goal, null, null, "R1", null, 10f, 1f, 2f, 5f);

            Assert.AreEqual(result.Path.Count, result.Times.Count);
            Assert.AreEqual(result.Path.Count, result.Speeds.Count);
            Assert.AreEqual(10f, result.Times[0], 0.01f,
                "First time should equal startTime");
            Assert.IsTrue(result.Times[1] > result.Times[0],
                "Times should be monotonically increasing");
        }

        [Test]
        public void BasicAStar_StartEqualsGoal_ReturnsSingleWaypoint()
        {
            var indoorSpace = TestIndoorSpaceFactory.BuildDirectedCorridor();
            var graph = BuildWiredCpGraph(indoorSpace);

            var start = graph.GetNode("cb12");

            var finder = new BasicAStarPathFinder(SearchMethod.Euclidean_Distance);
            NavigationPathResult result = finder.FindPath(
                graph, start, start, null, null, "R1", null, 0f, 1f, 2f, 5f);

            Assert.IsNotNull(result);
            Assert.AreEqual(1, result.Path.Count);
            Assert.AreEqual("cb12", result.Path[0].Id);
        }

        // =====================================================================
        //  BfsPathFinder
        // =====================================================================

        [Test]
        public void Bfs_FindsPathInDirectedGraph()
        {
            var indoorSpace = TestIndoorSpaceFactory.BuildDirectedCorridor();
            var graph = BuildWiredCpGraph(indoorSpace);

            var start = graph.GetNode("cb12");
            var goal = graph.GetNode("cb23");

            var finder = new BfsPathFinder();
            NavigationPathResult result = finder.FindPath(
                graph, start, goal, null, null, "R1", null, 0f, 1f, 2f, 5f);

            Assert.IsNotNull(result);
            Assert.AreEqual(2, result.Path.Count);
            Assert.AreEqual("cb12", result.Path[0].Id);
            Assert.AreEqual("cb23", result.Path[1].Id);
        }

        // =====================================================================
        //  CellSpaceAStarPathFinder
        // =====================================================================

        [Test]
        public void CellSpaceAStar_ReturnsCentroidWaypointsWithoutConnectionPointRemap()
        {
            var indoorSpace = TestIndoorSpaceFactory.BuildDirectedCorridor();
            var graph = NavigationGraphBuilder.FromCellSpaces(indoorSpace, 3.0f, 0.05f);

            var start = graph.GetNode("c1");
            var goal = graph.GetNode("c3");

            var finder = new CellSpaceAStarPathFinder();
            NavigationPathResult result = finder.FindPath(
                graph, start, goal, null, null, "R1", null, 0f, 1f, 2f, 5f);

            Assert.IsNotNull(result);
            CollectionAssert.AreEqual(
                new[] { "c1", "c2", "c3" },
                result.Path.Select(w => w.Id).ToArray(),
                "CellSpace A* should return centroid waypoints directly");

            // All nodes should be CellSpace kind
            Assert.IsTrue(result.Path.All(w => w.Kind == WaypointKind.CellSpace));
        }

        [Test]
        public void CellSpaceAStar_CrossFloor_FindsPathThroughElevator()
        {
            var indoorSpace = TestIndoorSpaceFactory.BuildTwoFloorElevator();
            var graph = NavigationGraphBuilder.FromCellSpaces(indoorSpace, 3.0f, 0.05f);

            var start = graph.GetNode("c1F1");
            var goal = graph.GetNode("c1F2");

            var finder = new CellSpaceAStarPathFinder();
            NavigationPathResult result = finder.FindPath(
                graph, start, goal, null, null, "R1", null, 0f, 1f, 2f, 5f);

            Assert.IsNotNull(result);
            Assert.IsTrue(result.Path.Count >= 4,
                "Cross-floor path should include cells on both floors plus elevator nodes");
            Assert.AreEqual("c1F1", result.Path.First().Id);
            Assert.AreEqual("c1F2", result.Path.Last().Id);
            Assert.IsTrue(result.Path.Any(w => w.Id == "EV1"));
            Assert.IsTrue(result.Path.Any(w => w.Id == "EV2"));
        }

        // =====================================================================
        //  TimeSpaceAStarPathFinder (adapter)
        // =====================================================================

        [Test]
        public void TimeSpaceAStar_ImplementsIPathFinder()
        {
            var indoorSpace = TestIndoorSpaceFactory.BuildDirectedCorridor();

            var finder = new TimeSpaceAStarPathFinder(indoorSpace, SearchMethod.Euclidean_Distance);

            Assert.IsInstanceOf<IPathFinder>(finder);
        }

        [Test]
        public void TimeSpaceAStar_SameStartAndGoal_ReturnsSingleWaypoint()
        {
            var indoorSpace = TestIndoorSpaceFactory.BuildDirectedCorridor();
            var navGraph = BuildWiredCpGraph(indoorSpace);
            var start = navGraph.GetNode("cb12");

            var finder = new TimeSpaceAStarPathFinder(indoorSpace, SearchMethod.Euclidean_Distance);
            var result = finder.FindPath(navGraph, start, start, null, null, "R1", null, 0f, 1f, 2f, 5f);

            Assert.IsNotNull(result);
            Assert.AreEqual(1, result.Path.Count);
            Assert.AreEqual("cb12", result.Path[0].Id);
            Assert.AreEqual(WaypointKind.ConnectionPoint, result.Path[0].Kind);
        }

        // =====================================================================
        //  AnytimeSippPathFinder (adapter)
        // =====================================================================

        [Test]
        public void AnytimeSipp_ImplementsIPathFinder()
        {
            var indoorSpace = TestIndoorSpaceFactory.BuildDirectedCorridor();

            var finder = new AnytimeSippPathFinder(indoorSpace, SearchMethod.Euclidean_Distance);

            Assert.IsInstanceOf<IPathFinder>(finder);
        }

        [Test]
        public void AnytimeSipp_SameStartAndGoal_ReturnsSingleWaypoint()
        {
            var indoorSpace = TestIndoorSpaceFactory.BuildDirectedCorridor();
            var navGraph = BuildWiredCpGraph(indoorSpace);
            var start = navGraph.GetNode("cb12");

            var finder = new AnytimeSippPathFinder(indoorSpace, SearchMethod.Euclidean_Distance);
            var result = finder.FindPath(navGraph, start, start, null, null, "R1", null, 0f, 1f, 2f, 5f);

            Assert.IsNotNull(result);
            Assert.AreEqual(1, result.Path.Count);
            Assert.AreEqual("cb12", result.Path[0].Id);
        }

        // =====================================================================
        //  Empty graph / null input edge cases
        // =====================================================================

        [Test]
        public void BasicAStar_EmptyGraph_ReturnsNull()
        {
            var graph = new NavigationGraph();
            var finder = new BasicAStarPathFinder(SearchMethod.Euclidean_Distance);
            var result = finder.FindPath(graph, null, null, null, null, "R1", null, 0f, 1f, 2f, 5f);
            Assert.IsNull(result);
        }

        [Test]
        public void CellSpaceAStar_NullGraph_ReturnsNull()
        {
            var finder = new CellSpaceAStarPathFinder();
            var result = finder.FindPath(null, null, null, null, null, "R1", null, 0f, 1f, 2f, 5f);
            Assert.IsNull(result);
        }

        // =====================================================================
        //  NavigationPathResult
        // =====================================================================

        [Test]
        public void NavigationPathResult_HasPlanIdAndMetrics()
        {
            var indoorSpace = TestIndoorSpaceFactory.BuildDirectedCorridor();
            var graph = BuildWiredCpGraph(indoorSpace);

            var finder = new BasicAStarPathFinder(SearchMethod.Euclidean_Distance);
            NavigationPathResult result = finder.FindPath(
                graph, graph.GetNode("cb12"), graph.GetNode("cb23"),
                null, null, "R1", null, 0f, 1f, 2f, 5f);

            Assert.IsNotNull(result.PlanId);
            Assert.IsTrue(result.PlanId.Contains("R1"));
            Assert.IsNotNull(result.Metrics);
            Assert.IsTrue(result.Metrics.ActualPathLength > 0);
        }
    }
}
