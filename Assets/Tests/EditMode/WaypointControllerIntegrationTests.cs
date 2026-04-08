using System.Collections.Generic;
using Map;
using NUnit.Framework;
using PathPlanning;
using CentralControl;

namespace EditModeTests
{
    /// <summary>
    /// Integration tests verifying the Waypoint type flows through the full pipeline:
    /// EventManager event args, PathPlanner contracts, and occupancy recording.
    /// RobotController is a MonoBehaviour and cannot be fully tested in EditMode,
    /// so we test the contracts it depends on.
    /// </summary>
    public class WaypointControllerIntegrationTests
    {
        [Test]
        public void EventManager_PathEventArgs_UseWaypointType()
        {
            var indoorSpace = TestIndoorSpaceFactory.BuildDirectedCorridor();
            var hypergraph = indoorSpace.GetHypergraph();
            var graph = NavigationGraphBuilder.FromHypergraph(hypergraph, indoorSpace, 3.0f, 0.05f);
            graph.AddEdge("cb12", "cb23");

            var path = new List<Waypoint> { graph.GetNode("cb12"), graph.GetNode("cb23") };
            var times = new List<float> { 0f, 2f };
            var speeds = new List<float> { 1f, 1f };

            var args = new RobotPathEventArgs("R1", "plan-1", path, times, speeds);

            Assert.AreEqual(2, args.Path.Count);
            Assert.AreEqual("cb12", args.Path[0].Id);
            Assert.AreEqual(WaypointKind.ConnectionPoint, args.Path[0].Kind);
        }

        [Test]
        public void PathPlanner_NewConstructor_AcceptsNavigationGraphs()
        {
            var indoorSpace = TestIndoorSpaceFactory.BuildDirectedCorridor();
            var hypergraph = indoorSpace.GetHypergraph();
            var cpGraph = NavigationGraphBuilder.FromHypergraph(hypergraph, indoorSpace, 3.0f, 0.05f);
            var cellGraph = NavigationGraphBuilder.FromCellSpaces(indoorSpace, 3.0f, 0.05f);
            var cpQueries = new NavigationQueryService(indoorSpace, cpGraph, 3.0f, 0.05f);
            var cellQueries = new NavigationQueryService(indoorSpace, cellGraph, 3.0f, 0.05f);

            var planner = new PathPlanner(
                cpGraph, cellGraph, cpQueries, cellQueries,
                indoorSpace, SearchAlgorithm.Astar_Basic, SearchMethod.Euclidean_Distance,
                "R1", 2f, 5f);

            Assert.IsNotNull(planner);
        }

        [Test]
        public void PathPlanner_FindPath_ReturnsNavigationPathResult()
        {
            var indoorSpace = TestIndoorSpaceFactory.BuildDirectedCorridor();
            var hypergraph = indoorSpace.GetHypergraph();
            var cpGraph = NavigationGraphBuilder.FromHypergraph(hypergraph, indoorSpace, 3.0f, 0.05f);
            cpGraph.AddEdge("cb12", "cb23");
            var cellGraph = NavigationGraphBuilder.FromCellSpaces(indoorSpace, 3.0f, 0.05f);
            var cpQueries = new NavigationQueryService(indoorSpace, cpGraph, 3.0f, 0.05f);
            var cellQueries = new NavigationQueryService(indoorSpace, cellGraph, 3.0f, 0.05f);

            var planner = new PathPlanner(
                cpGraph, cellGraph, cpQueries, cellQueries,
                indoorSpace, SearchAlgorithm.Astar_Basic, SearchMethod.Euclidean_Distance,
                "R1", 2f, 5f);

            var start = cpGraph.GetNode("cb12");
            var goal = cpGraph.GetNode("cb23");
            NavigationPathResult result = planner.FindPathWithDynamicObstacles(
                null, null, start, goal, "R1", null, 0f, 1f);

            Assert.IsNotNull(result);
            Assert.AreEqual(2, result.Path.Count);
            Assert.AreEqual("cb12", result.Path[0].Id);
            Assert.AreEqual("cb23", result.Path[1].Id);
        }

        [Test]
        public void OccupancyLayer_RecordPath_AcceptsWaypointFromPlannerResult()
        {
            var indoorSpace = TestIndoorSpaceFactory.BuildDirectedCorridor();
            var hypergraph = indoorSpace.GetHypergraph();
            var cpGraph = NavigationGraphBuilder.FromHypergraph(hypergraph, indoorSpace, 3.0f, 0.05f);
            cpGraph.AddEdge("cb12", "cb23");
            var cellGraph = NavigationGraphBuilder.FromCellSpaces(indoorSpace, 3.0f, 0.05f);
            var cpQueries = new NavigationQueryService(indoorSpace, cpGraph, 3.0f, 0.05f);
            var cellQueries = new NavigationQueryService(indoorSpace, cellGraph, 3.0f, 0.05f);

            var planner = new PathPlanner(
                cpGraph, cellGraph, cpQueries, cellQueries,
                indoorSpace, SearchAlgorithm.Astar_Basic, SearchMethod.Euclidean_Distance,
                "R1", 2f, 5f);

            var start = cpGraph.GetNode("cb12");
            var goal = cpGraph.GetNode("cb23");
            NavigationPathResult result = planner.FindPathWithDynamicObstacles(
                null, null, start, goal, "R1", null, 0f, 1f);

            // Feed planner result directly into occupancy layer
            var layer = new DynamicOccupancyLayer(indoorSpace, 60f, 0f, false);
            layer.RecordPathOccupancy(result.Path, result.Times, "R1", result.PlanId);

            // Verify end-to-end flow works
            var c1Occ = layer.GetOccupancies("c1", 0f, 10f);
            var c2Occ = layer.GetOccupancies("c2", 0f, 10f);
            Assert.IsTrue(c1Occ.Count > 0 || c2Occ.Count > 0,
                "Planner → Occupancy pipeline should record cell occupancies");
        }
    }
}
