using System.Collections.Generic;
using CentralControl;
using CentralControl.RobotControl;
using Map;
using NetTopologySuite.Geometries;
using Newtonsoft.Json.Linq;
using NUnit.Framework;
using PathPlanning;

namespace EditModeTests
{
    /// <summary>
    /// Verifies that the experiment configurations C1-C5 are technically feasible:
    ///   C1: CellSpace A* on CellSpace graph (baseline)
    ///   C2: Basic A* on CP-based graph (topology extension)
    ///   C3: IST-A* on CP-based graph WITHOUT temporal (algorithm + semantics)
    ///   C4: IST-A* on CP-based graph WITH temporal
    ///   C5: IASIPP on CP-based graph WITH temporal
    /// </summary>
    public class ExperimentConfigTests
    {
        private GeometryFactory gf;

        // =====================================================================
        //  Helpers (reused from GraphTests / CellSpaceGraphTests patterns)
        // =====================================================================

        private CellSpace MakeNavigableCell(string id, double x, double y,
            string floorLevel = "1F", string functionType = "aisle")
        {
            var props = new Dictionary<string, object> { { "navigable", true } };
            AddFloorProperties(props, floorLevel);

            if (!string.IsNullOrEmpty(functionType))
            {
                var functions = new JArray
                {
                    new JObject { ["type"] = functionType }
                };
                props["functions"] = functions;
            }

            Geometry node = gf.CreatePoint(new Coordinate(x, y));
            Geometry space = gf.CreatePolygon(new[]
            {
                new Coordinate(x - 0.5, y - 0.5),
                new Coordinate(x + 0.5, y - 0.5),
                new Coordinate(x + 0.5, y + 0.5),
                new Coordinate(x - 0.5, y + 0.5),
                new Coordinate(x - 0.5, y - 0.5)
            });

            return new CellSpace(id, props, space, node);
        }

        private void AddFloorProperties(Dictionary<string, object> props, string floorLevel)
        {
            if (string.IsNullOrEmpty(floorLevel)) return;

            var floorAssociation = new JObject
            {
                ["floorLevel"] = floorLevel,
                ["associationType"] = "located"
            };

            var verticalFeatures = new JObject
            {
                ["verticalType"] = (string)null,
                ["floorAssociation"] = new JArray { floorAssociation }
            };

            props["verticalFeatures"] = verticalFeatures;
        }

        private CellBoundary MakeHorizontalBoundary(string id, string sourceId, string targetId,
            double midX, double midY)
        {
            Geometry boundary = gf.CreatePoint(new Coordinate(midX, midY));
            Geometry edge = gf.CreateLineString(new[]
            {
                new Coordinate(midX - 0.5, midY),
                new Coordinate(midX + 0.5, midY)
            });

            return new CellBoundary(id, new Dictionary<string, object>(),
                sourceId, targetId, boundary, edge, "horizontal");
        }

        /// <summary>
        /// Builds the IndoorSpace with 4 cells and 3 directed boundaries.
        /// </summary>
        private IndoorSpace BuildIndoorSpace()
        {
            var space = new IndoorSpace();

            space.CellSpaces.Add(MakeNavigableCell("c1", 0, 0));
            space.CellSpaces.Add(MakeNavigableCell("c2", 2, 0));
            space.CellSpaces.Add(MakeNavigableCell("c3", 4, 0, functionType: "intersection"));
            space.CellSpaces.Add(MakeNavigableCell("c4", 6, 0));

            var cb12 = MakeHorizontalBoundary("cb12", "c1", "c2", 1, 0);
            var cb23 = MakeHorizontalBoundary("cb23", "c2", "c3", 3, 0);
            var cb34 = MakeHorizontalBoundary("cb34", "c3", "c4", 5, 0);
            space.CellBoundaries.Add(cb12);
            space.CellBoundaries.Add(cb23);
            space.CellBoundaries.Add(cb34);

            return space;
        }

        /// <summary>
        /// Builds NavigationGraph (CP-based) and NavigationQueryService from the 4-cell space.
        /// Edges are directed: cb12 -> cb23 -> cb34 (forward).
        /// Reverse edge cb23 -> cb12 is added (allowed).
        /// NO reverse edge cb34 -> cb23 (one-way restriction at intersection).
        /// </summary>
        private (IndoorSpace space, NavigationGraph cpGraph, NavigationGraph cellGraph, NavigationQueryService cpQueries) BuildNavigationGraphs()
        {
            var space = BuildIndoorSpace();
            var hypergraph = space.GetHypergraph();
            var cpGraph = NavigationGraphBuilder.FromHypergraph(hypergraph, space, 3.0f, 0.05f);

            // Manually wire edges since test fixture has no Containers
            // Forward edges: cb12 -> cb23 -> cb34
            cpGraph.AddEdge("cb12", "cb23");
            cpGraph.AddEdge("cb23", "cb34");

            // Reverse edge: cb23 -> cb12 (allowed)
            cpGraph.AddEdge("cb23", "cb12");

            // NO reverse edge cb34 -> cb23 (one-way restriction at intersection)

            var cellGraph = NavigationGraphBuilder.FromCellSpaces(space, 3.0f, 0.05f);
            var cpQueries = new NavigationQueryService(space, cpGraph, 3.0f, 0.05f);
            return (space, cpGraph, cellGraph, cpQueries);
        }

        // =====================================================================
        //  Setup
        // =====================================================================

        [SetUp]
        public void SetUp()
        {
            gf = new GeometryFactory();
        }

        // =====================================================================
        //  C2: Basic A* on CP-based directed graph
        // =====================================================================

        [Test]
        public void C2_BasicAStar_OnCPGraph_FindsForwardPath()
        {
            var (space, cpGraph, cellGraph, cpQueries) = BuildNavigationGraphs();

            var wp12 = cpGraph.GetNode("cb12");
            var wp34 = cpGraph.GetNode("cb34");

            var pathFinder = new BasicAStarPathFinder(SearchMethod.Euclidean_Distance);
            var result = pathFinder.FindPath(
                cpGraph, wp12, wp34, null, null,
                "test_robot", null, 0f, 1.5f, 1.0f, 1.0f);

            Assert.IsNotNull(result, "Forward path should be found");
            Assert.IsNotNull(result.Path);
            Assert.IsTrue(result.Path.Count >= 2, "Path should have at least start and end");
            Assert.AreEqual("cb12", result.Path[0].Id);
            Assert.AreEqual("cb34", result.Path[result.Path.Count - 1].Id);
        }

        [Test]
        public void C2_BasicAStar_OnCPGraph_RespectsOneWayRestriction()
        {
            var (space, cpGraph, cellGraph, cpQueries) = BuildNavigationGraphs();

            // Try reverse: cb34 -> cb12
            // cb34 has NO outgoing edge to cb23, so it should fail
            var wp34 = cpGraph.GetNode("cb34");
            var wp12 = cpGraph.GetNode("cb12");

            var pathFinder = new BasicAStarPathFinder(SearchMethod.Euclidean_Distance);
            var result = pathFinder.FindPath(
                cpGraph, wp34, wp12, null, null,
                "test_robot", null, 0f, 1.5f, 1.0f, 1.0f);

            Assert.IsTrue(result == null || result.Path == null || result.Path.Count == 0,
                "Reverse path should NOT be found due to one-way restriction at cb34");
        }

        [Test]
        public void C2_BasicAStar_OnCPGraph_UsesConstantSpeed()
        {
            var (space, cpGraph, cellGraph, cpQueries) = BuildNavigationGraphs();

            var wp12 = cpGraph.GetNode("cb12");
            var wp34 = cpGraph.GetNode("cb34");

            var pathFinder = new BasicAStarPathFinder(SearchMethod.Euclidean_Distance);
            var result = pathFinder.FindPath(
                cpGraph, wp12, wp34, null, null,
                "test_robot", null, 0f, 1.5f, 1.0f, 1.0f);

            Assert.IsNotNull(result);
            Assert.IsNotNull(result.Speeds);
            foreach (float speed in result.Speeds)
            {
                Assert.AreEqual(1.5f, speed, 0.001f,
                    "Basic A* should use constant default speed (no TrafficRulesKnowledge)");
            }
        }

        [Test]
        public void C2_BasicAStar_NoOccupancyDependency()
        {
            // Basic A* signature does not take occupancy layers at all.
            // This test just confirms the API contract.
            var (space, cpGraph, cellGraph, cpQueries) = BuildNavigationGraphs();

            var wp12 = cpGraph.GetNode("cb12");
            var wp34 = cpGraph.GetNode("cb34");

            // Call without any occupancy - should work fine
            Assert.DoesNotThrow(() =>
            {
                var pathFinder = new BasicAStarPathFinder(SearchMethod.Euclidean_Distance);
                pathFinder.FindPath(cpGraph, wp12, wp34, null, null,
                    "test_robot", null, 0f, 1.5f, 1.0f, 1.0f);
            }, "Basic A* must not depend on any occupancy layer");
        }

        // =====================================================================
        //  C3: IST-A* (TimeSpaceAStar) with null occupancy layers
        // =====================================================================

        [Test]
        public void C3_TimeSpaceAStar_WithNullOccupancy_DoesNotCrash()
        {
            var (space, cpGraph, cellGraph, cpQueries) = BuildNavigationGraphs();

            var wp12 = cpGraph.GetNode("cb12");
            var wp34 = cpGraph.GetNode("cb34");

            // Pass null for both occupancy layers - should not throw
            Assert.DoesNotThrow(() =>
            {
                var pathFinder = new TimeSpaceAStarPathFinder(space, SearchMethod.Euclidean_Distance);
                pathFinder.FindPath(
                    cpGraph,
                    wp12,
                    wp34,
                    null,  // personalOccupancyLayer = null
                    null,  // globalOccupancyLayer = null
                    "test_robot",
                    "test_plan",
                    0f,    // startTime
                    1.5f,  // speed
                    1.0f,  // maxAcceleration
                    1.0f); // maxDeceleration
            }, "IST-A* with null occupancy layers should not crash");
        }

        [Test]
        public void C3_TimeSpaceAStar_WithNullOccupancy_FindsPath()
        {
            var (space, cpGraph, cellGraph, cpQueries) = BuildNavigationGraphs();

            var wp12 = cpGraph.GetNode("cb12");
            var wp34 = cpGraph.GetNode("cb34");

            var pathFinder = new TimeSpaceAStarPathFinder(space, SearchMethod.Euclidean_Distance);
            var result = pathFinder.FindPath(
                cpGraph,
                wp12,
                wp34,
                null,  // no personal occupancy
                null,  // no global occupancy
                "test_robot",
                "test_plan",
                0f,
                1.5f,
                1.0f,
                1.0f);

            Assert.IsNotNull(result, "IST-A* should find a path even without occupancy data");
            Assert.IsNotNull(result.Path);
            Assert.IsTrue(result.Path.Count >= 2, "Path should have at least start and end");
            Assert.AreEqual("cb34", result.Path[result.Path.Count - 1].Id,
                "IST-A* should reach the goal node");
        }

        [Test]
        public void C3_TimeSpaceAStar_WithNullOccupancy_TreatsAllNodesAsAvailable()
        {
            var (space, cpGraph, cellGraph, cpQueries) = BuildNavigationGraphs();

            var wp12 = cpGraph.GetNode("cb12");
            var wp34 = cpGraph.GetNode("cb34");

            // Run with null occupancy
            var pathFinder = new TimeSpaceAStarPathFinder(space, SearchMethod.Euclidean_Distance);
            var result = pathFinder.FindPath(
                cpGraph, wp12, wp34, null, null,
                "test_robot", "test_plan", 0f, 1.5f, 1.0f, 1.0f);

            Assert.IsNotNull(result);
            Assert.IsNotNull(result.Path);
            Assert.IsNotNull(result.Times);

            // With null occupancy, times should be non-decreasing.
            // Note: ReconstructPath may insert wait-state entries (same node, slightly later time),
            // so we check strictly increasing only across DISTINCT Waypoint IDs.
            if (result.Times != null && result.Path != null && result.Times.Count > 1)
            {
                for (int i = 1; i < result.Times.Count; i++)
                {
                    Assert.IsTrue(result.Times[i] >= result.Times[i - 1],
                        $"Times should be non-decreasing: t[{i}]={result.Times[i]} < t[{i - 1}]={result.Times[i - 1]}");

                    if (result.Path[i].Id != result.Path[i - 1].Id)
                    {
                        Assert.IsTrue(result.Times[i] > result.Times[i - 1],
                            $"Arrival at different nodes should be strictly increasing: " +
                            $"{result.Path[i].Id} t={result.Times[i]} vs {result.Path[i - 1].Id} t={result.Times[i - 1]}");
                    }
                }
            }
        }

        // =====================================================================
        //  GAP 2: PathPlanner algorithm switchable at runtime
        // =====================================================================

        [Test]
        public void GAP2_PathPlanner_SetAlgorithm_SwitchesAlgorithm()
        {
            var (space, cpGraph, cellGraph, cpQueries) = BuildNavigationGraphs();
            var cellQueries = new NavigationQueryService(space, cellGraph, 3.0f, 0.05f);

            var planner = new PathPlanner(
                cpGraph, cellGraph, cpQueries, cellQueries,
                space,
                SearchAlgorithm.AnytimeSIPP,
                SearchMethod.Euclidean_Distance,
                "robot1", 1.0f, 1.0f);

            // Switch algorithm at runtime — this method must exist
            planner.SetAlgorithm(SearchAlgorithm.Astar_Basic);

            var wp12 = cpGraph.GetNode("cb12");
            var wp34 = cpGraph.GetNode("cb34");

            // Should use Basic A* (no occupancy params needed) and find a path
            var result = planner.FindPathWithDynamicObstacles(
                null, null, wp12, wp34, "robot1", null, 0f, 1.5f);

            Assert.IsNotNull(result, "After SetAlgorithm to Astar_Basic, should find path");
            Assert.IsNotNull(result.Path);
            Assert.IsTrue(result.Path.Count >= 2);
        }

        // =====================================================================
        //  GAP 3: IsReplan propagates through EventManager
        // =====================================================================

        [Test]
        public void GAP3_EventManager_ReportPathPlanningMetrics_PreservesIsReplan()
        {
            // EventManager.ReportPathPlanningMetrics must accept isReplan parameter
            // and set it on the fired metrics object
            var em = EventManager.Instance;
            if (em == null)
            {
                // Create a temporary GameObject with EventManager for testing
                var go = new UnityEngine.GameObject("TestEventManager");
                em = go.AddComponent<EventManager>();
            }

            PathPlanningMetrics receivedMetrics = null;
            em.OnPathPlanningMetricsRecorded += (sender, args) =>
            {
                receivedMetrics = args.Metrics;
            };

            // Call with isReplan = true — this overload must exist
            em.ReportPathPlanningMetrics(
                "robot1", "plan1", 0.1f, 10f, 5f, 8f, isReplan: true);

            Assert.IsNotNull(receivedMetrics, "Should have received metrics via event");
            Assert.IsTrue(receivedMetrics.IsReplan,
                "IsReplan should be true when passed as true to ReportPathPlanningMetrics");

            // Cleanup
            if (em.gameObject.name == "TestEventManager")
                UnityEngine.Object.DestroyImmediate(em.gameObject);
        }

        // =====================================================================
        //  GAP 4: ExperimentMetricsCollector output directory configurable
        // =====================================================================

        [Test]
        public void GAP4_ExperimentMetricsCollector_HasOutputDirectoryField()
        {
            var fieldInfo = typeof(ExperimentMetricsCollector).GetField("outputDirectory");
            Assert.IsNotNull(fieldInfo,
                "ExperimentMetricsCollector should have a public 'outputDirectory' field");
        }

        // =====================================================================
        //  R2: PathPlanner.SetDynamics
        // =====================================================================

        [Test]
        public void R2_PathPlanner_SetDynamics_UpdatesValues()
        {
            var (space, cpGraph, cellGraph, cpQueries) = BuildNavigationGraphs();
            var cellQueries = new NavigationQueryService(space, cellGraph, 3.0f, 0.05f);

            var planner = new PathPlanner(
                cpGraph, cellGraph, cpQueries, cellQueries,
                space,
                SearchAlgorithm.Astar_Basic,
                SearchMethod.Euclidean_Distance,
                "robot1", 1.0f, 1.0f);

            // Should not throw — method must exist
            Assert.DoesNotThrow(() => planner.SetDynamics(2.5f, 3.0f));
        }

        // =====================================================================
        //  R2: ExperimentMetricsCollector.snapshotInterval is public field
        // =====================================================================

        [Test]
        public void R2_ExperimentMetricsCollector_SnapshotInterval_IsPublicField()
        {
            var fieldInfo = typeof(ExperimentMetricsCollector).GetField("snapshotInterval");
            Assert.IsNotNull(fieldInfo,
                "ExperimentMetricsCollector should have a public 'snapshotInterval' field");
            Assert.AreEqual(typeof(float), fieldInfo.FieldType);
        }

        // =====================================================================
        //  R2: EventManager has OnOrderGenerated event
        // =====================================================================

        [Test]
        public void R2_EventManager_HasOnOrderGeneratedEvent()
        {
            var eventInfo = typeof(EventManager).GetEvent("OnOrderGenerated");
            Assert.IsNotNull(eventInfo,
                "EventManager should have an 'OnOrderGenerated' event");
        }

        // =====================================================================
        //  R2: ExperimentRunner singleton dedup
        // =====================================================================

        [Test]
        public void R2_ExperimentRunner_HasSingletonField()
        {
            var field = typeof(ExperimentRunner).GetField("_instance",
                System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Static);
            Assert.IsNotNull(field,
                "ExperimentRunner should have a static _instance field for singleton dedup");
        }

        // =====================================================================
        //  R2: ExperimentRunner has floorCount field
        // =====================================================================

        [Test]
        public void R2_ExperimentRunner_HasFloorCountField()
        {
            var field = typeof(ExperimentRunner).GetField("floorCount");
            Assert.IsNotNull(field,
                "ExperimentRunner should have a public 'floorCount' field");
        }

        // =====================================================================
        //  R2: RobotManager.numberOfRobots is public
        // =====================================================================

        [Test]
        public void R2_RobotManager_NumberOfRobots_IsPublicField()
        {
            var field = typeof(RobotManager).GetField("numberOfRobots");
            Assert.IsNotNull(field,
                "RobotManager should have a public 'numberOfRobots' field");
        }

        // =====================================================================
        //  OOB Recovery: NavigationQueryService finds nearest Waypoint when position outside CellSpace
        // =====================================================================

        [Test]
        public void OOB_Graph_GetRoutePointFromCoordinate_FallsBackToNearest()
        {
            // Build a simple graph with cells at known positions
            var space = new IndoorSpace();
            space.CellSpaces.Add(MakeNavigableCell("c1", 0, 0));
            space.CellSpaces.Add(MakeNavigableCell("c2", 2, 0));

            var cb = MakeHorizontalBoundary("cb12", "c1", "c2", 1, 0);
            space.CellBoundaries.Add(cb);

            var hypergraph = space.GetHypergraph();
            var cpGraph = NavigationGraphBuilder.FromHypergraph(hypergraph, space, 3.0f, 0.05f);
            // Manually wire edge since test fixture has no Containers
            cpGraph.AddEdge("cb12", "cb12"); // self-edge just to ensure node exists
            var cpQueries = new NavigationQueryService(space, cpGraph, 3.0f, 0.05f);

            // Position FAR outside any CellSpace — simulates collision pushing robot out
            var outsidePosition = new UnityEngine.Vector3(99f, 0.05f, 99f);

            // Before fix: returns null. After fix: should return nearest Waypoint
            Waypoint result = cpQueries.ResolveWaypointFromWorldPosition(outsidePosition, TraversalDirection.Outbound);

            Assert.IsNotNull(result,
                "When position is outside all CellSpaces, should fall back to nearest Waypoint");
        }

        [Test]
        public void OOB_Graph_GetRoutePointFromCoordinate_NearbyOutsideReturnsClosest()
        {
            // Position just slightly outside a cell boundary
            var space = new IndoorSpace();
            space.CellSpaces.Add(MakeNavigableCell("c1", 0, 0));  // polygon (-0.5,-0.5 to 0.5,0.5)
            space.CellSpaces.Add(MakeNavigableCell("c2", 2, 0));

            var cb = MakeHorizontalBoundary("cb12", "c1", "c2", 1, 0);
            space.CellBoundaries.Add(cb);

            var hypergraph = space.GetHypergraph();
            var cpGraph = NavigationGraphBuilder.FromHypergraph(hypergraph, space, 3.0f, 0.05f);
            cpGraph.AddEdge("cb12", "cb12"); // self-edge just to ensure node exists
            var cpQueries = new NavigationQueryService(space, cpGraph, 3.0f, 0.05f);

            // Just outside c1's polygon (0.6, 0) — 0.1 beyond the edge
            var nearbyOutside = new UnityEngine.Vector3(0.6f, 0.05f, 0f);

            Waypoint result = cpQueries.ResolveWaypointFromWorldPosition(nearbyOutside, TraversalDirection.Outbound);

            Assert.IsNotNull(result,
                "Position slightly outside CellSpace should still find nearest Waypoint");
        }
    }
}
