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
        /// Builds a 4-cell directed CP graph with a one-way constraint:
        ///   c1 (0,0) -&gt; c2 (2,0) -&gt; c3 (4,0) -&gt; c4 (6,0)
        /// Edges are directed: c1-&gt;c2-&gt;c3-&gt;c4.
        /// Also adds reverse edges c2-&gt;c1, c3-&gt;c2, c4-&gt;c3 to allow bidirectional travel
        /// EXCEPT c3-&gt;c2 is omitted to simulate a one-way restriction.
        /// </summary>
        private (IndoorSpace space, Graph graph) BuildDirectedCPGraph()
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

            var graph = new Graph(space, 3.0f);

            // Create ConnectionPoints from boundaries
            var cp12 = space.GetConnectionPointFromCellBoundary("cb12");
            var cp23 = space.GetConnectionPointFromCellBoundary("cb23");
            var cp34 = space.GetConnectionPointFromCellBoundary("cb34");

            var rp12 = new RoutePoint(cp12);
            var rp23 = new RoutePoint(cp23);
            var rp34 = new RoutePoint(cp34);

            graph.AddNode(rp12);
            graph.AddNode(rp23);
            graph.AddNode(rp34);

            // Forward edges: cp12 -> cp23 -> cp34
            rp12.AddChild(cp23);
            rp23.AddParent(cp12);

            rp23.AddChild(cp34);
            rp34.AddParent(cp23);

            // Reverse edge: cp23 -> cp12 (allowed)
            rp23.AddChild(cp12);
            rp12.AddParent(cp23);

            // NO reverse edge cp34 -> cp23 (one-way restriction at intersection)

            return (space, graph);
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
            var (space, graph) = BuildDirectedCPGraph();

            var rp12 = graph.GetRoutePointFromConnectionPoint(
                space.GetConnectionPointFromCellBoundary("cb12"));
            var rp34 = graph.GetRoutePointFromConnectionPoint(
                space.GetConnectionPointFromCellBoundary("cb34"));

            var (path, speeds) = Astar_Basic.AstarAlgorithm(
                graph, rp12, rp34, 1.5f, SearchMethod.Euclidean_Distance);

            Assert.IsNotNull(path, "Forward path should be found");
            Assert.IsTrue(path.Count >= 2, "Path should have at least start and end");
            Assert.AreEqual("cb12", path[0].Id);
            Assert.AreEqual("cb34", path[path.Count - 1].Id);
        }

        [Test]
        public void C2_BasicAStar_OnCPGraph_RespectsOneWayRestriction()
        {
            var (space, graph) = BuildDirectedCPGraph();

            // Try reverse: cp34 -> cp12
            // cp34 has NO child edge to cp23, so it should fail
            var rp34 = graph.GetRoutePointFromConnectionPoint(
                space.GetConnectionPointFromCellBoundary("cb34"));
            var rp12 = graph.GetRoutePointFromConnectionPoint(
                space.GetConnectionPointFromCellBoundary("cb12"));

            var (path, speeds) = Astar_Basic.AstarAlgorithm(
                graph, rp34, rp12, 1.5f, SearchMethod.Euclidean_Distance);

            Assert.IsTrue(path == null || path.Count == 0,
                "Reverse path should NOT be found due to one-way restriction at cp34");
        }

        [Test]
        public void C2_BasicAStar_OnCPGraph_UsesConstantSpeed()
        {
            var (space, graph) = BuildDirectedCPGraph();

            var rp12 = graph.GetRoutePointFromConnectionPoint(
                space.GetConnectionPointFromCellBoundary("cb12"));
            var rp34 = graph.GetRoutePointFromConnectionPoint(
                space.GetConnectionPointFromCellBoundary("cb34"));

            var (path, speeds) = Astar_Basic.AstarAlgorithm(
                graph, rp12, rp34, 1.5f, SearchMethod.Euclidean_Distance);

            Assert.IsNotNull(speeds);
            foreach (float speed in speeds)
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
            var (space, graph) = BuildDirectedCPGraph();

            var rp12 = graph.GetRoutePointFromConnectionPoint(
                space.GetConnectionPointFromCellBoundary("cb12"));
            var rp34 = graph.GetRoutePointFromConnectionPoint(
                space.GetConnectionPointFromCellBoundary("cb34"));

            // Call without any occupancy - should work fine
            Assert.DoesNotThrow(() =>
            {
                Astar_Basic.AstarAlgorithm(graph, rp12, rp34, 1.5f, SearchMethod.Euclidean_Distance);
            }, "Basic A* must not depend on any occupancy layer");
        }

        // =====================================================================
        //  C3: IST-A* (TimeSpaceAStar) with null occupancy layers
        // =====================================================================

        [Test]
        public void C3_TimeSpaceAStar_WithNullOccupancy_DoesNotCrash()
        {
            var (space, graph) = BuildDirectedCPGraph();

            var rp12 = graph.GetRoutePointFromConnectionPoint(
                space.GetConnectionPointFromCellBoundary("cb12"));
            var rp34 = graph.GetRoutePointFromConnectionPoint(
                space.GetConnectionPointFromCellBoundary("cb34"));

            // Pass null for both occupancy layers - should not throw
            Assert.DoesNotThrow(() =>
            {
                TimeSpaceAStar.TimeSpaceAStarAlgorithm(
                    graph,
                    space,
                    null,  // personalOccupancyLayer = null
                    null,  // globalOccupancyLayer = null
                    rp12,
                    rp34,
                    "test_robot",
                    "test_plan",
                    0f,    // startTime
                    1.5f,  // speed
                    1.0f,  // maxAcceleration
                    1.0f,  // maxDeceleration
                    SearchMethod.Euclidean_Distance);
            }, "IST-A* with null occupancy layers should not crash");
        }

        [Test]
        public void C3_TimeSpaceAStar_WithNullOccupancy_FindsPath()
        {
            var (space, graph) = BuildDirectedCPGraph();

            var rp12 = graph.GetRoutePointFromConnectionPoint(
                space.GetConnectionPointFromCellBoundary("cb12"));
            var rp34 = graph.GetRoutePointFromConnectionPoint(
                space.GetConnectionPointFromCellBoundary("cb34"));

            var (path, times, speeds, planId, metrics) = TimeSpaceAStar.TimeSpaceAStarAlgorithm(
                graph,
                space,
                null,  // no personal occupancy
                null,  // no global occupancy
                rp12,
                rp34,
                "test_robot",
                "test_plan",
                0f,
                1.5f,
                1.0f,
                1.0f,
                SearchMethod.Euclidean_Distance);

            Assert.IsNotNull(path, "IST-A* should find a path even without occupancy data");
            Assert.IsTrue(path.Count >= 2, "Path should have at least start and end");
            Assert.AreEqual("cb34", path[path.Count - 1].Id,
                "IST-A* should reach the goal node");
        }

        [Test]
        public void C3_TimeSpaceAStar_WithNullOccupancy_TreatsAllNodesAsAvailable()
        {
            var (space, graph) = BuildDirectedCPGraph();

            var rp12 = graph.GetRoutePointFromConnectionPoint(
                space.GetConnectionPointFromCellBoundary("cb12"));
            var rp34 = graph.GetRoutePointFromConnectionPoint(
                space.GetConnectionPointFromCellBoundary("cb34"));

            // Run with null occupancy
            var (pathNoOcc, timesNoOcc, _, _, _) = TimeSpaceAStar.TimeSpaceAStarAlgorithm(
                graph, space, null, null, rp12, rp34,
                "test_robot", "test_plan", 0f, 1.5f, 1.0f, 1.0f,
                SearchMethod.Euclidean_Distance);

            Assert.IsNotNull(pathNoOcc);
            Assert.IsNotNull(timesNoOcc);

            // With null occupancy, times should be non-decreasing.
            // Note: ReconstructPath may insert wait-state entries (same node, slightly later time),
            // so we check strictly increasing only across DISTINCT ConnectionPoint IDs.
            if (timesNoOcc != null && pathNoOcc != null && timesNoOcc.Count > 1)
            {
                for (int i = 1; i < timesNoOcc.Count; i++)
                {
                    Assert.IsTrue(timesNoOcc[i] >= timesNoOcc[i - 1],
                        $"Times should be non-decreasing: t[{i}]={timesNoOcc[i]} < t[{i - 1}]={timesNoOcc[i - 1]}");

                    if (pathNoOcc[i].Id != pathNoOcc[i - 1].Id)
                    {
                        Assert.IsTrue(timesNoOcc[i] > timesNoOcc[i - 1],
                            $"Arrival at different nodes should be strictly increasing: " +
                            $"{pathNoOcc[i].Id} t={timesNoOcc[i]} vs {pathNoOcc[i - 1].Id} t={timesNoOcc[i - 1]}");
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
            var (space, graph) = BuildDirectedCPGraph();

            var planner = new PathPlanner(
                graph, space,
                SearchAlgorithm.AnytimeSIPP,
                SearchMethod.Euclidean_Distance,
                "robot1", 1.0f, 1.0f);

            // Switch algorithm at runtime — this method must exist
            planner.SetAlgorithm(SearchAlgorithm.Astar_Basic);

            var rp12 = graph.GetRoutePointFromConnectionPoint(
                space.GetConnectionPointFromCellBoundary("cb12"));
            var rp34 = graph.GetRoutePointFromConnectionPoint(
                space.GetConnectionPointFromCellBoundary("cb34"));

            // Should use Basic A* (no occupancy params needed) and find a path
            var (path, times, speeds, planId, metrics) = planner.FindPathWithDynamicObstacles(
                null, null, rp12, rp34, "robot1", null, 0f, 1.5f);

            Assert.IsNotNull(path, "After SetAlgorithm to Astar_Basic, should find path");
            Assert.IsTrue(path.Count >= 2);
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
            var (space, graph) = BuildDirectedCPGraph();

            var planner = new PathPlanner(
                graph, space,
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
        //  OOB Recovery: Graph finds nearest RoutePoint when position outside CellSpace
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

            var graph = new Graph(space, 3.0f);
            ConnectionPoint cp12 = space.GetConnectionPointFromCellBoundary("cb12");
            graph.AddNode(new RoutePoint(cp12));

            // Position FAR outside any CellSpace — simulates collision pushing robot out
            var outsidePosition = new UnityEngine.Vector3(99f, 0.05f, 99f);

            // Before fix: returns null. After fix: should return nearest RoutePoint
            RoutePoint result = graph.GetRoutePointFromCoordinate(outsidePosition, true);

            Assert.IsNotNull(result,
                "When position is outside all CellSpaces, should fall back to nearest RoutePoint");
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

            var graph = new Graph(space, 3.0f);
            ConnectionPoint cp12 = space.GetConnectionPointFromCellBoundary("cb12");
            graph.AddNode(new RoutePoint(cp12));

            // Just outside c1's polygon (0.6, 0) — 0.1 beyond the edge
            var nearbyOutside = new UnityEngine.Vector3(0.6f, 0.05f, 0f);

            RoutePoint result = graph.GetRoutePointFromCoordinate(nearbyOutside, true);

            Assert.IsNotNull(result,
                "Position slightly outside CellSpace should still find nearest RoutePoint");
        }
    }
}
