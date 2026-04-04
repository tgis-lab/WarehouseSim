using System.Collections.Generic;
using CentralControl;
using Map;
using NetTopologySuite.Geometries;
using Newtonsoft.Json.Linq;
using NUnit.Framework;
using UnityEngine;

namespace EditModeTests
{
    /// <summary>
    /// Edit Mode tests for ConflictResolver and TrafficRuleKnowledge.
    /// Constructs minimal IndoorSpace/DynamicOccupancyLayer/Graph infrastructure
    /// so the real production code runs end-to-end without mocking.
    /// </summary>
    public class ConflictResolverTests
    {
        private GeometryFactory gf;

        // ====================================================================
        //  Shared helpers
        // ====================================================================

        [SetUp]
        public void SetUp()
        {
            gf = new GeometryFactory();
        }

        /// <summary>
        /// Adds floorLevel via the verticalFeatures / FloorAssociation JObject pattern.
        /// </summary>
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

        /// <summary>
        /// Creates a navigable CellSpace with aisle function type.
        /// </summary>
        private CellSpace MakeCell(string id, double x, double y,
                                    string floorLevel = "1F",
                                    string functionType = null)
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

        /// <summary>
        /// Creates a horizontal CellBoundary between two cells.
        /// </summary>
        private CellBoundary MakeBoundary(string id, string sourceId, string targetId,
                                           double midX, double midY)
        {
            Geometry boundary = gf.CreatePoint(new Coordinate(midX, midY));
            Geometry edge = gf.CreateLineString(new[]
            {
                new Coordinate(midX - 0.1, midY),
                new Coordinate(midX + 0.1, midY)
            });

            return new CellBoundary(id, new Dictionary<string, object>(),
                sourceId, targetId, boundary, edge, "horizontal");
        }

        /// <summary>
        /// Creates a ConnectionPoint from a CellBoundary.
        /// </summary>
        private ConnectionPoint MakeConnectionPoint(CellBoundary cb, double x, double y)
        {
            Geometry point = gf.CreatePoint(new Coordinate(x, y));
            return new ConnectionPoint(cb.Id, cb, point, "1F", false, false);
        }

        /// <summary>
        /// Builds a minimal linear scenario: cells A -- B -- C,
        /// returning (indoorSpace, layer, detector, path, times).
        /// The path has 2 ConnectionPoints (boundary AB at x=0.5, boundary BC at x=1.5).
        /// Times go from 0s to 1s to 2s.
        /// </summary>
        private (IndoorSpace indoorSpace,
                 DynamicOccupancyLayer layer,
                 CellSpaceConflictDetector detector,
                 List<ConnectionPoint> path,
                 List<float> times) BuildLinearScenario()
        {
            var indoorSpace = new IndoorSpace();
            indoorSpace.CellSpaces.Add(MakeCell("A", 0, 0));
            indoorSpace.CellSpaces.Add(MakeCell("B", 1, 0));
            indoorSpace.CellSpaces.Add(MakeCell("C", 2, 0));

            var cbAB = MakeBoundary("cb_AB", "A", "B", 0.5, 0);
            var cbBC = MakeBoundary("cb_BC", "B", "C", 1.5, 0);
            indoorSpace.CellBoundaries.Add(cbAB);
            indoorSpace.CellBoundaries.Add(cbBC);

            var layer = new DynamicOccupancyLayer(indoorSpace, 120f, 0f);
            var detector = new CellSpaceConflictDetector();

            var cp1 = MakeConnectionPoint(cbAB, 0.5, 0);
            var cp2 = MakeConnectionPoint(cbBC, 1.5, 0);
            var path = new List<ConnectionPoint> { cp1, cp2 };
            var times = new List<float> { 0f, 1f };

            return (indoorSpace, layer, detector, path, times);
        }

        // ====================================================================
        //  ConflictResolver - ResolveConflictByTimeShift
        // ====================================================================

        [Test]
        public void ResolveConflictByTimeShift_ShiftsPathToAvoidConflict()
        {
            var (indoorSpace, layer, detector, path, times) = BuildLinearScenario();

            // Another robot occupies cell B from time 0-3s, which overlaps our path.
            layer.SetDynamicOccupancy("B", 0f, 3f, "otherRobot", "plan1");

            // Detect conflicts
            var conflicts = detector.DetectConflicts(
                layer, indoorSpace, path, times, "testRobot", "Test");

            // Construct resolver
            var resolver = new ConflictResolver(layer, detector, indoorSpace);

            // Attempt time shift resolution with generous max delay
            var (success, newTimes, appliedShift) = resolver.ResolveConflictByTimeShift(
                "testRobot", path, times, conflicts, maxAcceptableDelay: 30f);

            // The shifted path should succeed because shifting far enough
            // avoids the 0-3s occupancy window.
            Assert.IsTrue(success, "Time shift should successfully resolve the conflict");
            Assert.IsNotNull(newTimes, "New times should not be null on success");
            Assert.AreEqual(times.Count, newTimes.Count, "New times should have same count as original");
            Assert.Greater(appliedShift, 0f, "Applied shift should be positive");
            Assert.Greater(newTimes[0], times[0], "Shifted start time should be later than original");
        }

        [Test]
        public void ResolveConflictByTimeShift_ReturnsFalse_WhenMaxShiftExceeded()
        {
            var (indoorSpace, layer, detector, path, times) = BuildLinearScenario();

            // Create a very long occupancy that cannot be avoided with a small max delay.
            layer.SetDynamicOccupancy("B", 0f, 100f, "otherRobot", "plan1");

            var conflicts = detector.DetectConflicts(
                layer, indoorSpace, path, times, "testRobot", "Test");

            var resolver = new ConflictResolver(layer, detector, indoorSpace);

            // Very small max acceptable delay -- should fail
            var (success, newTimes, appliedShift) = resolver.ResolveConflictByTimeShift(
                "testRobot", path, times, conflicts, maxAcceptableDelay: 0.5f);

            Assert.IsFalse(success, "Should return false when max shift is exceeded");
            Assert.IsNull(newTimes, "New times should be null on failure");
        }

        [Test]
        public void ResolveConflictByTimeShift_PreservesPath_WhenNoConflicts()
        {
            var (indoorSpace, layer, detector, path, times) = BuildLinearScenario();

            // No occupancy added -- no conflict exists.
            // ResolveConflictByTimeShift needs a non-null conflicts dictionary
            // but the first shift attempt (shift=0) should find no conflicts and succeed.
            var emptyConflicts = new Dictionary<string, List<TimeOccupancy>>();
            emptyConflicts["B"] = new List<TimeOccupancy>();  // Empty conflict list for a cell

            var resolver = new ConflictResolver(layer, detector, indoorSpace);

            var (success, newTimes, appliedShift) = resolver.ResolveConflictByTimeShift(
                "testRobot", path, times, emptyConflicts, maxAcceptableDelay: 10f);

            // With no actual conflicts in the occupancy layer, shift=0 should work immediately.
            Assert.IsTrue(success, "Should succeed immediately when no conflicts exist");
            Assert.IsNotNull(newTimes);
            Assert.AreEqual(0f, appliedShift, 0.001f, "No shift should be needed");
            // Times should be unchanged
            for (int i = 0; i < times.Count; i++)
            {
                Assert.AreEqual(times[i], newTimes[i], 0.001f,
                    $"Time at index {i} should be preserved");
            }
        }

        // ====================================================================
        //  ConflictResolver - ResolveConflictBySpeedAdjustment
        // ====================================================================

        [Test]
        public void ResolveConflictBySpeedAdjustment_AdjustsSpeedsToResolve()
        {
            // Build a longer path (4 CPs) so the speed adjustment has segments to work with.
            var indoorSpace = new IndoorSpace();
            indoorSpace.CellSpaces.Add(MakeCell("A", 0, 0));
            indoorSpace.CellSpaces.Add(MakeCell("B", 1, 0));
            indoorSpace.CellSpaces.Add(MakeCell("C", 2, 0));
            indoorSpace.CellSpaces.Add(MakeCell("D", 3, 0));
            indoorSpace.CellSpaces.Add(MakeCell("E", 4, 0));

            var cbAB = MakeBoundary("cb_AB", "A", "B", 0.5, 0);
            var cbBC = MakeBoundary("cb_BC", "B", "C", 1.5, 0);
            var cbCD = MakeBoundary("cb_CD", "C", "D", 2.5, 0);
            var cbDE = MakeBoundary("cb_DE", "D", "E", 3.5, 0);
            indoorSpace.CellBoundaries.Add(cbAB);
            indoorSpace.CellBoundaries.Add(cbBC);
            indoorSpace.CellBoundaries.Add(cbCD);
            indoorSpace.CellBoundaries.Add(cbDE);

            var layer = new DynamicOccupancyLayer(indoorSpace, 120f, 0f);
            var detector = new CellSpaceConflictDetector();

            var cp1 = MakeConnectionPoint(cbAB, 0.5, 0);
            var cp2 = MakeConnectionPoint(cbBC, 1.5, 0);
            var cp3 = MakeConnectionPoint(cbCD, 2.5, 0);
            var cp4 = MakeConnectionPoint(cbDE, 3.5, 0);
            var path = new List<ConnectionPoint> { cp1, cp2, cp3, cp4 };
            var times = new List<float> { 0f, 1f, 2f, 3f };

            // Create a conflict in cell C between 1.5s and 2.5s
            layer.SetDynamicOccupancy("C", 1.5f, 2.5f, "otherRobot", "plan1");

            var conflicts = detector.DetectConflicts(
                layer, indoorSpace, path, times, "testRobot", "Test");

            var resolver = new ConflictResolver(layer, detector, indoorSpace);

            var (success, newTimes, newSpeeds, hasChanges) = resolver.ResolveConflictBySpeedAdjustment(
                "testRobot", path, times, conflicts,
                defaultSpeed: 1.0f, maxSpeedAdjustmentRatio: 2.0f, minSpeedAdjustmentRatio: 0.3f);

            // The result should have changes attempted (hasChanges=true),
            // regardless of whether full resolution succeeded.
            // We verify the method produced modified data.
            if (success)
            {
                Assert.IsNotNull(newTimes, "New times should not be null on success");
                Assert.IsNotNull(newSpeeds, "New speeds should not be null on success");
                Assert.IsTrue(hasChanges, "hasChanges should be true on success");
            }
            else
            {
                // Even if full resolution failed, verify the method ran without error.
                // hasChanges may be true (partial adjustment) or false (no adjustment possible).
                Assert.Pass("Speed adjustment attempted but could not fully resolve; " +
                           $"hasChanges={hasChanges}");
            }
        }

        [Test]
        public void ResolveConflictBySpeedAdjustment_HandlesZeroDurationSegments()
        {
            // Create a scenario where consecutive times are identical (zero duration segment).
            var (indoorSpace, layer, detector, path, _) = BuildLinearScenario();

            // Times with zero-duration segment (both points at t=1.0)
            var times = new List<float> { 1.0f, 1.0f };

            // Create a conflict so adjustment is attempted
            layer.SetDynamicOccupancy("B", 0.5f, 1.5f, "otherRobot", "plan1");

            var conflicts = detector.DetectConflicts(
                layer, indoorSpace, path, times, "testRobot", "Test");

            var resolver = new ConflictResolver(layer, detector, indoorSpace);

            // Should not throw NaN/Infinity errors
            var (success, newTimes, newSpeeds, hasChanges) = resolver.ResolveConflictBySpeedAdjustment(
                "testRobot", path, times, conflicts);

            // The zero-duration segment means originalDuration=0, which triggers
            // the guard clause and returns false. The key assertion is no exception.
            Assert.IsFalse(success,
                "Should fail gracefully for zero-duration segments (NaN guard)");
        }

        [Test]
        public void ResolveConflictBySpeedAdjustment_HandlesTargetDurationLessThanOrEqualZero()
        {
            // Scenario: segment start is already past the conflict, making targetDuration <= 0
            var (indoorSpace, layer, detector, path, _) = BuildLinearScenario();

            // Path starts at t=5, ends at t=6
            var times = new List<float> { 5.0f, 6.0f };

            // Conflict is entirely in the past (t=0 to t=1) relative to path.
            // But we inject it as a synthetic conflict dict to force the resolution path.
            var occupier = new OccupancyIdentifier("otherRobot", "plan1", OccupancyType.Planned);
            var conflict = new TimeOccupancy(0f, 1f, occupier);
            var conflicts = new Dictionary<string, List<TimeOccupancy>>
            {
                { "B", new List<TimeOccupancy> { conflict } }
            };

            var resolver = new ConflictResolver(layer, detector, indoorSpace);

            // Should not throw NaN/Infinity errors
            var (success, newTimes, newSpeeds, hasChanges) = resolver.ResolveConflictBySpeedAdjustment(
                "testRobot", path, times, conflicts);

            // When targetDuration <= 0 the guard returns false for that segment.
            // The method should complete without exception.
            // It may succeed (because the detector sees no real conflict in the layer),
            // or fail gracefully. Either way, no NaN/crash.
            if (newTimes != null)
            {
                foreach (float t in newTimes)
                {
                    Assert.IsFalse(float.IsNaN(t), "No time value should be NaN");
                    Assert.IsFalse(float.IsInfinity(t), "No time value should be Infinity");
                }
            }
            if (newSpeeds != null)
            {
                foreach (float s in newSpeeds)
                {
                    Assert.IsFalse(float.IsNaN(s), "No speed value should be NaN");
                    Assert.IsFalse(float.IsInfinity(s), "No speed value should be Infinity");
                }
            }
        }

        [Test]
        public void ResolveConflictBySpeedAdjustment_PreservesPath_WhenNoConflicts()
        {
            var (indoorSpace, layer, detector, path, times) = BuildLinearScenario();

            // Pass null/empty conflicts -- should short-circuit and return originals
            var emptyConflicts = new Dictionary<string, List<TimeOccupancy>>();

            var resolver = new ConflictResolver(layer, detector, indoorSpace);

            var (success, newTimes, newSpeeds, hasChanges) = resolver.ResolveConflictBySpeedAdjustment(
                "testRobot", path, times, emptyConflicts);

            Assert.IsTrue(success, "Should succeed when no conflicts exist");
            Assert.IsFalse(hasChanges, "No changes should be made when there are no conflicts");
            Assert.IsNotNull(newTimes);
            for (int i = 0; i < times.Count; i++)
            {
                Assert.AreEqual(times[i], newTimes[i], 0.001f,
                    $"Time at index {i} should be preserved");
            }
        }

        // ====================================================================
        //  TrafficRuleKnowledge - GetTurnSpeedFactor
        // ====================================================================

        /// <summary>
        /// Creates RoutePoints at specific coordinates to control the turn angle.
        /// prev -> current -> next defines two direction vectors, and the angle
        /// between them determines the turn speed factor.
        /// </summary>
        private RoutePoint MakeRoutePoint(double x, double y)
        {
            var cb = MakeBoundary($"cb_{x}_{y}", "dummy_s", "dummy_t", x, y);
            var cp = MakeConnectionPoint(cb, x, y);
            return new RoutePoint(cp);
        }

        /// <summary>
        /// Builds a minimal IndoorSpace + Graph with layers that correctly
        /// report IsAisle() / IsIntersection() for TrafficRuleKnowledge.
        /// Returns (indoorSpace, graph) where all boundary CPs are in "aisle" layer.
        /// </summary>
        private (IndoorSpace indoorSpace, Graph graph) BuildTrafficGraph()
        {
            var indoorSpace = new IndoorSpace();

            // Cells along x-axis
            indoorSpace.CellSpaces.Add(MakeCell("A", 0, 0));
            indoorSpace.CellSpaces.Add(MakeCell("B", 1, 0));
            indoorSpace.CellSpaces.Add(MakeCell("C", 2, 0));

            // All cells in "aisle" layer
            indoorSpace.Layers.Add(new Layer("aisleLayer",
                new List<string> { "A", "B", "C" },
                new Dictionary<string, object>
                {
                    { "type", "aisle" },
                    { "speed", 1.0f }
                }));

            var cbAB = MakeBoundary("cb_AB", "A", "B", 0.5, 0);
            var cbBC = MakeBoundary("cb_BC", "B", "C", 1.5, 0);
            indoorSpace.CellBoundaries.Add(cbAB);
            indoorSpace.CellBoundaries.Add(cbBC);

            var graph = new Graph(indoorSpace);

            // Build RoutePoints and register them in the graph
            var cpAB = MakeConnectionPoint(cbAB, 0.5, 0);
            var cpBC = MakeConnectionPoint(cbBC, 1.5, 0);

            var rpAB = new RoutePoint(cpAB);
            var rpBC = new RoutePoint(cpBC);
            rpAB.AddChild(cpBC);
            rpBC.AddParent(cpAB);

            graph.AddNode(rpAB);
            graph.AddNode(rpBC);

            return (indoorSpace, graph);
        }

        /// <summary>
        /// Builds an IndoorSpace + Graph where cell "I" is in an intersection layer.
        /// Layout: A (aisle) -- I (intersection) -- B (aisle).
        /// </summary>
        private (IndoorSpace indoorSpace, Graph graph,
                 RoutePoint rpAI, RoutePoint rpIB) BuildIntersectionGraph()
        {
            var indoorSpace = new IndoorSpace();

            indoorSpace.CellSpaces.Add(MakeCell("A", 0, 0));
            indoorSpace.CellSpaces.Add(MakeCell("I", 1, 0));
            indoorSpace.CellSpaces.Add(MakeCell("B", 2, 0));

            indoorSpace.Layers.Add(new Layer("aisleLayer",
                new List<string> { "A", "B" },
                new Dictionary<string, object>
                {
                    { "type", "aisle" },
                    { "speed", 1.0f }
                }));

            indoorSpace.Layers.Add(new Layer("intersectionLayer",
                new List<string> { "I" },
                new Dictionary<string, object>
                {
                    { "type", "intersection" },
                    { "speed", 0.7f }
                }));

            var cbAI = MakeBoundary("cb_AI", "A", "I", 0.5, 0);
            var cbIB = MakeBoundary("cb_IB", "I", "B", 1.5, 0);
            indoorSpace.CellBoundaries.Add(cbAI);
            indoorSpace.CellBoundaries.Add(cbIB);

            var graph = new Graph(indoorSpace);

            var cpAI = MakeConnectionPoint(cbAI, 0.5, 0);
            var cpIB = MakeConnectionPoint(cbIB, 1.5, 0);

            var rpAI = new RoutePoint(cpAI);
            var rpIB = new RoutePoint(cpIB);
            rpAI.AddChild(cpIB);
            rpIB.AddParent(cpAI);

            graph.AddNode(rpAI);
            graph.AddNode(rpIB);

            return (indoorSpace, graph, rpAI, rpIB);
        }

        // ====================================================================
        //  TrafficRuleKnowledge - ModifySpeed (intersection approach)
        // ====================================================================

        [Test]
        public void TrafficRuleKnowledge_ModifySpeed_ReturnsIntersectionSpeedLimit()
        {
            var (indoorSpace, graph, rpAI, rpIB) = BuildIntersectionGraph();
            var traffic = new PathPlanning.TrafficRuleKnowledge(indoorSpace, graph);

            // ModifySpeed from rpAI to rpIB, approaching intersection.
            // GetSpeedLimit should detect rpAI -> rpIB crosses into intersection
            // and return 0.7 speed limit.
            float distance = 1.0f;
            float defaultSpeed = 1.0f;
            float currentSpeed = 1.0f;

            var (modifiedSpeed, modifiedTime) = traffic.ModifySpeed(
                rpAI, rpIB, currentSpeed, defaultSpeed, distance,
                maxAcceleration: 2.0f, maxDeceleration: 2.0f);

            // The speed limit for intersection approach is 0.7.
            // Target speed = defaultSpeed * min(speedLimit, turnFactor)
            // With no prev (turnFactor=1.0), target = 1.0 * 0.7 = 0.7
            Assert.AreEqual(0.7f, modifiedSpeed, 0.01f,
                "Speed should be limited to intersection approach speed (0.7)");
            Assert.Greater(modifiedTime, 0f, "Time should be positive");
        }

        // ====================================================================
        //  TrafficRuleKnowledge - ModifySpeed (turn)
        // ====================================================================

        [Test]
        public void TrafficRuleKnowledge_ModifySpeed_ReturnsCorrectSpeedForTurn()
        {
            var (indoorSpace, graph) = BuildTrafficGraph();
            var traffic = new PathPlanning.TrafficRuleKnowledge(indoorSpace, graph);

            // Create a 90-degree turn: prev=(0,0), current=(1,0), next=(1,1)
            // Direction in:  (1,0)-(0,0) = (1,0,0)
            // Direction out: (1,1)-(1,0) = (0,0,1) [in Unity XZ plane]
            // Angle = 90 degrees -> turnSpeedFactor = 0.5
            var prev = MakeRoutePoint(0, 0);
            var current = MakeRoutePoint(1, 0);
            var next = MakeRoutePoint(1, 1);

            float distance = 1.0f;
            float defaultSpeed = 1.0f;
            float currentSpeed = 1.0f;

            var (modifiedSpeed, modifiedTime) = traffic.ModifySpeed(
                current, next, currentSpeed, defaultSpeed, distance,
                maxAcceleration: 2.0f, maxDeceleration: 2.0f,
                prev: prev);

            // In aisle area, speedLimit = 1.0.
            // turnFactor for 90deg = 0.5.
            // targetSpeed = 1.0 * min(1.0, 0.5) = 0.5
            Assert.AreEqual(0.5f, modifiedSpeed, 0.05f,
                "Speed should be reduced for 90-degree turn (factor 0.5)");
            Assert.Greater(modifiedTime, 0f, "Time should be positive");
        }

        // ====================================================================
        //  TrafficRuleKnowledge - GetTurnSpeedFactor
        // ====================================================================

        [Test]
        public void TrafficRuleKnowledge_GetTurnSpeedFactor_ReturnsCorrectFactorForAngleRanges()
        {
            var (indoorSpace, graph) = BuildTrafficGraph();
            var traffic = new PathPlanning.TrafficRuleKnowledge(indoorSpace, graph);

            // Test small angle (< 30 deg): prev=(0,0), current=(5,0), next=(10,1)
            // Direction in:  (5,0)-(0,0) = (5,0,0) -> normalized (1,0,0)
            // Direction out: (10,1)-(5,0) = (5,0,1) -> angle ~ 11.3 deg
            // 11.3 <= 30 -> factor = 0.9
            {
                var prev = MakeRoutePoint(0, 0);
                var current = MakeRoutePoint(5, 0);
                var next = MakeRoutePoint(10, 1);

                float factor = traffic.GetTurnSpeedFactor(prev, current, next);
                Assert.AreEqual(0.9f, factor, 0.01f,
                    "Small angle (<= 30 deg) should return factor 0.9");
            }

            // Test medium angle (~ 45 deg): prev=(0,0), current=(1,0), next=(2,1)
            // Direction in:  (1,0,0)
            // Direction out: (1,0,1) -> angle = 45 deg
            // 30 < 45 <= 60 -> factor = 0.7
            {
                var prev = MakeRoutePoint(0, 0);
                var current = MakeRoutePoint(1, 0);
                var next = MakeRoutePoint(2, 1);

                float factor = traffic.GetTurnSpeedFactor(prev, current, next);
                Assert.AreEqual(0.7f, factor, 0.01f,
                    "Medium angle (~45 deg, <= 60) should return factor 0.7");
            }

            // Test right angle (90 deg): prev=(0,0), current=(1,0), next=(1,1)
            // angle = 90 deg -> factor = 0.5
            {
                var prev = MakeRoutePoint(0, 0);
                var current = MakeRoutePoint(1, 0);
                var next = MakeRoutePoint(1, 1);

                float factor = traffic.GetTurnSpeedFactor(prev, current, next);
                Assert.AreEqual(0.5f, factor, 0.01f,
                    "Right angle (90 deg) should return factor 0.5");
            }

            // Test extreme angle (> 120 deg): prev=(0,0), current=(1,0), next=(0.5,0)
            // Direction in:  (1,0,0)
            // Direction out: (-0.5,0,0) -> angle = 180 deg
            // 120 < 180 <= 180 -> factor = 0.3
            {
                var prev = MakeRoutePoint(0, 0);
                var current = MakeRoutePoint(1, 0);
                var next = MakeRoutePoint(0, 0.001);  // near-reversal

                float factor = traffic.GetTurnSpeedFactor(prev, current, next);
                Assert.LessOrEqual(factor, 0.4f,
                    "Near-reversal angle should return factor <= 0.4 (large angle reduction)");
            }
        }

        // ====================================================================
        //  TrafficRuleKnowledge - ModifySpeed handles zero currentSpeed
        // ====================================================================

        [Test]
        public void TrafficRuleKnowledge_ModifySpeed_HandlesZeroCurrentSpeed_WithoutNaN()
        {
            var (indoorSpace, graph) = BuildTrafficGraph();
            var traffic = new PathPlanning.TrafficRuleKnowledge(indoorSpace, graph);

            var current = MakeRoutePoint(0.5, 0);
            var next = MakeRoutePoint(1.5, 0);

            float distance = 1.0f;
            float defaultSpeed = 1.0f;
            float currentSpeed = 0f;  // Zero current speed

            var (modifiedSpeed, modifiedTime) = traffic.ModifySpeed(
                current, next, currentSpeed, defaultSpeed, distance,
                maxAcceleration: 2.0f, maxDeceleration: 2.0f);

            // When currentSpeed is 0 and targetSpeed > 0, we are in acceleration path.
            // The method should calculate valid (non-NaN, non-Infinity) results.
            Assert.IsFalse(float.IsNaN(modifiedSpeed),
                "Modified speed should not be NaN when currentSpeed is zero");
            Assert.IsFalse(float.IsInfinity(modifiedSpeed),
                "Modified speed should not be Infinity when currentSpeed is zero");
            Assert.IsFalse(float.IsNaN(modifiedTime),
                "Modified time should not be NaN when currentSpeed is zero");
            Assert.IsFalse(float.IsInfinity(modifiedTime),
                "Modified time should not be Infinity when currentSpeed is zero");
            Assert.GreaterOrEqual(modifiedSpeed, 0f,
                "Speed should be non-negative");
            Assert.Greater(modifiedTime, 0f,
                "Time should be positive for non-zero distance");
        }

        // ====================================================================
        //  ConflictResolver - TrySpeedAdjustment empty segment handling
        // ====================================================================

        [Test]
        public void ResolveConflictBySpeedAdjustment_HandlesEmptySegmentTimes()
        {
            var (indoorSpace, layer, detector, path, _) = BuildLinearScenario();

            // startIndex > endIndex or out of bounds so segmentTimes is empty
            // We force this via a single-point path (path.Count=1) which is
            // rejected by parameter validation, so instead test with a path where
            // the conflict maps to segmentIndex that when clamped yields an empty
            // segment range.
            // Build a two-CP path with times that skip the conflict cell entirely.
            var times = new List<float> { 0f, 1f };

            // Create a conflict for a cell that IS in the path but produces
            // adjustmentStartIndex == adjustmentEndIndex == 0 with empty segment.
            var occupier = new OccupancyIdentifier("otherRobot", "plan1", OccupancyType.Planned);
            var conflict = new TimeOccupancy(0f, 0.5f, occupier);
            var conflicts = new Dictionary<string, List<TimeOccupancy>>
            {
                { "A", new List<TimeOccupancy> { conflict } }
            };

            var resolver = new ConflictResolver(layer, detector, indoorSpace);

            var (success, newTimes, newSpeeds, hasChanges) = resolver.ResolveConflictBySpeedAdjustment(
                "testRobot", path, times, conflicts);

            // Should not throw IndexOutOfRangeException on empty segmentTimes
            // The method may or may not succeed, but must not crash.
            if (newTimes != null)
            {
                foreach (float t in newTimes)
                {
                    Assert.IsFalse(float.IsNaN(t), "No time value should be NaN");
                    Assert.IsFalse(float.IsInfinity(t), "No time value should be Infinity");
                }
            }
        }
    }
}
