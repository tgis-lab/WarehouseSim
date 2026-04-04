using System.Collections.Generic;
using System.Linq;
using Map;
using NetTopologySuite.Geometries;
using Newtonsoft.Json.Linq;
using NUnit.Framework;

namespace EditModeTests
{
    /// <summary>
    /// Edit Mode tests for IndoorSpace: lookup methods, hypergraph generation,
    /// floor-level queries, and connection-point resolution with fallback.
    /// </summary>
    public class IndoorSpaceTests
    {
        private IndoorSpace indoorSpace;
        private GeometryFactory gf;

        // --------------- Helper Methods ---------------

        /// <summary>
        /// Creates a navigable CellSpace with a Point node at (x, y) and a small square polygon.
        /// Places it on the given floor level.
        /// </summary>
        private CellSpace MakeCell(string id, double x, double y, string floorLevel = "1F",
                                   bool navigable = true, string verticalType = null,
                                   bool isVerticalConnection = false,
                                   List<string> connectedFloors = null)
        {
            var props = new Dictionary<string, object> { { "navigable", navigable } };
            AddFloorProperties(props, floorLevel, verticalType, isVerticalConnection, connectedFloors);

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
        /// Adds verticalFeatures / FloorAssociation JObject pattern
        /// that CellSpace.GetLocatedFloor() and IsVerticalConnection() expect.
        /// </summary>
        private void AddFloorProperties(Dictionary<string, object> props, string floorLevel,
                                        string verticalType = null,
                                        bool isVerticalConnection = false,
                                        List<string> connectedFloors = null)
        {
            if (string.IsNullOrEmpty(floorLevel)) return;

            var associations = new JArray();

            // "located" association for the home floor
            associations.Add(new JObject
            {
                ["floorLevel"] = floorLevel,
                ["associationType"] = "located"
            });

            // "connected" associations for elevator/stair targets
            if (connectedFloors != null)
            {
                foreach (var floor in connectedFloors)
                {
                    associations.Add(new JObject
                    {
                        ["floorLevel"] = floor,
                        ["associationType"] = "connected"
                    });
                }
            }

            var verticalFeatures = new JObject
            {
                ["verticalType"] = verticalType,
                ["floorAssociation"] = associations
            };

            props["verticalFeatures"] = verticalFeatures;

            if (isVerticalConnection)
            {
                props["isVerticalConnection"] = true;
            }
        }

        /// <summary>
        /// Creates a horizontal CellBoundary. The boundary geometry is a Point at the midpoint.
        /// </summary>
        private CellBoundary MakeBoundary(string id, string sourceId, string targetId,
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
        /// Creates a vertical CellBoundary (elevator/stair transition between floors).
        /// </summary>
        private CellBoundary MakeVerticalBoundary(string id, string sourceId, string targetId)
        {
            return new CellBoundary(id, new Dictionary<string, object>(),
                sourceId, targetId, null, null, "vertical");
        }

        /// <summary>
        /// Builds a test IndoorSpace with:
        ///   Floor 1F: c1 -- c2 -- ev1  (aisle cells + elevator cell)
        ///   Floor 2F: ev2 -- c3        (elevator cell + aisle cell)
        ///   Vertical boundary: cb_vert connects ev1 <-> ev2
        ///
        /// Boundaries:
        ///   cb1: c1 -> c2  (horizontal)
        ///   cb2: c2 -> ev1 (horizontal)
        ///   cb_vert: ev1 -> ev2 (vertical)
        ///   cb3: ev2 -> c3 (horizontal)
        ///
        /// Layers:
        ///   layer_aisle: type=aisle, contains {c1, c2, c3}
        ///   layer_elevator: type=elevator, contains {ev1, ev2}
        ///
        /// Containers:
        ///   container1: cell=c2, sources=[cb1], targets=[cb2]
        /// </summary>
        private void BuildTestIndoorSpace()
        {
            indoorSpace = new IndoorSpace();

            // --- CellSpaces ---
            indoorSpace.CellSpaces.Add(MakeCell("c1", 0, 0, "1F"));
            indoorSpace.CellSpaces.Add(MakeCell("c2", 1, 0, "1F"));
            indoorSpace.CellSpaces.Add(MakeCell("ev1", 2, 0, "1F",
                verticalType: "elevator", isVerticalConnection: true,
                connectedFloors: new List<string> { "2F" }));
            indoorSpace.CellSpaces.Add(MakeCell("ev2", 2, 0, "2F",
                verticalType: "elevator", isVerticalConnection: true,
                connectedFloors: new List<string> { "1F" }));
            indoorSpace.CellSpaces.Add(MakeCell("c3", 3, 0, "2F"));

            // --- CellBoundaries ---
            indoorSpace.CellBoundaries.Add(MakeBoundary("cb1", "c1", "c2", 0.5, 0));
            indoorSpace.CellBoundaries.Add(MakeBoundary("cb2", "c2", "ev1", 1.5, 0));
            indoorSpace.CellBoundaries.Add(MakeVerticalBoundary("cb_vert", "ev1", "ev2"));
            indoorSpace.CellBoundaries.Add(MakeBoundary("cb3", "ev2", "c3", 2.5, 0));

            // --- Layers ---
            indoorSpace.Layers.Add(new Layer("layer_aisle",
                new List<string> { "c1", "c2", "c3" },
                new Dictionary<string, object> { { "type", "aisle" }, { "speed", 1.0f } }));

            indoorSpace.Layers.Add(new Layer("layer_elevator",
                new List<string> { "ev1", "ev2" },
                new Dictionary<string, object> { { "type", "elevator" }, { "speed", 0.5f } }));

            // --- Containers ---
            indoorSpace.Containers.Add(new Container("container1", "c2",
                new List<string> { "cb1" },
                new List<string> { "cb2" },
                null));
        }

        // --------------- Setup ---------------

        [SetUp]
        public void SetUp()
        {
            gf = new GeometryFactory();
            indoorSpace = null;
        }

        // ===================================================================
        //  1. GetCellSpaceFromId
        // ===================================================================

        [Test]
        public void GetCellSpaceFromId_Found_ReturnsCorrectCell()
        {
            BuildTestIndoorSpace();

            CellSpace result = indoorSpace.GetCellSpaceFromId("c2");

            Assert.IsNotNull(result);
            Assert.AreEqual("c2", result.Id);
        }

        [Test]
        public void GetCellSpaceFromId_NotFound_ReturnsNull()
        {
            BuildTestIndoorSpace();

            CellSpace result = indoorSpace.GetCellSpaceFromId("nonexistent");

            Assert.IsNull(result);
        }

        // ===================================================================
        //  2. GetCellSpaceFromConnectionPoint
        // ===================================================================

        [Test]
        public void GetCellSpaceFromConnectionPoint_SourceDirection_ReturnsSourceCell()
        {
            BuildTestIndoorSpace();

            // cb1 has source=c1, target=c2
            ConnectionPoint cp = indoorSpace.GetConnectionPointFromCellBoundary("cb1");
            Assert.IsNotNull(cp, "ConnectionPoint for cb1 should exist");

            // sequence=true means source direction
            CellSpace result = indoorSpace.GetCellSpaceFromConnectionPoint(cp, sequence: true);

            Assert.IsNotNull(result);
            Assert.AreEqual("c1", result.Id, "Source of cb1 is c1");
        }

        [Test]
        public void GetCellSpaceFromConnectionPoint_TargetDirection_ReturnsTargetCell()
        {
            BuildTestIndoorSpace();

            // cb1 has source=c1, target=c2
            ConnectionPoint cp = indoorSpace.GetConnectionPointFromCellBoundary("cb1");
            Assert.IsNotNull(cp, "ConnectionPoint for cb1 should exist");

            // sequence=false means target direction
            CellSpace result = indoorSpace.GetCellSpaceFromConnectionPoint(cp, sequence: false);

            Assert.IsNotNull(result);
            Assert.AreEqual("c2", result.Id, "Target of cb1 is c2");
        }

        // ===================================================================
        //  3. GetConnectionPointFromCellSpace (with fallback direction)
        // ===================================================================

        [Test]
        public void GetConnectionPointFromCellSpace_PrimaryDirection_Found()
        {
            BuildTestIndoorSpace();

            // c1 is the source of cb1, so sequence=true should find it on the first pass
            ConnectionPoint cp = indoorSpace.GetConnectionPointFromCellSpace("c1", sequence: true);

            Assert.IsNotNull(cp);
            Assert.AreEqual("cb1", cp.Id, "c1 is source of cb1");
        }

        [Test]
        public void GetConnectionPointFromCellSpace_FallbackDirection_Found()
        {
            BuildTestIndoorSpace();

            // c3 is the target of cb3 (ev2 -> c3). Searching as source (sequence=true)
            // will fail on the first pass, so it should fall back and find c3 as target of cb3.
            ConnectionPoint cp = indoorSpace.GetConnectionPointFromCellSpace("c3", sequence: true);

            Assert.IsNotNull(cp, "Should find connection point via fallback direction");
            Assert.AreEqual("cb3", cp.Id, "c3 is target of cb3, found via fallback");
        }

        [Test]
        public void GetConnectionPointFromCellSpace_NoCellInAnyBoundary_ReturnsNull()
        {
            BuildTestIndoorSpace();

            // Add an orphan cell that does not appear in any boundary
            indoorSpace.CellSpaces.Add(MakeCell("orphan", 10, 10, "1F"));

            ConnectionPoint cp = indoorSpace.GetConnectionPointFromCellSpace("orphan", sequence: true);

            Assert.IsNull(cp, "Orphan cell should not resolve to any connection point");
        }

        // ===================================================================
        //  4. GetCellBoundaryFromId
        // ===================================================================

        [Test]
        public void GetCellBoundaryFromId_Found_ReturnsCorrectBoundary()
        {
            BuildTestIndoorSpace();

            CellBoundary result = indoorSpace.GetCellBoundaryFromId("cb2");

            Assert.IsNotNull(result);
            Assert.AreEqual("cb2", result.Id);
            Assert.AreEqual("c2", result.Source);
            Assert.AreEqual("ev1", result.Target);
        }

        [Test]
        public void GetCellBoundaryFromId_NotFound_ReturnsNull()
        {
            BuildTestIndoorSpace();

            CellBoundary result = indoorSpace.GetCellBoundaryFromId("nonexistent");

            Assert.IsNull(result);
        }

        // ===================================================================
        //  5. GetHypergraph
        // ===================================================================

        [Test]
        public void GetHypergraph_GeneratesCorrectConnectionPointsAndRLineGroups()
        {
            BuildTestIndoorSpace();

            Hypergraph hypergraph = indoorSpace.GetHypergraph();

            Assert.IsNotNull(hypergraph);

            // One ConnectionPoint per CellBoundary: cb1, cb2, cb_vert, cb3
            Assert.AreEqual(4, hypergraph.HyperNodes.Count,
                "Should have one HyperNode per CellBoundary");

            // Verify each boundary is represented
            var nodeIds = hypergraph.HyperNodes.Select(n => n.Id).ToList();
            Assert.Contains("cb1", nodeIds);
            Assert.Contains("cb2", nodeIds);
            Assert.Contains("cb_vert", nodeIds);
            Assert.Contains("cb3", nodeIds);

            // One RLineGroup per Container: container1
            Assert.AreEqual(1, hypergraph.HyperEdges.Count,
                "Should have one HyperEdge per Container");

            RLineGroup rLineGroup = hypergraph.HyperEdges[0];
            Assert.AreEqual("container1", rLineGroup.Id);

            // container1: sources=[cb1], targets=[cb2]
            Assert.AreEqual(1, rLineGroup.Sources.Count);
            Assert.Contains("cb1", rLineGroup.Sources);
            Assert.AreEqual(1, rLineGroup.Targets.Count);
            Assert.Contains("cb2", rLineGroup.Targets);

            // ConnectionPoints should include both sources and targets
            Assert.Contains("cb1", rLineGroup.ConnectionPoints);
            Assert.Contains("cb2", rLineGroup.ConnectionPoints);

            // InnerEdges: cb1 -> cb2 (no limited edges)
            Assert.AreEqual(1, rLineGroup.InnerEdges.Count);
            Assert.AreEqual("cb1", rLineGroup.InnerEdges[0][0]);
            Assert.AreEqual("cb2", rLineGroup.InnerEdges[0][1]);
        }

        [Test]
        public void GetHypergraph_VerticalConnectionPoint_HasCorrectFlags()
        {
            BuildTestIndoorSpace();

            Hypergraph hypergraph = indoorSpace.GetHypergraph();

            // The vertical boundary should produce a ConnectionPoint flagged as elevator
            ConnectionPoint vertCp = hypergraph.HyperNodes.FirstOrDefault(n => n.Id == "cb_vert");
            Assert.IsNotNull(vertCp, "Vertical boundary should produce a ConnectionPoint");
            Assert.IsTrue(vertCp.IsVertical(), "cb_vert should be a vertical connection");
            Assert.IsTrue(vertCp.IsElevator, "cb_vert connects elevator cells, so IsElevator should be true");
            Assert.IsFalse(vertCp.IsStair, "cb_vert is not a stair connection");
        }

        [Test]
        public void GetHypergraph_VerticalConnectionPoint_HasResolvedFloorLevel()
        {
            BuildTestIndoorSpace();

            Hypergraph hypergraph = indoorSpace.GetHypergraph();

            ConnectionPoint vertCp = hypergraph.HyperNodes.FirstOrDefault(n => n.Id == "cb_vert");
            Assert.IsNotNull(vertCp);
            Assert.IsNotNull(vertCp.FloorLevel,
                "Vertical CP FloorLevel must NOT be null — it should be resolved from " +
                "associated CellSpaces (lower floor) for path planning state deduplication");
            Assert.AreEqual("1F", vertCp.FloorLevel,
                "Vertical CP connecting 1F↔2F should resolve to lower floor (1F)");
        }

        [Test]
        public void GetConnectionPointFromCellBoundary_VerticalBoundary_HasResolvedFloorLevel()
        {
            BuildTestIndoorSpace();

            ConnectionPoint vertCp = indoorSpace.GetConnectionPointFromCellBoundary("cb_vert");
            Assert.IsNotNull(vertCp);
            Assert.IsNotNull(vertCp.FloorLevel,
                "Vertical CP from GetConnectionPointFromCellBoundary must have resolved FloorLevel");
            Assert.AreEqual("1F", vertCp.FloorLevel);
        }

        // ===================================================================
        //  6. GetAllFloorLevels
        // ===================================================================

        [Test]
        public void GetAllFloorLevels_ReturnsUniqueFloorsSorted()
        {
            BuildTestIndoorSpace();

            List<string> floors = indoorSpace.GetAllFloorLevels();

            Assert.IsNotNull(floors);
            Assert.AreEqual(2, floors.Count, "Test data has cells on 1F and 2F");
            Assert.AreEqual("1F", floors[0], "1F should sort before 2F");
            Assert.AreEqual("2F", floors[1]);
        }

        [Test]
        public void GetAllFloorLevels_EmptyIndoorSpace_ReturnsEmptyList()
        {
            indoorSpace = new IndoorSpace();

            List<string> floors = indoorSpace.GetAllFloorLevels();

            Assert.IsNotNull(floors);
            Assert.AreEqual(0, floors.Count);
        }

        // ===================================================================
        //  7. GetCellSpacesOnFloor
        // ===================================================================

        [Test]
        public void GetCellSpacesOnFloor_FiltersByFloorCorrectly()
        {
            BuildTestIndoorSpace();

            List<CellSpace> floor1Cells = indoorSpace.GetCellSpacesOnFloor("1F");
            List<CellSpace> floor2Cells = indoorSpace.GetCellSpacesOnFloor("2F");

            // 1F: c1, c2, ev1
            Assert.AreEqual(3, floor1Cells.Count, "1F should have c1, c2, ev1");
            var floor1Ids = floor1Cells.Select(c => c.Id).ToList();
            Assert.Contains("c1", floor1Ids);
            Assert.Contains("c2", floor1Ids);
            Assert.Contains("ev1", floor1Ids);

            // 2F: ev2, c3
            Assert.AreEqual(2, floor2Cells.Count, "2F should have ev2, c3");
            var floor2Ids = floor2Cells.Select(c => c.Id).ToList();
            Assert.Contains("ev2", floor2Ids);
            Assert.Contains("c3", floor2Ids);
        }

        [Test]
        public void GetCellSpacesOnFloor_NonexistentFloor_ReturnsEmpty()
        {
            BuildTestIndoorSpace();

            List<CellSpace> cells = indoorSpace.GetCellSpacesOnFloor("B1");

            Assert.IsNotNull(cells);
            Assert.AreEqual(0, cells.Count);
        }

        // ===================================================================
        //  8. GetVerticalConnectionsOnFloor
        // ===================================================================

        [Test]
        public void GetVerticalConnectionsOnFloor_FindsElevatorCells()
        {
            BuildTestIndoorSpace();

            List<CellSpace> verticals1F = indoorSpace.GetVerticalConnectionsOnFloor("1F");
            List<CellSpace> verticals2F = indoorSpace.GetVerticalConnectionsOnFloor("2F");

            // ev1 on 1F
            Assert.AreEqual(1, verticals1F.Count, "1F should have one vertical connection (ev1)");
            Assert.AreEqual("ev1", verticals1F[0].Id);

            // ev2 on 2F
            Assert.AreEqual(1, verticals2F.Count, "2F should have one vertical connection (ev2)");
            Assert.AreEqual("ev2", verticals2F[0].Id);
        }

        [Test]
        public void GetVerticalConnectionsOnFloor_NoVerticalConnections_ReturnsEmpty()
        {
            // Build a single-floor space with no vertical connections
            indoorSpace = new IndoorSpace();
            indoorSpace.CellSpaces.Add(MakeCell("a1", 0, 0, "1F"));
            indoorSpace.CellSpaces.Add(MakeCell("a2", 1, 0, "1F"));
            indoorSpace.CellBoundaries.Add(MakeBoundary("cb_a", "a1", "a2", 0.5, 0));

            List<CellSpace> verticals = indoorSpace.GetVerticalConnectionsOnFloor("1F");

            Assert.IsNotNull(verticals);
            Assert.AreEqual(0, verticals.Count,
                "No cells are marked as vertical connections");
        }

        // ===================================================================
        //  9. Additional coverage: IsStair / IsElevator on IndoorSpace
        // ===================================================================

        [Test]
        public void IsElevator_TrueForBoundaryConnectingElevatorCells()
        {
            BuildTestIndoorSpace();

            CellBoundary vertBoundary = indoorSpace.GetCellBoundaryFromId("cb_vert");
            Assert.IsNotNull(vertBoundary);

            Assert.IsTrue(indoorSpace.IsElevator(vertBoundary),
                "cb_vert connects elevator cells, so IsElevator should be true");
            Assert.IsFalse(indoorSpace.IsStair(vertBoundary),
                "cb_vert connects elevator cells, not stairs");
        }

        [Test]
        public void IsElevator_FalseForHorizontalAisleBoundary()
        {
            BuildTestIndoorSpace();

            CellBoundary horizontalBoundary = indoorSpace.GetCellBoundaryFromId("cb1");
            Assert.IsNotNull(horizontalBoundary);

            Assert.IsFalse(indoorSpace.IsElevator(horizontalBoundary),
                "cb1 connects aisle cells, not elevators");
            Assert.IsFalse(indoorSpace.IsStair(horizontalBoundary),
                "cb1 connects aisle cells, not stairs");
        }

        // ===================================================================
        //  10. GetVerticalBoundariesBetweenFloors
        // ===================================================================

        [Test]
        public void GetVerticalBoundariesBetweenFloors_FindsCorrectBoundaries()
        {
            BuildTestIndoorSpace();

            List<CellBoundary> vertBoundaries =
                indoorSpace.GetVerticalBoundariesBetweenFloors("1F", "2F");

            Assert.AreEqual(1, vertBoundaries.Count);
            Assert.AreEqual("cb_vert", vertBoundaries[0].Id);
        }

        [Test]
        public void GetVerticalBoundariesBetweenFloors_ReverseOrder_StillFinds()
        {
            BuildTestIndoorSpace();

            // Order of floor arguments should not matter
            List<CellBoundary> vertBoundaries =
                indoorSpace.GetVerticalBoundariesBetweenFloors("2F", "1F");

            Assert.AreEqual(1, vertBoundaries.Count,
                "Should find vertical boundaries regardless of floor argument order");
            Assert.AreEqual("cb_vert", vertBoundaries[0].Id);
        }

        // ===================================================================
        //  11. GetAllConnectionPoints
        // ===================================================================

        [Test]
        public void GetAllConnectionPoints_ReturnsOnePerBoundary()
        {
            BuildTestIndoorSpace();

            List<ConnectionPoint> allCps = indoorSpace.GetAllConnectionPoints();

            Assert.AreEqual(4, allCps.Count,
                "Should have one ConnectionPoint per CellBoundary (4 boundaries)");

            var cpIds = allCps.Select(cp => cp.Id).ToList();
            Assert.Contains("cb1", cpIds);
            Assert.Contains("cb2", cpIds);
            Assert.Contains("cb_vert", cpIds);
            Assert.Contains("cb3", cpIds);
        }

        // ===================================================================
        //  12. GetCellBoundaryFloorLevel
        // ===================================================================

        [Test]
        public void GetCellBoundaryFloorLevel_HorizontalBoundary_ReturnsSourceFloor()
        {
            BuildTestIndoorSpace();

            CellBoundary cb1 = indoorSpace.GetCellBoundaryFromId("cb1");
            string floorLevel = indoorSpace.GetCellBoundaryFloorLevel(cb1);

            Assert.AreEqual("1F", floorLevel,
                "Horizontal boundary between non-elevator cells returns source floor");
        }

        [Test]
        public void GetCellBoundaryFloorLevel_VerticalBoundary_ReturnsNull()
        {
            BuildTestIndoorSpace();

            CellBoundary cbVert = indoorSpace.GetCellBoundaryFromId("cb_vert");
            string floorLevel = indoorSpace.GetCellBoundaryFloorLevel(cbVert);

            Assert.IsNull(floorLevel,
                "Vertical boundary has no single floor level, should return null");
        }

        [Test]
        public void GetCellBoundaryFloorLevel_HorizontalBoundaryWithOneElevator_ReturnsNonElevatorFloor()
        {
            BuildTestIndoorSpace();

            // cb2: source=c2 (non-elevator, 1F), target=ev1 (elevator, 1F)
            CellBoundary cb2 = indoorSpace.GetCellBoundaryFromId("cb2");
            string floorLevel = indoorSpace.GetCellBoundaryFloorLevel(cb2);

            Assert.AreEqual("1F", floorLevel,
                "When one side is an elevator, should return the non-elevator cell's floor");
        }
    }
}
