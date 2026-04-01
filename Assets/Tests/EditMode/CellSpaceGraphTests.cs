using System.Collections.Generic;
using Map;
using NetTopologySuite.Geometries;
using Newtonsoft.Json.Linq;
using NUnit.Framework;

namespace EditModeTests
{
    /// <summary>
    /// Edit Mode tests for CellSpaceGraph: adjacency building, A* pathfinding,
    /// ConnectionPoint mapping, and null-safety edge cases.
    /// </summary>
    public class CellSpaceGraphTests
    {
        private IndoorSpace indoorSpace;
        private GeometryFactory gf;

        // --------------- Helper Methods ---------------

        /// <summary>
        /// Creates a navigable CellSpace with a Point node at (x, y) and a small square polygon.
        /// Optionally places it on a floor level.
        /// </summary>
        private CellSpace MakeNavigableCell(string id, double x, double y, string floorLevel = "1F")
        {
            var props = new Dictionary<string, object> { { "navigable", true } };
            AddFloorProperties(props, floorLevel);

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
        /// Creates a non-navigable CellSpace (e.g. a shelf / obstacle).
        /// </summary>
        private CellSpace MakeNonNavigableCell(string id, double x, double y, string floorLevel = "1F")
        {
            var props = new Dictionary<string, object> { { "navigable", false } };
            AddFloorProperties(props, floorLevel);

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
        /// Creates a navigable CellSpace with a null Node geometry (for null-safety tests).
        /// </summary>
        private CellSpace MakeNavigableCellNullNode(string id, string floorLevel = "1F")
        {
            var props = new Dictionary<string, object> { { "navigable", true } };
            AddFloorProperties(props, floorLevel);

            Geometry space = gf.CreatePolygon(new[]
            {
                new Coordinate(0, 0),
                new Coordinate(1, 0),
                new Coordinate(1, 1),
                new Coordinate(0, 1),
                new Coordinate(0, 0)
            });

            return new CellSpace(id, props, space, null);
        }

        /// <summary>
        /// Adds floorLevel via the verticalFeatures / FloorAssociation JObject pattern
        /// that CellSpace.GetLocatedFloor() expects.
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
        /// Creates a horizontal CellBoundary between two cells. The boundary geometry
        /// is a LineString at the midpoint between the two cell centres.
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
        /// Creates a vertical CellBoundary (elevator / stair transition between floors).
        /// </summary>
        private CellBoundary MakeVerticalBoundary(string id, string sourceId, string targetId)
        {
            return new CellBoundary(id, new Dictionary<string, object>(),
                sourceId, targetId, null, null, "vertical");
        }

        /// <summary>
        /// Builds a simple 4-cell linear graph on a single floor:
        ///   A -- B -- C -- D
        /// </summary>
        private CellSpaceGraph BuildLinearGraph()
        {
            indoorSpace = new IndoorSpace();

            indoorSpace.CellSpaces.Add(MakeNavigableCell("A", 0, 0));
            indoorSpace.CellSpaces.Add(MakeNavigableCell("B", 1, 0));
            indoorSpace.CellSpaces.Add(MakeNavigableCell("C", 2, 0));
            indoorSpace.CellSpaces.Add(MakeNavigableCell("D", 3, 0));

            indoorSpace.CellBoundaries.Add(MakeBoundary("cb_AB", "A", "B", 0.5, 0));
            indoorSpace.CellBoundaries.Add(MakeBoundary("cb_BC", "B", "C", 1.5, 0));
            indoorSpace.CellBoundaries.Add(MakeBoundary("cb_CD", "C", "D", 2.5, 0));

            return new CellSpaceGraph(indoorSpace);
        }

        /// <summary>
        /// Builds a 2-floor graph with a vertical connection:
        ///   Floor 1: A -- B -- EV1
        ///   Floor 2: EV2 -- C -- D
        ///   Vertical: EV1 -- EV2
        /// </summary>
        private CellSpaceGraph BuildCrossFloorGraph()
        {
            indoorSpace = new IndoorSpace();

            indoorSpace.CellSpaces.Add(MakeNavigableCell("A",  0, 0, "1F"));
            indoorSpace.CellSpaces.Add(MakeNavigableCell("B",  1, 0, "1F"));
            indoorSpace.CellSpaces.Add(MakeNavigableCell("EV1", 2, 0, "1F"));
            indoorSpace.CellSpaces.Add(MakeNavigableCell("EV2", 2, 0, "2F"));
            indoorSpace.CellSpaces.Add(MakeNavigableCell("C",  3, 0, "2F"));
            indoorSpace.CellSpaces.Add(MakeNavigableCell("D",  4, 0, "2F"));

            indoorSpace.CellBoundaries.Add(MakeBoundary("cb_AB",    "A",   "B",   0.5, 0));
            indoorSpace.CellBoundaries.Add(MakeBoundary("cb_BEV1",  "B",   "EV1", 1.5, 0));
            indoorSpace.CellBoundaries.Add(MakeVerticalBoundary("cb_EV", "EV1", "EV2"));
            indoorSpace.CellBoundaries.Add(MakeBoundary("cb_EV2C",  "EV2", "C",   2.5, 0));
            indoorSpace.CellBoundaries.Add(MakeBoundary("cb_CD",    "C",   "D",   3.5, 0));

            return new CellSpaceGraph(indoorSpace);
        }

        // --------------- Setup ---------------

        [SetUp]
        public void SetUp()
        {
            gf = new GeometryFactory();
            indoorSpace = null;
        }

        // ===================================================================
        //  1. BuildGraph
        // ===================================================================

        [Test]
        public void BuildGraph_CreatesCorrectNodeCount()
        {
            var graph = BuildLinearGraph();
            Assert.AreEqual(4, graph.NodeCount);
        }

        [Test]
        public void BuildGraph_ExcludesNonNavigableCells()
        {
            indoorSpace = new IndoorSpace();
            indoorSpace.CellSpaces.Add(MakeNavigableCell("A", 0, 0));
            indoorSpace.CellSpaces.Add(MakeNonNavigableCell("obstacle", 1, 0));
            indoorSpace.CellSpaces.Add(MakeNavigableCell("B", 2, 0));

            indoorSpace.CellBoundaries.Add(MakeBoundary("cb_AO", "A", "obstacle", 0.5, 0));
            indoorSpace.CellBoundaries.Add(MakeBoundary("cb_OB", "obstacle", "B", 1.5, 0));

            var graph = new CellSpaceGraph(indoorSpace);

            Assert.AreEqual(2, graph.NodeCount, "Non-navigable cells should be excluded");
            Assert.IsTrue(graph.Contains("A"));
            Assert.IsTrue(graph.Contains("B"));
            Assert.IsFalse(graph.Contains("obstacle"));
        }

        [Test]
        public void BuildGraph_BidirectionalAdjacency()
        {
            var graph = BuildLinearGraph();

            var neighborsOfA = graph.GetNeighbors("A");
            var neighborsOfB = graph.GetNeighbors("B");

            Assert.Contains("B", neighborsOfA, "A should be adjacent to B");
            Assert.Contains("A", neighborsOfB, "B should be adjacent to A");
        }

        [Test]
        public void BuildGraph_IgnoresBoundariesWithMissingCells()
        {
            indoorSpace = new IndoorSpace();
            indoorSpace.CellSpaces.Add(MakeNavigableCell("A", 0, 0));

            // Boundary references "X" which does not exist
            indoorSpace.CellBoundaries.Add(MakeBoundary("cb_AX", "A", "X", 0.5, 0));

            var graph = new CellSpaceGraph(indoorSpace);

            Assert.AreEqual(1, graph.NodeCount);
            Assert.AreEqual(0, graph.GetNeighbors("A").Count,
                "Boundary to non-existent cell should be ignored");
        }

        // ===================================================================
        //  2. FindPath
        // ===================================================================

        [Test]
        public void FindPath_ReturnsPathForConnectedCells()
        {
            var graph = BuildLinearGraph();

            List<string> path = graph.FindPath("A", "D");

            Assert.IsNotNull(path);
            Assert.AreEqual(4, path.Count, "Path A->D through linear graph should have 4 cells");
            Assert.AreEqual("A", path[0]);
            Assert.AreEqual("D", path[path.Count - 1]);
        }

        [Test]
        public void FindPath_ReturnsShortestPath()
        {
            // Build a diamond graph:  A -- B -- D
            //                          \-- C --/
            indoorSpace = new IndoorSpace();
            indoorSpace.CellSpaces.Add(MakeNavigableCell("A", 0, 0));
            indoorSpace.CellSpaces.Add(MakeNavigableCell("B", 1, 1));  // upper path
            indoorSpace.CellSpaces.Add(MakeNavigableCell("C", 1, -1)); // lower path
            indoorSpace.CellSpaces.Add(MakeNavigableCell("D", 2, 0));

            indoorSpace.CellBoundaries.Add(MakeBoundary("cb_AB", "A", "B", 0.5, 0.5));
            indoorSpace.CellBoundaries.Add(MakeBoundary("cb_AC", "A", "C", 0.5, -0.5));
            indoorSpace.CellBoundaries.Add(MakeBoundary("cb_BD", "B", "D", 1.5, 0.5));
            indoorSpace.CellBoundaries.Add(MakeBoundary("cb_CD", "C", "D", 1.5, -0.5));

            var graph = new CellSpaceGraph(indoorSpace);
            List<string> path = graph.FindPath("A", "D");

            Assert.IsNotNull(path);
            // Both routes have equal Manhattan distance; A* should return a 3-cell path
            Assert.AreEqual(3, path.Count, "Shortest path through diamond should be 3 cells");
            Assert.AreEqual("A", path[0]);
            Assert.AreEqual("D", path[path.Count - 1]);
        }

        [Test]
        public void FindPath_StartEqualsGoal_ReturnsOneElementPath()
        {
            var graph = BuildLinearGraph();

            List<string> path = graph.FindPath("B", "B");

            Assert.IsNotNull(path);
            Assert.AreEqual(1, path.Count);
            Assert.AreEqual("B", path[0]);
        }

        // ===================================================================
        //  3. FindPath cross-floor
        // ===================================================================

        [Test]
        public void FindPath_CrossFloor_IncludesVerticalTransition()
        {
            var graph = BuildCrossFloorGraph();

            List<string> path = graph.FindPath("A", "D");

            Assert.IsNotNull(path, "Cross-floor path should be found");
            Assert.IsTrue(path.Count >= 5, "Path should include cells on both floors plus the elevator nodes");
            Assert.AreEqual("A", path[0]);
            Assert.AreEqual("D", path[path.Count - 1]);

            // The path must transit through both elevator cells
            Assert.IsTrue(path.Contains("EV1"), "Path should pass through EV1");
            Assert.IsTrue(path.Contains("EV2"), "Path should pass through EV2");
        }

        // ===================================================================
        //  4. FindPath no path
        // ===================================================================

        [Test]
        public void FindPath_ReturnsNullForDisconnectedCells()
        {
            indoorSpace = new IndoorSpace();
            indoorSpace.CellSpaces.Add(MakeNavigableCell("A", 0, 0));
            indoorSpace.CellSpaces.Add(MakeNavigableCell("B", 10, 10));
            // No boundary between them

            var graph = new CellSpaceGraph(indoorSpace);

            List<string> path = graph.FindPath("A", "B");

            Assert.IsNull(path, "Should return null when no path exists");
        }

        [Test]
        public void FindPath_ReturnsNullForMissingStartOrGoal()
        {
            var graph = BuildLinearGraph();

            Assert.IsNull(graph.FindPath("MISSING", "A"),
                "Should return null when start cell is not in graph");
            Assert.IsNull(graph.FindPath("A", "MISSING"),
                "Should return null when goal cell is not in graph");
        }

        // ===================================================================
        //  5. FindPath adjacent (1-hop)
        // ===================================================================

        [Test]
        public void FindPath_AdjacentCells_Returns2CellPath()
        {
            var graph = BuildLinearGraph();

            List<string> path = graph.FindPath("A", "B");

            Assert.IsNotNull(path, "Adjacent cells should have a path");
            Assert.AreEqual(2, path.Count, "1-hop path should have exactly 2 cells");
            Assert.AreEqual("A", path[0]);
            Assert.AreEqual("B", path[1]);
        }

        // ===================================================================
        //  6. MapToConnectionPointPath
        // ===================================================================

        [Test]
        public void MapToConnectionPointPath_CorrectCount()
        {
            var graph = BuildLinearGraph();

            // Cell path: A -> B -> C -> D (4 cells, 3 transitions)
            var cellPath = new List<string> { "A", "B", "C", "D" };
            var (cpPath, speeds) = graph.MapToConnectionPointPath(cellPath, 1.0f);

            Assert.AreEqual(3, cpPath.Count,
                "4-cell path should produce 3 ConnectionPoints (one per transition)");
            Assert.AreEqual(3, speeds.Count);
        }

        [Test]
        public void MapToConnectionPointPath_ReturnsEmptyForTooShortPath()
        {
            var graph = BuildLinearGraph();

            var singleCell = new List<string> { "A" };
            var (cpPath, speeds) = graph.MapToConnectionPointPath(singleCell, 1.0f);

            Assert.AreEqual(0, cpPath.Count,
                "Single-cell path is too short to produce ConnectionPoints");
            Assert.AreEqual(0, speeds.Count);
        }

        [Test]
        public void MapToConnectionPointPath_NullInput_ReturnsEmpty()
        {
            var graph = BuildLinearGraph();

            var (cpPath, speeds) = graph.MapToConnectionPointPath(null, 1.0f);

            Assert.AreEqual(0, cpPath.Count);
            Assert.AreEqual(0, speeds.Count);
        }

        [Test]
        public void MapToConnectionPointPath_SpeedsMatchDefaultSpeed()
        {
            var graph = BuildLinearGraph();

            var cellPath = new List<string> { "A", "B", "C" };
            var (cpPath, speeds) = graph.MapToConnectionPointPath(cellPath, 2.5f);

            foreach (float speed in speeds)
            {
                Assert.AreEqual(2.5f, speed, 0.001f,
                    "All speeds should match the default speed");
            }
        }

        // ===================================================================
        //  7. HeuristicCost / EdgeCost null safety
        // ===================================================================

        [Test]
        public void FindPath_NullNodeGeometry_DoesNotThrow()
        {
            // Build a graph where one cell has a null Node geometry.
            // HeuristicCost and EdgeCost should return float.MaxValue instead of throwing.
            indoorSpace = new IndoorSpace();
            indoorSpace.CellSpaces.Add(MakeNavigableCell("A", 0, 0));
            indoorSpace.CellSpaces.Add(MakeNavigableCellNullNode("NullNode"));
            indoorSpace.CellSpaces.Add(MakeNavigableCell("B", 2, 0));

            indoorSpace.CellBoundaries.Add(MakeBoundary("cb_AN", "A", "NullNode", 0.5, 0));
            indoorSpace.CellBoundaries.Add(MakeBoundary("cb_NB", "NullNode", "B", 1.5, 0));

            var graph = new CellSpaceGraph(indoorSpace);

            // Should not throw -- the null node makes the edge cost MaxValue,
            // so A* will not find a viable path through NullNode.
            Assert.DoesNotThrow(() => graph.FindPath("A", "B"));

            // Path is still found (MaxValue cost doesn't make it unreachable, just very expensive)
            // The key guarantee is no crash — the null geometry is safely handled
            var path = graph.FindPath("A", "B");
            Assert.IsNotNull(path, "Path should still be found (MaxValue cost, not unreachable)");
            Assert.IsTrue(path.Contains("NullNode"), "Path must go through the null-geometry node (only route)");
        }

        // ===================================================================
        //  8. Contains / GetNeighbors accessors
        // ===================================================================

        [Test]
        public void Contains_ReturnsTrueForExistingCell()
        {
            var graph = BuildLinearGraph();
            Assert.IsTrue(graph.Contains("A"));
            Assert.IsTrue(graph.Contains("D"));
        }

        [Test]
        public void Contains_ReturnsFalseForMissingCell()
        {
            var graph = BuildLinearGraph();
            Assert.IsFalse(graph.Contains("MISSING"));
        }

        [Test]
        public void GetNeighbors_ReturnsEmptyListForMissingCell()
        {
            var graph = BuildLinearGraph();
            var neighbors = graph.GetNeighbors("MISSING");
            Assert.IsNotNull(neighbors);
            Assert.AreEqual(0, neighbors.Count);
        }

        [Test]
        public void GetCellSpace_ReturnsCorrectCell()
        {
            var graph = BuildLinearGraph();

            CellSpace cs = graph.GetCellSpace("B");
            Assert.IsNotNull(cs);
            Assert.AreEqual("B", cs.Id);
        }

        [Test]
        public void GetCellSpace_ReturnsNullForMissingCell()
        {
            var graph = BuildLinearGraph();
            Assert.IsNull(graph.GetCellSpace("MISSING"));
        }

        // ===================================================================
        //  9. Duplicate boundary edge deduplication
        // ===================================================================

        [Test]
        public void BuildGraph_DuplicateBoundaries_NoDuplicateNeighbors()
        {
            indoorSpace = new IndoorSpace();
            indoorSpace.CellSpaces.Add(MakeNavigableCell("A", 0, 0));
            indoorSpace.CellSpaces.Add(MakeNavigableCell("B", 1, 0));

            // Two boundaries between the same pair of cells
            indoorSpace.CellBoundaries.Add(MakeBoundary("cb1", "A", "B", 0.5, 0));
            indoorSpace.CellBoundaries.Add(MakeBoundary("cb2", "A", "B", 0.5, 0));

            var graph = new CellSpaceGraph(indoorSpace);

            Assert.AreEqual(1, graph.GetNeighbors("A").Count,
                "Duplicate boundaries should not create duplicate neighbors");
            Assert.AreEqual(1, graph.GetNeighbors("B").Count);
        }
    }
}
