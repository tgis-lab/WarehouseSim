using System.Collections.Generic;
using System.Linq;
using Map;
using NetTopologySuite.Geometries;
using Newtonsoft.Json.Linq;
using NUnit.Framework;

namespace EditModeTests
{
    /// <summary>
    /// Edit Mode tests for the Map Schema classes: CellSpace, CellBoundary,
    /// ConnectionPoint, Layer, Container, RLineGroup, and Hypergraph.
    /// </summary>
    public class MapSchemaTests
    {
        private GeometryFactory gf;

        // ==================== Helpers ====================

        [SetUp]
        public void SetUp()
        {
            gf = new GeometryFactory();
        }

        /// <summary>
        /// Creates a basic CellSpace with the given properties dictionary.
        /// Node is a Point at (x, y); Space is a small square polygon around it.
        /// </summary>
        private CellSpace MakeCell(string id, Dictionary<string, object> props,
                                   double x = 0, double y = 0)
        {
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
        /// Builds a properties dictionary with verticalFeatures in the JObject
        /// format that CellSpace.GetVerticalFeatures() can deserialise.
        /// </summary>
        private Dictionary<string, object> MakePropsWithVerticalFeatures(
            string verticalType, string locatedFloor,
            List<string> connectedFloors = null, bool navigable = true)
        {
            var props = new Dictionary<string, object> { { "navigable", navigable } };

            var associations = new JArray();
            if (!string.IsNullOrEmpty(locatedFloor))
            {
                associations.Add(new JObject
                {
                    ["floorLevel"] = locatedFloor,
                    ["associationType"] = "located"
                });
            }
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

            props["verticalFeatures"] = new JObject
            {
                ["verticalType"] = verticalType,
                ["floorAssociation"] = associations
            };

            return props;
        }

        /// <summary>
        /// Builds a properties dictionary with a "functions" JArray containing
        /// one or more function objects with given types and optional features.
        /// </summary>
        private Dictionary<string, object> MakePropsWithFunctions(
            bool navigable, params (string type, JObject features)[] functions)
        {
            var props = new Dictionary<string, object> { { "navigable", navigable } };
            var arr = new JArray();
            foreach (var (type, features) in functions)
            {
                var funcObj = new JObject { ["type"] = type };
                if (features != null)
                    funcObj["features"] = features;
                arr.Add(funcObj);
            }
            props["functions"] = arr;
            return props;
        }

        /// <summary>
        /// Creates a horizontal CellBoundary with a point boundary at (midX, midY).
        /// </summary>
        private CellBoundary MakeHorizontalBoundary(string id, string source, string target,
                                                     double midX = 0.5, double midY = 0)
        {
            Geometry boundary = gf.CreatePoint(new Coordinate(midX, midY));
            Geometry edge = gf.CreateLineString(new[]
            {
                new Coordinate(midX - 0.5, midY),
                new Coordinate(midX + 0.5, midY)
            });
            return new CellBoundary(id, new Dictionary<string, object>(),
                source, target, boundary, edge, "horizontal");
        }

        /// <summary>
        /// Creates a vertical CellBoundary (no geometry).
        /// </summary>
        private CellBoundary MakeVerticalBoundary(string id, string source, string target)
        {
            return new CellBoundary(id, new Dictionary<string, object>(),
                source, target, null, null, "vertical");
        }

        // ==================== CellSpace Tests ====================

        [Test]
        public void CellSpace_IsNavigable_ReturnsTrue_WhenNavigablePropertyIsTrue()
        {
            var props = new Dictionary<string, object> { { "navigable", true } };
            var cell = MakeCell("c1", props);

            Assert.IsTrue(cell.IsNavigable());
        }

        [Test]
        public void CellSpace_IsNavigable_ReturnsFalse_WhenNavigablePropertyIsFalse()
        {
            var props = new Dictionary<string, object> { { "navigable", false } };
            var cell = MakeCell("c2", props);

            Assert.IsFalse(cell.IsNavigable());
        }

        [Test]
        public void CellSpace_IsNavigable_ReturnsFalse_WhenNavigablePropertyMissing()
        {
            var props = new Dictionary<string, object>();
            var cell = MakeCell("c3", props);

            Assert.IsFalse(cell.IsNavigable());
        }

        [Test]
        public void CellSpace_GetLocatedFloor_ReturnsCorrectFloor()
        {
            var props = MakePropsWithVerticalFeatures(null, "2F");
            var cell = MakeCell("c4", props);

            Assert.AreEqual("2F", cell.GetLocatedFloor());
        }

        [Test]
        public void CellSpace_GetLocatedFloor_ReturnsNull_WhenNoVerticalFeatures()
        {
            var props = new Dictionary<string, object> { { "navigable", true } };
            var cell = MakeCell("c5", props);

            Assert.IsNull(cell.GetLocatedFloor());
        }

        [Test]
        public void CellSpace_IsElevator_ReturnsTrue_WhenVerticalTypeIsElevator()
        {
            var props = MakePropsWithVerticalFeatures("elevator", "1F",
                new List<string> { "2F" });
            var cell = MakeCell("ev1", props);

            Assert.IsTrue(cell.IsElevator());
            Assert.IsFalse(cell.IsStair());
        }

        [Test]
        public void CellSpace_IsStair_ReturnsTrue_WhenVerticalTypeIsStair()
        {
            var props = MakePropsWithVerticalFeatures("stair", "1F",
                new List<string> { "2F" });
            var cell = MakeCell("st1", props);

            Assert.IsTrue(cell.IsStair());
            Assert.IsFalse(cell.IsElevator());
        }

        [Test]
        public void CellSpace_IsElevatorAndIsStair_ReturnFalse_WhenNoVerticalFeatures()
        {
            var props = new Dictionary<string, object> { { "navigable", true } };
            var cell = MakeCell("c6", props);

            Assert.IsFalse(cell.IsElevator());
            Assert.IsFalse(cell.IsStair());
        }

        [Test]
        public void CellSpace_GetBoundings_ReturnsListOfBoundings_ForPickingPoint()
        {
            var features = new JObject
            {
                ["boundings"] = new JArray { "shelf_A", "shelf_B" }
            };
            var props = MakePropsWithFunctions(true, ("picking point", features));
            var cell = MakeCell("pp1", props);

            var boundings = cell.GetBoundings();
            Assert.IsNotNull(boundings);
            Assert.AreEqual(2, boundings.Count);
            Assert.Contains("shelf_A", boundings);
            Assert.Contains("shelf_B", boundings);
        }

        [Test]
        public void CellSpace_GetBoundings_ReturnsNull_WhenNoPickingPointFunction()
        {
            var props = MakePropsWithFunctions(true, ("aisle", null));
            var cell = MakeCell("a1", props);

            Assert.IsNull(cell.GetBoundings());
        }

        [Test]
        public void CellSpace_GetBoundings_ReturnsNull_WhenNoFunctionsProperty()
        {
            var props = new Dictionary<string, object> { { "navigable", true } };
            var cell = MakeCell("c7", props);

            Assert.IsNull(cell.GetBoundings());
        }

        [Test]
        public void CellSpace_HasFunctionOfType_DetectsAisle()
        {
            var props = MakePropsWithFunctions(true, ("aisle", null));
            var cell = MakeCell("a2", props);

            // HasFunctionOfType is private; test via the public methods it backs
            Assert.IsFalse(cell.IsPickingPoint());
            Assert.IsFalse(cell.IsBusinesspoint());
            Assert.IsFalse(cell.IsQueue());
            Assert.IsFalse(cell.IsPackingPoint());
        }

        [Test]
        public void CellSpace_HasFunctionOfType_DetectsPickingPoint()
        {
            var features = new JObject { ["boundings"] = new JArray { "s1" } };
            var props = MakePropsWithFunctions(true, ("picking point", features));
            var cell = MakeCell("pp2", props);

            Assert.IsTrue(cell.IsPickingPoint());
            Assert.IsFalse(cell.IsBusinesspoint());
            Assert.IsFalse(cell.IsPackingPoint());
        }

        [Test]
        public void CellSpace_HasFunctionOfType_DetectsQueueArea()
        {
            var props = MakePropsWithFunctions(true, ("queue area", null));
            var cell = MakeCell("q1", props);

            Assert.IsTrue(cell.IsQueue());
            Assert.IsFalse(cell.IsPickingPoint());
        }

        [Test]
        public void CellSpace_HasFunctionOfType_DetectsPackingPoint()
        {
            var props = MakePropsWithFunctions(true, ("packing point", null));
            var cell = MakeCell("pk1", props);

            Assert.IsTrue(cell.IsPackingPoint());
            Assert.IsFalse(cell.IsPickingPoint());
        }

        [Test]
        public void CellSpace_HasFunctionOfType_DetectsShelf()
        {
            var props = MakePropsWithFunctions(false, ("shelf", null));
            var cell = MakeCell("sh1", props);

            Assert.IsTrue(cell.IsBusinesspoint());
        }

        [Test]
        public void CellSpace_HasFunctionOfType_CaseInsensitive()
        {
            var props = MakePropsWithFunctions(true, ("Picking Point", null));
            var cell = MakeCell("pp3", props);

            // HasFunctionOfType uses StringComparison.OrdinalIgnoreCase
            Assert.IsTrue(cell.IsPickingPoint());
        }

        [Test]
        public void CellSpace_GetConnectedFloors_ReturnsCorrectFloors()
        {
            var props = MakePropsWithVerticalFeatures("elevator", "1F",
                new List<string> { "2F", "3F" });
            var cell = MakeCell("ev2", props);

            var floors = cell.GetConnectedFloors();
            Assert.AreEqual(2, floors.Count);
            Assert.Contains("2F", floors);
            Assert.Contains("3F", floors);
        }

        [Test]
        public void CellSpace_IsLocatedOnFloor_MatchesCorrectly()
        {
            var props = MakePropsWithVerticalFeatures(null, "B1");
            var cell = MakeCell("b1cell", props);

            Assert.IsTrue(cell.IsLocatedOnFloor("B1"));
            Assert.IsFalse(cell.IsLocatedOnFloor("1F"));
        }

        [Test]
        public void CellSpace_IsVerticalConnection_ReturnsTrueWhenPropertySet()
        {
            var props = new Dictionary<string, object>
            {
                { "navigable", true },
                { "isVerticalConnection", true }
            };
            var cell = MakeCell("vc1", props);

            Assert.IsTrue(cell.IsVerticalConnection());
        }

        [Test]
        public void CellSpace_IsVerticalConnection_ReturnsFalseWhenPropertyNotSet()
        {
            var props = new Dictionary<string, object> { { "navigable", true } };
            var cell = MakeCell("c8", props);

            Assert.IsFalse(cell.IsVerticalConnection());
        }

        [Test]
        public void CellSpace_StatusDefaultsToFree()
        {
            var props = new Dictionary<string, object>();
            var cell = MakeCell("c9", props);

            Assert.AreEqual(CellSpaceStatus.Free, cell.GetStatus());
        }

        [Test]
        public void CellSpace_SetStatus_ChangesStatus()
        {
            var props = new Dictionary<string, object>();
            var cell = MakeCell("c10", props);

            cell.SetStatus(CellSpaceStatus.OccupiedByFacilities);
            Assert.AreEqual(CellSpaceStatus.OccupiedByFacilities, cell.GetStatus());
        }

        // ==================== CellBoundary Tests ====================

        [Test]
        public void CellBoundary_IsHorizontal_ReturnsTrue_ForHorizontalTransition()
        {
            var cb = MakeHorizontalBoundary("cb1", "A", "B");

            Assert.IsTrue(cb.IsHorizontal());
            Assert.IsFalse(cb.IsVertical());
            Assert.AreEqual(TransitionType.Horizontal, cb.TransitionType);
        }

        [Test]
        public void CellBoundary_IsVertical_ReturnsTrue_ForVerticalTransition()
        {
            var cb = MakeVerticalBoundary("cb2", "EV1", "EV2");

            Assert.IsTrue(cb.IsVertical());
            Assert.IsFalse(cb.IsHorizontal());
            Assert.AreEqual(TransitionType.Vertical, cb.TransitionType);
        }

        [Test]
        public void CellBoundary_DefaultsToHorizontal_WhenTransitionTypeIsNull()
        {
            var cb = new CellBoundary("cb3", new Dictionary<string, object>(),
                "X", "Y", null, null, null);

            Assert.IsTrue(cb.IsHorizontal());
            Assert.AreEqual(TransitionType.Horizontal, cb.TransitionType);
        }

        [Test]
        public void CellBoundary_DefaultsToHorizontal_WhenTransitionTypeIsEmpty()
        {
            var cb = new CellBoundary("cb3b", new Dictionary<string, object>(),
                "X", "Y", null, null, "");

            Assert.IsTrue(cb.IsHorizontal());
        }

        [Test]
        public void CellBoundary_SourceAndTarget_StoredCorrectly()
        {
            var cb = MakeHorizontalBoundary("cb4", "cellA", "cellB");

            Assert.AreEqual("cb4", cb.Id);
            Assert.AreEqual("cellA", cb.Source);
            Assert.AreEqual("cellB", cb.Target);
        }

        [Test]
        public void CellBoundary_ConvertToConnectionPoint_Horizontal_UsesBoundaryCentroid()
        {
            double midX = 3.0, midY = 4.0;
            var cb = MakeHorizontalBoundary("cb5", "A", "B", midX, midY);

            var cp = cb.ConvertToConnectionPoint("1F", false, false);

            Assert.IsNotNull(cp);
            Assert.AreEqual("cb5", cp.Id);
            Assert.AreEqual("1F", cp.FloorLevel);
            Assert.IsFalse(cp.IsStair);
            Assert.IsFalse(cp.IsElevator);
            Assert.AreSame(cb, cp.CellBoundary);
            // Boundary is a Point at (3, 4); its centroid is the point itself
            Assert.IsNotNull(cp.Point);
            Assert.AreEqual(midX, ((Point)cp.Point).X, 0.001);
            Assert.AreEqual(midY, ((Point)cp.Point).Y, 0.001);
        }

        [Test]
        public void CellBoundary_ConvertToConnectionPoint_Vertical_CreatesSyntheticPoint()
        {
            var cb = MakeVerticalBoundary("cb6", "EV1", "EV2");

            var cp = cb.ConvertToConnectionPoint("1F", false, true);

            Assert.IsNotNull(cp);
            Assert.AreEqual("cb6", cp.Id);
            Assert.IsTrue(cp.IsElevator);
            Assert.IsFalse(cp.IsStair);
            // Vertical transitions get a synthetic point at (0, 0)
            Assert.IsNotNull(cp.Point);
            Assert.AreEqual(0.0, ((Point)cp.Point).X, 0.001);
            Assert.AreEqual(0.0, ((Point)cp.Point).Y, 0.001);
        }

        [Test]
        public void CellBoundary_ConvertToConnectionPoint_WithStairFlag()
        {
            var cb = MakeVerticalBoundary("cb7", "ST1", "ST2");

            var cp = cb.ConvertToConnectionPoint("2F", true, false);

            Assert.IsTrue(cp.IsStair);
            Assert.IsFalse(cp.IsElevator);
            Assert.AreEqual("2F", cp.FloorLevel);
        }

        // ==================== ConnectionPoint Tests ====================

        [Test]
        public void ConnectionPoint_IsVertical_DelegatesToCellBoundary()
        {
            var vertCb = MakeVerticalBoundary("vcp1", "S", "T");
            var vertCp = vertCb.ConvertToConnectionPoint("1F", false, false);

            Assert.IsTrue(vertCp.IsVertical());
            Assert.IsFalse(vertCp.IsHorizontal());
        }

        [Test]
        public void ConnectionPoint_IsHorizontal_DelegatesToCellBoundary()
        {
            var horizCb = MakeHorizontalBoundary("hcp1", "S", "T");
            var horizCp = horizCb.ConvertToConnectionPoint("1F", false, false);

            Assert.IsTrue(horizCp.IsHorizontal());
            Assert.IsFalse(horizCp.IsVertical());
        }

        [Test]
        public void ConnectionPoint_Position_MapsXToXAndYToZ()
        {
            var cb = MakeHorizontalBoundary("pcp1", "A", "B", 5.0, 7.0);
            var cp = cb.ConvertToConnectionPoint("1F", false, false);

            // Position maps NTS Point(x, y) to Unity Vector3(x, 0, y)
            var pos = cp.Position;
            Assert.AreEqual(5.0f, pos.x, 0.01f);
            Assert.AreEqual(0.0f, pos.y, 0.01f);
            Assert.AreEqual(7.0f, pos.z, 0.01f);
        }

        // ==================== Layer Tests ====================

        [Test]
        public void Layer_IsAisle_ReturnsTrue_WhenTypeIsAisle()
        {
            var props = new Dictionary<string, object>
            {
                { "type", "aisle" },
                { "speed", 2.0f }
            };
            var layer = new Layer("l1", new List<string> { "c1", "c2" }, props);

            Assert.IsTrue(layer.IsAisle());
            Assert.IsFalse(layer.IsIntersection());
            Assert.IsFalse(layer.IsQueue());
        }

        [Test]
        public void Layer_IsIntersection_ReturnsTrue_WhenTypeIsIntersection()
        {
            var props = new Dictionary<string, object>
            {
                { "type", "intersection" },
                { "speed", 1.5f }
            };
            var layer = new Layer("l2", new List<string> { "c3" }, props);

            Assert.IsTrue(layer.IsIntersection());
            Assert.IsFalse(layer.IsAisle());
            Assert.IsFalse(layer.IsQueue());
        }

        [Test]
        public void Layer_IsQueue_ReturnsTrue_WhenTypeIsQueue()
        {
            var props = new Dictionary<string, object>
            {
                { "type", "queue" },
                { "speed", 0.5f }
            };
            var layer = new Layer("l3", new List<string> { "q1", "q2" }, props);

            Assert.IsTrue(layer.IsQueue());
            Assert.IsFalse(layer.IsAisle());
            Assert.IsFalse(layer.IsIntersection());
        }

        [Test]
        public void Layer_SpeedLimit_ReturnsCorrectValue()
        {
            var props = new Dictionary<string, object>
            {
                { "type", "aisle" },
                { "speed", 3.5f }
            };
            var layer = new Layer("l4", new List<string>(), props);

            Assert.AreEqual(3.5f, layer.SpeedLimit(), 0.001f);
        }

        [Test]
        public void Layer_SpeedLimit_ConvertsIntToFloat()
        {
            // In JSON deserialization, "speed": 2 may arrive as int or long
            var props = new Dictionary<string, object>
            {
                { "type", "aisle" },
                { "speed", 2 }
            };
            var layer = new Layer("l5", new List<string>(), props);

            Assert.AreEqual(2.0f, layer.SpeedLimit(), 0.001f);
        }

        [Test]
        public void Layer_DeaccelerationFactor_ReturnsValue_WhenPresent()
        {
            var props = new Dictionary<string, object>
            {
                { "type", "intersection" },
                { "speed", 1.0f },
                { "deaccelerationFactor", 0.6f }
            };
            var layer = new Layer("l6", new List<string>(), props);

            Assert.AreEqual(0.6f, layer.DeaccelerationFactor(), 0.001f);
        }

        [Test]
        public void Layer_DeaccelerationFactor_ReturnsOne_WhenAbsent()
        {
            var props = new Dictionary<string, object>
            {
                { "type", "aisle" },
                { "speed", 2.0f }
            };
            var layer = new Layer("l7", new List<string>(), props);

            Assert.AreEqual(1.0f, layer.DeaccelerationFactor(), 0.001f);
        }

        [Test]
        public void Layer_CellSpaces_StoresCorrectIds()
        {
            var cells = new List<string> { "c1", "c2", "c3" };
            var props = new Dictionary<string, object>
            {
                { "type", "aisle" },
                { "speed", 1.0f }
            };
            var layer = new Layer("l8", cells, props);

            Assert.AreEqual(3, layer.CellSpaces.Count);
            Assert.Contains("c1", layer.CellSpaces);
            Assert.Contains("c2", layer.CellSpaces);
            Assert.Contains("c3", layer.CellSpaces);
        }

        // ==================== Container / RLineGroup Tests ====================

        [Test]
        public void Container_ConvertToRLineGroup_GeneratesCorrectInnerEdges()
        {
            var container = new Container("r1", "cell1",
                new List<string> { "in1", "in2" },
                new List<string> { "out1", "out2" },
                null);

            var group = container.ConvertToRLineGroup();

            Assert.AreEqual("r1", group.Id);
            // 2 sources x 2 targets = 4 inner edges
            Assert.AreEqual(4, group.InnerEdges.Count);
            Assert.IsTrue(group.InnerEdges.Any(e => e[0] == "in1" && e[1] == "out1"));
            Assert.IsTrue(group.InnerEdges.Any(e => e[0] == "in1" && e[1] == "out2"));
            Assert.IsTrue(group.InnerEdges.Any(e => e[0] == "in2" && e[1] == "out1"));
            Assert.IsTrue(group.InnerEdges.Any(e => e[0] == "in2" && e[1] == "out2"));
        }

        [Test]
        public void Container_ConvertToRLineGroup_ExcludesLimitedEdges()
        {
            var limited = new List<List<string>>
            {
                new List<string> { "in1", "out2" }
            };
            var container = new Container("r2", "cell2",
                new List<string> { "in1" },
                new List<string> { "out1", "out2" },
                limited);

            var group = container.ConvertToRLineGroup();

            // 1 source x 2 targets = 2 edges, minus 1 limited = 1
            Assert.AreEqual(1, group.InnerEdges.Count);
            Assert.IsTrue(group.InnerEdges.Any(e => e[0] == "in1" && e[1] == "out1"));
            Assert.IsFalse(group.InnerEdges.Any(e => e[0] == "in1" && e[1] == "out2"));
        }

        [Test]
        public void Container_ConvertToRLineGroup_ConnectionPointsCombinesInsAndOuts()
        {
            var container = new Container("r3", "cell3",
                new List<string> { "in1", "in2" },
                new List<string> { "out1" },
                null);

            var group = container.ConvertToRLineGroup();

            Assert.AreEqual(3, group.ConnectionPoints.Count);
            Assert.Contains("in1", group.ConnectionPoints);
            Assert.Contains("in2", group.ConnectionPoints);
            Assert.Contains("out1", group.ConnectionPoints);
        }

        [Test]
        public void Container_ConvertToRLineGroup_SourcesAndTargetsPreserved()
        {
            var container = new Container("r3b", "cell3b",
                new List<string> { "in1" },
                new List<string> { "out1", "out2" },
                null);

            var group = container.ConvertToRLineGroup();

            Assert.AreEqual(1, group.Sources.Count);
            Assert.AreEqual("in1", group.Sources[0]);
            Assert.AreEqual(2, group.Targets.Count);
            Assert.Contains("out1", group.Targets);
            Assert.Contains("out2", group.Targets);
        }

        [Test]
        public void Container_ConvertToRLineGroup_EmptyIns_HandledGracefully()
        {
            var container = new Container("r4", "cell4",
                new List<string>(),
                new List<string> { "out1" },
                null);

            var group = container.ConvertToRLineGroup();

            Assert.AreEqual(0, group.InnerEdges.Count);
            Assert.AreEqual(1, group.ConnectionPoints.Count);
            Assert.Contains("out1", group.ConnectionPoints);
        }

        [Test]
        public void Container_ConvertToRLineGroup_EmptyOuts_HandledGracefully()
        {
            var container = new Container("r5", "cell5",
                new List<string> { "in1" },
                new List<string>(),
                null);

            var group = container.ConvertToRLineGroup();

            Assert.AreEqual(0, group.InnerEdges.Count);
            Assert.AreEqual(1, group.ConnectionPoints.Count);
            Assert.Contains("in1", group.ConnectionPoints);
        }

        [Test]
        public void Container_ConvertToRLineGroup_NullIns_HandledGracefully()
        {
            var container = new Container("r6", "cell6",
                null,
                new List<string> { "out1" },
                null);

            var group = container.ConvertToRLineGroup();

            Assert.AreEqual(0, group.InnerEdges.Count);
            Assert.AreEqual(1, group.ConnectionPoints.Count);
            Assert.Contains("out1", group.ConnectionPoints);
        }

        [Test]
        public void Container_ConvertToRLineGroup_NullOuts_HandledGracefully()
        {
            var container = new Container("r7", "cell7",
                new List<string> { "in1" },
                null,
                null);

            var group = container.ConvertToRLineGroup();

            Assert.AreEqual(0, group.InnerEdges.Count);
            Assert.AreEqual(1, group.ConnectionPoints.Count);
            Assert.Contains("in1", group.ConnectionPoints);
        }

        [Test]
        public void Container_ConvertToRLineGroup_BothNull_HandledGracefully()
        {
            var container = new Container("r8", "cell8", null, null, null);

            var group = container.ConvertToRLineGroup();

            Assert.AreEqual(0, group.InnerEdges.Count);
            Assert.AreEqual(0, group.ConnectionPoints.Count);
        }

        // ==================== RLineGroup Direct Tests ====================

        [Test]
        public void RLineGroup_DefaultConstructor_InitializesEmptyLists()
        {
            var group = new RLineGroup();

            Assert.AreEqual("", group.Id);
            Assert.IsNotNull(group.Sources);
            Assert.AreEqual(0, group.Sources.Count);
            Assert.IsNotNull(group.Targets);
            Assert.AreEqual(0, group.Targets.Count);
            Assert.IsNotNull(group.ConnectionPoints);
            Assert.AreEqual(0, group.ConnectionPoints.Count);
            Assert.IsNotNull(group.InnerEdges);
            Assert.AreEqual(0, group.InnerEdges.Count);
        }

        [Test]
        public void RLineGroup_AddMethods_WorkCorrectly()
        {
            var group = new RLineGroup();
            group.AddId("rg1");
            group.AddSource("s1");
            group.AddTarget("t1");
            group.AddConnectionPoint("cp1");
            group.AddInnerEdge(new List<string> { "s1", "t1" });

            Assert.AreEqual("rg1", group.Id);
            Assert.AreEqual(1, group.Sources.Count);
            Assert.AreEqual("s1", group.Sources[0]);
            Assert.AreEqual(1, group.Targets.Count);
            Assert.AreEqual("t1", group.Targets[0]);
            Assert.AreEqual(1, group.ConnectionPoints.Count);
            Assert.AreEqual("cp1", group.ConnectionPoints[0]);
            Assert.AreEqual(1, group.InnerEdges.Count);
        }

        // ==================== Hypergraph Tests ====================

        [Test]
        public void Hypergraph_GetVerticalConnectionPoints_FiltersCorrectly()
        {
            var hypergraph = new Hypergraph();

            var horizCb = MakeHorizontalBoundary("h1", "A", "B");
            var vertCb = MakeVerticalBoundary("v1", "EV1", "EV2");
            var horizCb2 = MakeHorizontalBoundary("h2", "C", "D", 2.0, 0);

            hypergraph.AddHyperNode(horizCb.ConvertToConnectionPoint("1F", false, false));
            hypergraph.AddHyperNode(vertCb.ConvertToConnectionPoint("1F", false, true));
            hypergraph.AddHyperNode(horizCb2.ConvertToConnectionPoint("1F", false, false));

            var verticals = hypergraph.GetVerticalConnectionPoints();

            Assert.AreEqual(1, verticals.Count);
            Assert.AreEqual("v1", verticals[0].Id);
        }

        [Test]
        public void Hypergraph_GetVerticalConnectionPoints_ReturnsEmpty_WhenNoneExist()
        {
            var hypergraph = new Hypergraph();

            var horizCb = MakeHorizontalBoundary("h3", "A", "B");
            hypergraph.AddHyperNode(horizCb.ConvertToConnectionPoint("1F", false, false));

            var verticals = hypergraph.GetVerticalConnectionPoints();

            Assert.AreEqual(0, verticals.Count);
        }

        [Test]
        public void Hypergraph_GetRelatedRLineGroups_FindsGroupsContainingConnectionPoint()
        {
            var hypergraph = new Hypergraph();

            // Add a connection point
            var cb = MakeHorizontalBoundary("cp_a", "A", "B");
            var cp = cb.ConvertToConnectionPoint("1F", false, false);
            hypergraph.AddHyperNode(cp);

            // Add RLineGroups - one contains "cp_a", one does not
            var group1 = new RLineGroup("g1",
                new List<string> { "cp_a" },
                new List<string> { "cp_b" },
                new List<string> { "cp_a", "cp_b" },
                new List<List<string>> { new List<string> { "cp_a", "cp_b" } });
            var group2 = new RLineGroup("g2",
                new List<string> { "cp_c" },
                new List<string> { "cp_d" },
                new List<string> { "cp_c", "cp_d" },
                new List<List<string>> { new List<string> { "cp_c", "cp_d" } });
            var group3 = new RLineGroup("g3",
                new List<string> { "cp_a" },
                new List<string> { "cp_c" },
                new List<string> { "cp_a", "cp_c" },
                new List<List<string>> { new List<string> { "cp_a", "cp_c" } });

            hypergraph.AddHyperEdge(group1);
            hypergraph.AddHyperEdge(group2);
            hypergraph.AddHyperEdge(group3);

            var related = hypergraph.GetRelatedRLineGroups(cp);

            Assert.AreEqual(2, related.Count);
            Assert.IsTrue(related.Any(g => g.Id == "g1"));
            Assert.IsTrue(related.Any(g => g.Id == "g3"));
            Assert.IsFalse(related.Any(g => g.Id == "g2"));
        }

        [Test]
        public void Hypergraph_GetRelatedRLineGroups_ReturnsEmpty_WhenNoMatch()
        {
            var hypergraph = new Hypergraph();

            var cb = MakeHorizontalBoundary("cp_lonely", "X", "Y");
            var cp = cb.ConvertToConnectionPoint("1F", false, false);
            hypergraph.AddHyperNode(cp);

            var group = new RLineGroup("g_other",
                new List<string> { "other" },
                new List<string> { "another" },
                new List<string> { "other", "another" },
                new List<List<string>>());
            hypergraph.AddHyperEdge(group);

            var related = hypergraph.GetRelatedRLineGroups(cp);

            Assert.AreEqual(0, related.Count);
        }

        [Test]
        public void Hypergraph_GetConnectionPointById_ReturnsCorrectCP()
        {
            var hypergraph = new Hypergraph();

            var cb1 = MakeHorizontalBoundary("find_me", "A", "B", 1.0, 2.0);
            var cb2 = MakeHorizontalBoundary("not_me", "C", "D", 3.0, 4.0);

            hypergraph.AddHyperNode(cb1.ConvertToConnectionPoint("1F", false, false));
            hypergraph.AddHyperNode(cb2.ConvertToConnectionPoint("2F", false, false));

            var found = hypergraph.GetConnectionPointById("find_me");

            Assert.IsNotNull(found);
            Assert.AreEqual("find_me", found.Id);
            Assert.AreEqual("1F", found.FloorLevel);
        }

        [Test]
        public void Hypergraph_GetConnectionPointById_ReturnsNull_WhenNotFound()
        {
            var hypergraph = new Hypergraph();

            var cb = MakeHorizontalBoundary("exists", "A", "B");
            hypergraph.AddHyperNode(cb.ConvertToConnectionPoint("1F", false, false));

            var found = hypergraph.GetConnectionPointById("does_not_exist");

            Assert.IsNull(found);
        }

        [Test]
        public void Hypergraph_DefaultConstructor_InitializesEmptyLists()
        {
            var hypergraph = new Hypergraph();

            Assert.IsNotNull(hypergraph.HyperNodes);
            Assert.AreEqual(0, hypergraph.HyperNodes.Count);
            Assert.IsNotNull(hypergraph.HyperEdges);
            Assert.AreEqual(0, hypergraph.HyperEdges.Count);
        }

        [Test]
        public void Hypergraph_ParameterizedConstructor_StoresLists()
        {
            var cb = MakeHorizontalBoundary("n1", "A", "B");
            var nodes = new List<ConnectionPoint>
            {
                cb.ConvertToConnectionPoint("1F", false, false)
            };
            var edges = new List<RLineGroup>
            {
                new RLineGroup("e1", new List<string>(), new List<string>(),
                    new List<string>(), new List<List<string>>())
            };

            var hypergraph = new Hypergraph(nodes, edges);

            Assert.AreEqual(1, hypergraph.HyperNodes.Count);
            Assert.AreEqual("n1", hypergraph.HyperNodes[0].Id);
            Assert.AreEqual(1, hypergraph.HyperEdges.Count);
            Assert.AreEqual("e1", hypergraph.HyperEdges[0].Id);
        }
    }
}
