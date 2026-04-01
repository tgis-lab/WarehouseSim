using System;
using System.Collections.Generic;
using CentralControl.PickingQueueControl;
using Map;
using NetTopologySuite.Geometries;
using UnityEngine.TestTools;
using NUnit.Framework;
using PathPlanning;

namespace EditModeTests
{
    /// <summary>
    /// Edit Mode tests for Astar_Basic, PriorityQueue, and PickingQueueManager.
    /// </summary>
    public class AlgorithmQueueTests
    {
        private GeometryFactory gf;
        private PickingQueueManager queueManager;

        [SetUp]
        public void SetUp()
        {
            gf = new GeometryFactory();
            queueManager = PickingQueueManager.Instance;
            queueManager.Reset();
        }

        // ===================================================================
        //  Graph / RoutePoint helper methods for Astar_Basic tests
        // ===================================================================

        /// <summary>
        /// Creates a ConnectionPoint at the given (x, y) coordinates with a horizontal boundary.
        /// </summary>
        private ConnectionPoint MakeConnectionPoint(string id, double x, double y)
        {
            Geometry point = gf.CreatePoint(new Coordinate(x, y));
            var boundary = new CellBoundary(
                "cb_" + id,
                new Dictionary<string, object>(),
                "src_" + id, "tgt_" + id,
                point, null, "horizontal");
            return new ConnectionPoint(id, boundary, point, "1F", false, false);
        }

        /// <summary>
        /// Builds a small test Graph with RoutePoints connected in a chain:
        ///   A(0,0) -> B(1,0) -> C(2,0) -> D(3,0)
        /// Returns the graph and the RoutePoints for direct reference.
        /// </summary>
        private (Graph graph, RoutePoint rpA, RoutePoint rpB, RoutePoint rpC, RoutePoint rpD) BuildLinearRouteGraph()
        {
            var indoorSpace = new IndoorSpace();
            var graph = new Graph(indoorSpace);

            var cpA = MakeConnectionPoint("A", 0, 0);
            var cpB = MakeConnectionPoint("B", 1, 0);
            var cpC = MakeConnectionPoint("C", 2, 0);
            var cpD = MakeConnectionPoint("D", 3, 0);

            var rpA = new RoutePoint(cpA);
            var rpB = new RoutePoint(cpB);
            var rpC = new RoutePoint(cpC);
            var rpD = new RoutePoint(cpD);

            // Wire up Children (forward edges) used by Astar_Basic
            rpA.AddChild(cpB);
            rpB.AddChild(cpC);
            rpC.AddChild(cpD);

            // Also add reverse edges so the graph is bidirectional
            rpB.AddChild(cpA);
            rpC.AddChild(cpB);
            rpD.AddChild(cpC);

            graph.AddNode(rpA);
            graph.AddNode(rpB);
            graph.AddNode(rpC);
            graph.AddNode(rpD);

            return (graph, rpA, rpB, rpC, rpD);
        }

        /// <summary>
        /// Builds a disconnected graph where node E has no edges to the A-B-C-D chain.
        /// </summary>
        private (Graph graph, RoutePoint rpA, RoutePoint rpE) BuildDisconnectedGraph()
        {
            var indoorSpace = new IndoorSpace();
            var graph = new Graph(indoorSpace);

            var cpA = MakeConnectionPoint("A", 0, 0);
            var cpE = MakeConnectionPoint("E", 100, 100);

            var rpA = new RoutePoint(cpA);
            var rpE = new RoutePoint(cpE);
            // No children linking A to E

            graph.AddNode(rpA);
            graph.AddNode(rpE);

            return (graph, rpA, rpE);
        }

        // ===================================================================
        //  Astar_Basic tests
        // ===================================================================

        [Test]
        public void AstarBasic_FindsPath_BetweenConnectedNodes()
        {
            var (graph, rpA, rpB, rpC, rpD) = BuildLinearRouteGraph();

            var (path, speeds) = Astar_Basic.AstarAlgorithm(
                graph, rpA, rpD, 1.0f, SearchMethod.Euclidean_Distance);

            Assert.IsNotNull(path);
            Assert.IsTrue(path.Count > 0, "Path should contain at least one ConnectionPoint");
            Assert.AreEqual("A", path[0].Id, "Path should start at A");
            Assert.AreEqual("D", path[path.Count - 1].Id, "Path should end at D");
            Assert.AreEqual(path.Count, speeds.Count, "Speeds list should match path length");
        }

        [Test]
        public void AstarBasic_ReturnsEmptyPath_ForDisconnectedNodes()
        {
            var (graph, rpA, rpE) = BuildDisconnectedGraph();

            var (path, speeds) = Astar_Basic.AstarAlgorithm(
                graph, rpA, rpE, 1.0f, SearchMethod.Euclidean_Distance);

            Assert.IsNotNull(path);
            Assert.AreEqual(0, path.Count, "Path should be empty when nodes are disconnected");
            Assert.AreEqual(0, speeds.Count);
        }

        [Test]
        public void AstarBasic_ReturnsEmptyPath_ForNullInputs()
        {
            var (graph, rpA, _, _, _) = BuildLinearRouteGraph();

            // Expect the error logs that A* emits for invalid inputs
            LogAssert.Expect(UnityEngine.LogType.Error, "Invalid input parameters for A* algorithm");
            LogAssert.Expect(UnityEngine.LogType.Error, "Invalid input parameters for A* algorithm");
            LogAssert.Expect(UnityEngine.LogType.Error, "Invalid input parameters for A* algorithm");

            // Null graph
            var (path1, speeds1) = Astar_Basic.AstarAlgorithm(
                null, rpA, rpA, 1.0f, SearchMethod.Euclidean_Distance);
            Assert.AreEqual(0, path1.Count, "Null graph should return empty path");

            // Null start
            var (path2, speeds2) = Astar_Basic.AstarAlgorithm(
                graph, null, rpA, 1.0f, SearchMethod.Euclidean_Distance);
            Assert.AreEqual(0, path2.Count, "Null start should return empty path");

            // Null end
            var (path3, speeds3) = Astar_Basic.AstarAlgorithm(
                graph, rpA, null, 1.0f, SearchMethod.Euclidean_Distance);
            Assert.AreEqual(0, path3.Count, "Null end should return empty path");
        }

        [Test]
        public void AstarBasic_HeuristicCalculation_IsCorrect()
        {
            // Verify indirectly by comparing Euclidean vs Manhattan on the same graph.
            // For a straight horizontal line, both heuristics produce identical paths.
            var (graph, rpA, rpB, rpC, rpD) = BuildLinearRouteGraph();

            var (pathEuclid, _) = Astar_Basic.AstarAlgorithm(
                graph, rpA, rpD, 1.0f, SearchMethod.Euclidean_Distance);
            var (pathManhattan, _) = Astar_Basic.AstarAlgorithm(
                graph, rpA, rpD, 1.0f, SearchMethod.Manhattan_Distance);

            Assert.AreEqual(pathEuclid.Count, pathManhattan.Count,
                "Both heuristics should find a path of the same length on a linear graph");

            // For the linear A(0,0)->D(3,0) graph, Euclidean distance = 3.0, Manhattan = 3.0
            // Verify both arrive at the same start/end
            Assert.AreEqual("A", pathEuclid[0].Id);
            Assert.AreEqual("D", pathEuclid[pathEuclid.Count - 1].Id);
            Assert.AreEqual("A", pathManhattan[0].Id);
            Assert.AreEqual("D", pathManhattan[pathManhattan.Count - 1].Id);

            // Additionally, on a straight line the heuristic values should be equal:
            // Euclidean(0,0 -> 3,0) = sqrt(9) = 3
            // Manhattan(0,0 -> 3,0) = |0-3| + |0-0| = 3
            // Both yield 3, so path costs are identical and path lengths must match.
            for (int i = 0; i < pathEuclid.Count; i++)
            {
                Assert.AreEqual(pathEuclid[i].Id, pathManhattan[i].Id,
                    $"Path nodes should match at index {i}");
            }
        }

        // ===================================================================
        //  PriorityQueue tests
        // ===================================================================

        [Test]
        public void PriorityQueue_Enqueue_Dequeue_ReturnsMinElement()
        {
            var pq = new PriorityQueue<string>();
            pq.Enqueue("high", 10f);
            pq.Enqueue("low", 1f);
            pq.Enqueue("medium", 5f);

            Assert.AreEqual("low", pq.Dequeue(), "Dequeue should return element with lowest priority");
            Assert.AreEqual("medium", pq.Dequeue());
            Assert.AreEqual("high", pq.Dequeue());
        }

        [Test]
        public void PriorityQueue_Count_TracksCorrectly()
        {
            var pq = new PriorityQueue<int>();

            Assert.AreEqual(0, pq.Count, "Empty queue should have count 0");

            pq.Enqueue(1, 1f);
            Assert.AreEqual(1, pq.Count);

            pq.Enqueue(2, 2f);
            Assert.AreEqual(2, pq.Count);

            pq.Dequeue();
            Assert.AreEqual(1, pq.Count, "Count should decrease after Dequeue");

            pq.Dequeue();
            Assert.AreEqual(0, pq.Count, "Count should be 0 after all elements dequeued");
        }

        [Test]
        public void PriorityQueue_Contains_ReturnsTrueForEnqueuedItems()
        {
            var pq = new PriorityQueue<string>();
            pq.Enqueue("alpha", 1f);
            pq.Enqueue("beta", 2f);

            Assert.IsTrue(pq.Contains("alpha"), "Should contain enqueued item 'alpha'");
            Assert.IsTrue(pq.Contains("beta"), "Should contain enqueued item 'beta'");
            Assert.IsFalse(pq.Contains("gamma"), "Should not contain item never enqueued");

            pq.Dequeue(); // removes "alpha"
            Assert.IsFalse(pq.Contains("alpha"), "Should not contain dequeued item");
        }

        [Test]
        public void PriorityQueue_UpdatePriority_ChangesElementPosition()
        {
            var pq = new PriorityQueue<string>();
            pq.Enqueue("first", 1f);
            pq.Enqueue("second", 5f);
            pq.Enqueue("third", 10f);

            // "third" starts with highest priority (10). Lower it to be the new minimum.
            pq.UpdatePriority("third", 0.5f);

            Assert.AreEqual("third", pq.Dequeue(),
                "After UpdatePriority to 0.5, 'third' should be dequeued first");
            Assert.AreEqual("first", pq.Dequeue());
            Assert.AreEqual("second", pq.Dequeue());
        }

        // ===================================================================
        //  PickingQueueManager tests
        // ===================================================================

        // queueManager is initialized in SetUp()

        [Test]
        public void PickingQueue_RegisterArrival_AddsRobotToQueue()
        {
            bool result = queueManager.RegisterArrival("PP1", "Robot1");

            Assert.IsTrue(result, "First robot should be allowed to pick immediately");
            Assert.IsTrue(queueManager.IsRobotInQueueOrActive("PP1", "Robot1"),
                "Robot1 should be tracked as active or in queue at PP1");
        }

        [Test]
        public void PickingQueue_RegisterArrival_FirstRobotBecomesActive()
        {
            queueManager.RegisterArrival("PP1", "Robot1");

            Assert.IsTrue(queueManager.IsRobotActive("PP1", "Robot1"),
                "First arriving robot should be the active robot at the picking point");
        }

        [Test]
        public void PickingQueue_PickingCompleted_AdvancesQueue()
        {
            // Robot1 arrives and starts picking
            queueManager.RegisterArrival("PP1", "Robot1");
            // Robot2 arrives and queues
            queueManager.RegisterArrival("PP1", "Robot2");

            string promotedRobot = null;
            queueManager.OnRobotAllowedToPick += (robotId) => promotedRobot = robotId;

            // Robot1 finishes picking
            queueManager.PickingCompleted("PP1", "Robot1");

            Assert.AreEqual("Robot2", promotedRobot,
                "Robot2 should receive OnRobotAllowedToPick after Robot1 completes");
            Assert.IsTrue(queueManager.IsRobotActive("PP1", "Robot2"),
                "Robot2 should now be the active robot");
            Assert.IsFalse(queueManager.IsRobotActive("PP1", "Robot1"),
                "Robot1 should no longer be active after completing");
        }

        [Test]
        public void PickingQueue_RegisterArrival_SecondRobotWaits()
        {
            // Robot1 arrives first and starts picking
            bool firstResult = queueManager.RegisterArrival("PP1", "Robot1");
            // Robot2 arrives second and must wait
            bool secondResult = queueManager.RegisterArrival("PP1", "Robot2");

            Assert.IsTrue(firstResult, "First robot should pick immediately");
            Assert.IsFalse(secondResult, "Second robot should be told to wait");
            Assert.IsTrue(queueManager.IsRobotActive("PP1", "Robot1"),
                "Robot1 should be active");
            Assert.IsFalse(queueManager.IsRobotActive("PP1", "Robot2"),
                "Robot2 should not be active (waiting in queue)");
            Assert.IsTrue(queueManager.IsRobotInQueueOrActive("PP1", "Robot2"),
                "Robot2 should still be tracked in the queue");
        }
    }
}
