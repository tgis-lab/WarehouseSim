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
        //  NavigationGraph helpers for BasicAStarPathFinder tests
        // ===================================================================

        /// <summary>
        /// Builds a NavigationGraph with 4 Waypoints in a bidirectional chain:
        ///   A(0,0) ↔ B(1,0) ↔ C(2,0) ↔ D(3,0)
        /// Uses CellSpace waypoints for simplicity.
        /// </summary>
        private NavigationGraph BuildLinearNavGraph()
        {
            var indoorSpace = new IndoorSpace();
            indoorSpace.CellSpaces.Add(TestIndoorSpaceFactory.MakeNavigableCell("A", 0, 0));
            indoorSpace.CellSpaces.Add(TestIndoorSpaceFactory.MakeNavigableCell("B", 1, 0));
            indoorSpace.CellSpaces.Add(TestIndoorSpaceFactory.MakeNavigableCell("C", 2, 0));
            indoorSpace.CellSpaces.Add(TestIndoorSpaceFactory.MakeNavigableCell("D", 3, 0));

            // Bidirectional boundaries
            indoorSpace.CellBoundaries.Add(TestIndoorSpaceFactory.MakeHorizontalBoundary("cb_AB", "A", "B", 0.5, 0));
            indoorSpace.CellBoundaries.Add(TestIndoorSpaceFactory.MakeHorizontalBoundary("cb_BA", "B", "A", 0.5, 0));
            indoorSpace.CellBoundaries.Add(TestIndoorSpaceFactory.MakeHorizontalBoundary("cb_BC", "B", "C", 1.5, 0));
            indoorSpace.CellBoundaries.Add(TestIndoorSpaceFactory.MakeHorizontalBoundary("cb_CB", "C", "B", 1.5, 0));
            indoorSpace.CellBoundaries.Add(TestIndoorSpaceFactory.MakeHorizontalBoundary("cb_CD", "C", "D", 2.5, 0));
            indoorSpace.CellBoundaries.Add(TestIndoorSpaceFactory.MakeHorizontalBoundary("cb_DC", "D", "C", 2.5, 0));

            return NavigationGraphBuilder.FromCellSpaces(indoorSpace, 3.0f, 0.05f);
        }

        /// <summary>
        /// Builds a disconnected graph where node E has no edges to A.
        /// </summary>
        private NavigationGraph BuildDisconnectedNavGraph()
        {
            var indoorSpace = new IndoorSpace();
            indoorSpace.CellSpaces.Add(TestIndoorSpaceFactory.MakeNavigableCell("A", 0, 0));
            indoorSpace.CellSpaces.Add(TestIndoorSpaceFactory.MakeNavigableCell("E", 100, 100));
            // No boundaries = no edges
            return NavigationGraphBuilder.FromCellSpaces(indoorSpace, 3.0f, 0.05f);
        }

        // ===================================================================
        //  BasicAStarPathFinder tests (replaces old Astar_Basic tests)
        // ===================================================================

        [Test]
        public void AstarBasic_FindsPath_BetweenConnectedNodes()
        {
            var graph = BuildLinearNavGraph();
            var finder = new BasicAStarPathFinder(SearchMethod.Euclidean_Distance);

            var result = finder.FindPath(graph, graph.GetNode("A"), graph.GetNode("D"),
                null, null, "R1", null, 0f, 1f, 2f, 5f);

            Assert.IsNotNull(result);
            Assert.IsTrue(result.Path.Count > 0, "Path should contain at least one Waypoint");
            Assert.AreEqual("A", result.Path[0].Id, "Path should start at A");
            Assert.AreEqual("D", result.Path[result.Path.Count - 1].Id, "Path should end at D");
            Assert.AreEqual(result.Path.Count, result.Speeds.Count, "Speeds list should match path length");
        }

        [Test]
        public void AstarBasic_ReturnsNull_ForDisconnectedNodes()
        {
            var graph = BuildDisconnectedNavGraph();
            var finder = new BasicAStarPathFinder(SearchMethod.Euclidean_Distance);

            var result = finder.FindPath(graph, graph.GetNode("A"), graph.GetNode("E"),
                null, null, "R1", null, 0f, 1f, 2f, 5f);

            Assert.IsNull(result, "Should return null when nodes are disconnected");
        }

        [Test]
        public void AstarBasic_ReturnsNull_ForNullInputs()
        {
            var graph = BuildLinearNavGraph();
            var finder = new BasicAStarPathFinder(SearchMethod.Euclidean_Distance);

            Assert.IsNull(finder.FindPath(null, graph.GetNode("A"), graph.GetNode("A"),
                null, null, "R1", null, 0f, 1f, 2f, 5f), "Null graph should return null");
            Assert.IsNull(finder.FindPath(graph, null, graph.GetNode("A"),
                null, null, "R1", null, 0f, 1f, 2f, 5f), "Null start should return null");
            Assert.IsNull(finder.FindPath(graph, graph.GetNode("A"), null,
                null, null, "R1", null, 0f, 1f, 2f, 5f), "Null end should return null");
        }

        [Test]
        public void AstarBasic_HeuristicCalculation_IsCorrect()
        {
            var graph = BuildLinearNavGraph();
            var finderEuclid = new BasicAStarPathFinder(SearchMethod.Euclidean_Distance);
            var finderManhattan = new BasicAStarPathFinder(SearchMethod.Manhattan_Distance);

            var resultEuclid = finderEuclid.FindPath(graph, graph.GetNode("A"), graph.GetNode("D"),
                null, null, "R1", null, 0f, 1f, 2f, 5f);
            var resultManhattan = finderManhattan.FindPath(graph, graph.GetNode("A"), graph.GetNode("D"),
                null, null, "R1", null, 0f, 1f, 2f, 5f);

            Assert.AreEqual(resultEuclid.Path.Count, resultManhattan.Path.Count,
                "Both heuristics should find a path of the same length on a linear graph");

            Assert.AreEqual("A", resultEuclid.Path[0].Id);
            Assert.AreEqual("D", resultEuclid.Path[resultEuclid.Path.Count - 1].Id);

            for (int i = 0; i < resultEuclid.Path.Count; i++)
            {
                Assert.AreEqual(resultEuclid.Path[i].Id, resultManhattan.Path[i].Id,
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
