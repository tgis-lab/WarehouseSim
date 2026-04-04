using System;
using System.Collections.Generic;
using System.Text.RegularExpressions;
using CentralControl;
using CentralControl.OrderAssignment;
using CentralControl.PickingQueueControl;
using Map;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.TestTools;

namespace EditModeTests
{
    public class InfrastructureTests
    {
        // ===================================================================
        //  W1-T1: SafeEventInvoker exception isolation
        // ===================================================================

        [Test]
        public void SafeEventInvoker_ContinuesAfterHandlerThrows()
        {
            bool firstCalled = false;
            bool thirdCalled = false;

            Action handler = null;
            handler += () => { firstCalled = true; };
            handler += () => { throw new InvalidOperationException("boom"); };
            handler += () => { thirdCalled = true; };

            LogAssert.Expect(LogType.Error, new Regex(@"\[SafeEventInvoker\].*Error invoking action"));

            SafeEventInvoker.SafeInvoke(handler);

            Assert.IsTrue(firstCalled, "First handler should have been called");
            Assert.IsTrue(thirdCalled, "Third handler should still run after second throws");
        }

        [Test]
        public void SafeEventInvoker_EventHandler_ContinuesAfterHandlerThrows()
        {
            bool firstCalled = false;
            bool thirdCalled = false;

            EventHandler<PickingEventArgs> handler = null;
            handler += (s, e) => { firstCalled = true; };
            handler += (s, e) => { throw new InvalidOperationException("boom"); };
            handler += (s, e) => { thirdCalled = true; };

            var pickingPoint = new PickingPoint("pp1", new List<string>(), "1F");
            var args = new PickingEventArgs("R1", pickingPoint, Vector3.zero);

            LogAssert.Expect(LogType.Error, new Regex(@"\[SafeEventInvoker\].*Error invoking event handler"));
            LogAssert.Expect(LogType.Error, new Regex(@"\[SafeEventInvoker\].*Inner exception"));

            SafeEventInvoker.SafeInvoke(handler, null, args);

            Assert.IsTrue(firstCalled, "First handler should have been called");
            Assert.IsTrue(thirdCalled, "Third handler should still run after second throws");
        }

        [Test]
        public void SafeEventInvoker_Action2_ContinuesAfterHandlerThrows()
        {
            bool firstCalled = false;
            bool thirdCalled = false;

            Action<string, string> handler = null;
            handler += (a, b) => { firstCalled = true; };
            handler += (a, b) => { throw new InvalidOperationException("boom"); };
            handler += (a, b) => { thirdCalled = true; };

            LogAssert.Expect(LogType.Error, new Regex(@"\[SafeEventInvoker\].*Error invoking action"));

            SafeEventInvoker.SafeInvoke(handler, "order1", "robot1");

            Assert.IsTrue(firstCalled, "First handler should have been called");
            Assert.IsTrue(thirdCalled, "Third handler should still run after second throws");
        }

        [Test]
        public void SafeEventInvoker_Action3_ContinuesAfterHandlerThrows()
        {
            bool firstCalled = false;
            bool thirdCalled = false;

            Action<string, string, string> handler = null;
            handler += (a, b, c) => { firstCalled = true; };
            handler += (a, b, c) => { throw new InvalidOperationException("boom"); };
            handler += (a, b, c) => { thirdCalled = true; };

            LogAssert.Expect(LogType.Error, new Regex(@"\[SafeEventInvoker\].*Error invoking action"));

            SafeEventInvoker.SafeInvoke(handler, "order1", "robot1", "reason");

            Assert.IsTrue(firstCalled, "First handler should have been called");
            Assert.IsTrue(thirdCalled, "Third handler should still run after second throws");
        }

        [Test]
        public void SafeEventInvoker_NullHandler_DoesNotThrow()
        {
            Assert.DoesNotThrow(() => SafeEventInvoker.SafeInvoke((Action)null));
            Assert.DoesNotThrow(() => SafeEventInvoker.SafeInvoke((Action<string, string>)null, "a", "b"));
            Assert.DoesNotThrow(() => SafeEventInvoker.SafeInvoke((Action<string, string, string>)null, "a", "b", "c"));
            Assert.DoesNotThrow(() => SafeEventInvoker.SafeInvoke<PickingEventArgs>(null, null, null));
        }

        // ===================================================================
        //  W1-T2: PickingQueueManager singleton identity
        // ===================================================================

        [Test]
        public void PickingQueueManager_Singleton_ReturnsSameInstance()
        {
            var instance1 = PickingQueueManager.Instance;
            var instance2 = PickingQueueManager.Instance;

            Assert.IsNotNull(instance1);
            Assert.AreSame(instance1, instance2, "Singleton should always return the same instance");
        }

        // ===================================================================
        //  W1-T6: OrderManager thread safety basics
        // ===================================================================

        [Test]
        public void OrderManager_GetNextOrder_ReturnsNullForEmptyList()
        {
            var go = new GameObject("OrderManagerTest");
            var manager = go.AddComponent<OrderManager>();

            var order = manager.GetNextOrder();

            Assert.IsNull(order, "GetNextOrder should return null when no orders exist");

            UnityEngine.Object.DestroyImmediate(go);
        }

        [Test]
        public void OrderManager_GetNextOrder_RemovesOrderFromPending()
        {
            var go = new GameObject("OrderManagerTest");
            var manager = go.AddComponent<OrderManager>();

            var pickingPoint = new PickingPoint("pp1", new List<string>(), "1F");
            var order = new Order("order1", Vector3.zero, pickingPoint, 5f, "1F");
            manager.AddOrder(order);

            var retrieved = manager.GetNextOrder();
            Assert.IsNotNull(retrieved, "GetNextOrder should return the added order");
            Assert.AreEqual("order1", retrieved.Id);

            var second = manager.GetNextOrder();
            Assert.IsNull(second, "GetNextOrder should return null after order was removed");

            UnityEngine.Object.DestroyImmediate(go);
        }
    }
}
