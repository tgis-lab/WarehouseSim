using System;
using System.Collections.Generic;
using UnityEngine;

namespace CentralControl
{
    /// <summary>
    /// Thread-safe event invocation with automatic cleanup of invalid listeners.
    /// Ensures events are only invoked on the main Unity thread.
    /// </summary>
    public static class SafeEventInvoker
    {
        private static readonly int mainThreadId = System.Threading.Thread.CurrentThread.ManagedThreadId;
        private static readonly Queue<Action> mainThreadQueue = new Queue<Action>();
        private static readonly object queueLock = new object();

        public static bool IsMainThread => System.Threading.Thread.CurrentThread.ManagedThreadId == mainThreadId;

        public static void InvokeOnMainThread(Action action)
        {
            if (action == null) return;

            if (IsMainThread)
            {
                action();
            }
            else
            {
                lock (queueLock)
                {
                    mainThreadQueue.Enqueue(action);
                }
            }
        }

        public static void ProcessMainThreadQueue()
        {
            if (!IsMainThread) return;

            Action[] actions;
            lock (queueLock)
            {
                if (mainThreadQueue.Count == 0) return;
                actions = mainThreadQueue.ToArray();
                mainThreadQueue.Clear();
            }

            foreach (var action in actions)
            {
                try
                {
                    action?.Invoke();
                }
                catch (Exception ex)
                {
                    Debug.LogError($"[SafeEventInvoker] Error executing queued action: {ex.Message}");
                }
            }
        }

        public static void SafeInvoke<T>(EventHandler<T> handler, object sender, T args) where T : EventArgs
        {
            if (handler == null) return;

            if (!IsMainThread)
            {
                InvokeOnMainThread(() => SafeInvoke(handler, sender, args));
                return;
            }

            Delegate[] delegates = handler.GetInvocationList();
            List<Delegate> invalidDelegates = null;

            foreach (Delegate del in delegates)
            {
                try
                {
                    if (del.Target is UnityEngine.Object unityObj)
                    {
                        if (unityObj == null)
                        {
                            invalidDelegates ??= new List<Delegate>();
                            invalidDelegates.Add(del);
                            continue;
                        }
                    }

                    del.DynamicInvoke(sender, args);
                }
                catch (Exception ex)
                {
                    Debug.LogError($"[SafeEventInvoker] Error invoking event handler: {ex.Message}");
                    
                    if (ex.InnerException != null)
                    {
                        Debug.LogError($"[SafeEventInvoker] Inner exception: {ex.InnerException.Message}");
                    }
                }
            }

            if (invalidDelegates != null)
            {
                Debug.LogWarning($"[SafeEventInvoker] Detected {invalidDelegates.Count} invalid listeners, cleanup recommended");
            }
        }

        public static void SafeInvoke(Action handler)
        {
            if (handler == null) return;

            if (!IsMainThread)
            {
                InvokeOnMainThread(() => SafeInvoke(handler));
                return;
            }

            Delegate[] delegates = handler.GetInvocationList();

            foreach (Delegate del in delegates)
            {
                try
                {
                    if (del.Target is UnityEngine.Object unityObj && unityObj == null)
                    {
                        continue;
                    }

                    del.DynamicInvoke();
                }
                catch (Exception ex)
                {
                    Debug.LogError($"[SafeEventInvoker] Error invoking action: {ex.Message}");
                }
            }
        }

        public static void SafeInvoke<T1, T2>(Action<T1, T2> handler, T1 arg1, T2 arg2)
        {
            if (handler == null) return;

            if (!IsMainThread)
            {
                InvokeOnMainThread(() => SafeInvoke(handler, arg1, arg2));
                return;
            }

            Delegate[] delegates = handler.GetInvocationList();

            foreach (Delegate del in delegates)
            {
                try
                {
                    if (del.Target is UnityEngine.Object unityObj && unityObj == null)
                    {
                        continue;
                    }

                    del.DynamicInvoke(arg1, arg2);
                }
                catch (Exception ex)
                {
                    Debug.LogError($"[SafeEventInvoker] Error invoking action: {ex.Message}");
                }
            }
        }

        public static void SafeInvoke<T1, T2, T3>(Action<T1, T2, T3> handler, T1 arg1, T2 arg2, T3 arg3)
        {
            if (handler == null) return;

            if (!IsMainThread)
            {
                InvokeOnMainThread(() => SafeInvoke(handler, arg1, arg2, arg3));
                return;
            }

            Delegate[] delegates = handler.GetInvocationList();

            foreach (Delegate del in delegates)
            {
                try
                {
                    if (del.Target is UnityEngine.Object unityObj && unityObj == null)
                    {
                        continue;
                    }

                    del.DynamicInvoke(arg1, arg2, arg3);
                }
                catch (Exception ex)
                {
                    Debug.LogError($"[SafeEventInvoker] Error invoking action: {ex.Message}");
                }
            }
        }

        public static EventHandler<T> CleanupInvalidListeners<T>(EventHandler<T> handler) where T : EventArgs
        {
            if (handler == null) return null;

            Delegate[] delegates = handler.GetInvocationList();
            EventHandler<T> cleanedHandler = null;
            int removedCount = 0;

            foreach (Delegate del in delegates)
            {
                bool isValid = true;

                if (del.Target is UnityEngine.Object unityObj && unityObj == null)
                {
                    isValid = false;
                    removedCount++;
                }

                if (isValid)
                {
                    cleanedHandler = (EventHandler<T>)Delegate.Combine(cleanedHandler, del);
                }
            }

            if (removedCount > 0)
            {
                Debug.Log($"[SafeEventInvoker] Cleaned up {removedCount} invalid listeners");
            }

            return cleanedHandler;
        }
    }
}
