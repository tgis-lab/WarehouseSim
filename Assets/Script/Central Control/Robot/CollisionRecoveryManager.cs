using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using CentralControl;

namespace CentralControl.RobotControl
{
    /// <summary>
    /// Robot-local collision recovery manager. Handles collision recovery as a predefined
    /// autonomous behavior. Communicates with central control via events only.
    /// </summary>
    public class CollisionRecoveryManager
    {
        #region Configuration
        private readonly int replanThreshold = 3;
        private readonly float minRandomWait = 0.3f;
        private readonly float maxRandomWait = 0.8f;
        private readonly float collisionHistoryTimeout = 30f;
        #endregion
        
        #region Private Fields
        private readonly RobotController owner;
        private Dictionary<string, CollisionRecord> collisionHistory = new Dictionary<string, CollisionRecord>();
        private bool isRecovering = false;

        #endregion
        
        private class CollisionRecord
        {
            public int Count;
            public float LastCollisionTime;
        }

        public CollisionRecoveryManager(RobotController ownerRobot)
        {
            owner = ownerRobot;
            
            if (EventManager.Instance != null)
            {
                EventManager.Instance.OnDeadlockDetected += HandleDeadlockDetected;
                EventManager.Instance.OnAreaCongested += HandleAreaCongested;
                EventManager.Instance.OnReplanRequested += HandleReplanRequested;
            }
        }

        public void Cleanup()
        {
            if (EventManager.Instance != null)
            {
                EventManager.Instance.OnDeadlockDetected -= HandleDeadlockDetected;
                EventManager.Instance.OnAreaCongested -= HandleAreaCongested;
                EventManager.Instance.OnReplanRequested -= HandleReplanRequested;
            }
        }
        
        #region Event Handlers
        private void HandleDeadlockDetected(object sender, DeadlockDetectedEventArgs e)
        {
            string myId = owner.Id.ToString();
            bool isInvolved = Array.Exists(e.InvolvedRobotIds, id => id == myId);
            
            if (isInvolved)
            {
                int myIndex = Array.IndexOf(e.InvolvedRobotIds, myId);
                float staggerDelay = myIndex * 0.5f;
                
                owner.StartCoroutine(HandleDeadlockResolution(staggerDelay));
            }
        }
        
        private void HandleAreaCongested(object sender, AreaCongestedEventArgs e)
        {
            string myId = owner.Id.ToString();
            bool isInCongestion = Array.Exists(e.CongestedRobotIds, id => id == myId);
            
            if (isInCongestion && e.Level >= CongestionLevel.High)
            {
                int myIndex = Array.IndexOf(e.CongestedRobotIds, myId);
                float staggerDelay = myIndex * 0.3f;
                
                owner.StartCoroutine(ConsiderAlternativeRoute(staggerDelay));
            }
        }
        
        private void HandleReplanRequested(object sender, ReplanRequestEventArgs e)
        {
            if (e.RobotId == owner.Id.ToString())
            {
                if (e.Reason == "DeadlockYield")
                {
                    owner.StartCoroutine(ExecuteDeadlockYield(e.SuggestedDelay));
                    return;
                }

                owner.StartCoroutine(ExecuteDelayedReplan(e.SuggestedDelay, e.Reason));
            }
        }
        #endregion
        
        #region Collision Recovery
        public void HandleRobotCollision(RobotController otherRobot, Vector3 collisionPosition)
        {
            if (owner == null || otherRobot == null) return;
            if (owner.IsBackingUp) return;
            
            string pairKey = GetOrderedPairKey(owner.Id, otherRobot.Id);
            
            CleanupExpiredRecords();
            
            if (!collisionHistory.TryGetValue(pairKey, out var record))
            {
                record = new CollisionRecord();
                collisionHistory[pairKey] = record;
            }
            
            record.Count++;
            record.LastCollisionTime = Time.time;
            
            ReportCollisionToCenter(otherRobot, collisionPosition);
            
            if (record.Count >= replanThreshold)
            {
                record.Count = 0;
                owner.RequestReplan("MultipleCollisions");
            }
            else
            {
                ExecuteBackupRecovery();
            }
        }
        
        public void HandleEnvironmentCollision(Vector3 obstaclePosition)
        {
            if (owner == null || owner.IsBackingUp) return;
            
            ReportEnvironmentCollisionToCenter(obstaclePosition);
            ExecuteBackupRecovery(obstaclePosition);
        }
        
        private void ReportCollisionToCenter(RobotController otherRobot, Vector3 position)
        {
            if (EventManager.Instance != null)
            {
                EventManager.Instance.ReportRobotCollision(
                    owner.Id.ToString(),
                    otherRobot.Id.ToString(),
                    otherRobot.gameObject,
                    Time.time,
                    position
                );
            }
        }
        
        private void ReportEnvironmentCollisionToCenter(Vector3 position)
        {
            if (EventManager.Instance != null)
            {
                EventManager.Instance.ReportEnvironmentCollision(
                    owner.Id.ToString(),
                    "environment",
                    null,
                    Time.time,
                    position
                );
            }
        }
        
        private void ExecuteBackupRecovery(Vector3? obstaclePosition = null)
        {
            if (isRecovering) return;
            
            Vector3 backupTarget = owner.CalculateSafeBackupTarget(obstaclePosition);
            
            if (Vector3.Distance(backupTarget, owner.transform.position) < 0.1f)
            {
                Debug.Log($"Robot {owner.Id}: No safe backup position, waiting in place");
                owner.StartCoroutine(WaitInPlaceAndReplan());
                return;
            }
            
            float randomWait = UnityEngine.Random.Range(minRandomWait, maxRandomWait);
            owner.StartBackupAndWait(backupTarget, randomWait);
        }
        
        private IEnumerator WaitInPlaceAndReplan()
        {
            isRecovering = true;
            float waitTime = UnityEngine.Random.Range(0.5f, 1.5f);
            yield return new WaitForSeconds(waitTime);
            
            isRecovering = false;
            if (owner != null && owner.CurrentOrder != null)
            {
                owner.RequestReplan("NoSafeBackupPosition");
            }
        }
        #endregion
        
        #region Deadlock Resolution
        private IEnumerator HandleDeadlockResolution(float delay)
        {
            yield return new WaitForSeconds(delay);
            
            if (owner != null && !owner.IsBackingUp)
            {
                owner.RequestReplan("DeadlockResolution");
            }
        }
        
        private IEnumerator ConsiderAlternativeRoute(float delay)
        {
            yield return new WaitForSeconds(delay);
            
            if (owner != null && owner.IsMoving && !owner.IsBackingUp)
            {
                ExecuteBackupRecovery();
            }
        }
        
        private IEnumerator ExecuteDelayedReplan(float delay, string reason)
        {
            if (delay > 0)
            {
                yield return new WaitForSeconds(delay);
            }
            
            if (owner != null)
            {
                owner.RequestReplan(reason);
            }
        }

        private IEnumerator ExecuteDeadlockYield(float delay)
        {
            if (delay > 0)
            {
                yield return new WaitForSeconds(delay);
            }

            if (owner != null && !owner.IsBackingUp)
            {
                ExecuteBackupRecovery();
            }
        }
        #endregion

        
        #region Recovery Completion Notification
        public void NotifyRecoveryCompleted(CollisionRecoveryResult result, float recoveryTime, float collisionTime, string otherRobotId = null)
        {
            if (EventManager.Instance != null && !string.IsNullOrEmpty(otherRobotId))
            {
                EventManager.Instance.TriggerCollisionRecoveryCompleted(
                    owner.Id.ToString(),
                    otherRobotId,
                    result,
                    recoveryTime,
                    collisionTime
                );
            }
        }
        #endregion
        
        #region Utility Methods
        private string GetOrderedPairKey(int id1, int id2)
        {
            return id1 < id2 ? $"{id1}_{id2}" : $"{id2}_{id1}";
        }
        
        private void CleanupExpiredRecords()
        {
            float currentTime = Time.time;
            List<string> toRemove = new List<string>();
            
            foreach (var kvp in collisionHistory)
            {
                if (currentTime - kvp.Value.LastCollisionTime > collisionHistoryTimeout)
                {
                    toRemove.Add(kvp.Key);
                }
            }
            
            foreach (var key in toRemove)
            {
                collisionHistory.Remove(key);
            }
        }
        #endregion
    }
}
