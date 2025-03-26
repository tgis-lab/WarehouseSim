using UnityEngine;
using UnityEngine.AI;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using Map;

namespace CentralControl.PickerControl
{
    public enum PickerState
    {
        Idle,
        Moving,
        Picking
    }

    public class PickerController : MonoBehaviour
    {
        #region Inspector Fields
        public int Id 
        { 
            get { return _id; } 
            private set 
            { 
                _id = value; 
                gameObject.name = $"Picker_{value}";
            } 
        }
        public float pickingTime = 5f;
        public float movementSpeed = 3f;
        public float workAreaRadius = 10f;
        #endregion

        #region Private Fields
        private int _id = -1;
        private IndoorSpace indoorSpace;
        private MapGrid mapGrid;
        private HashSet<PickerState> currentStates = new HashSet<PickerState>();
        private Vector3 currentPosition;
        private List<PickingPoint> assignedPickingPoints = new List<PickingPoint>();
        private NavMeshAgent navAgent;
        #endregion

        #region State Properties
        public Transform baseLocation;
        public bool IsIdle => currentStates.Contains(PickerState.Idle);
        public bool IsPicking => currentStates.Contains(PickerState.Picking);
        public bool IsMoving => currentStates.Contains(PickerState.Moving);
        public Vector3 CurrentPosition => transform.position;
        public int QueuedTaskCount => pickingQueue.Count;
        #endregion

        #region Picking Task Properties
        private Queue<PickingEventArgs> pickingQueue = new Queue<PickingEventArgs>();
        private bool isProcessingTasks = false;
        #endregion

        public PickerController()
        {
            _id = -1;
        }

        private void Awake()
        {
            if (EventManager.Instance != null)
            {
                EventManager.Instance.OnRobotReadyForPicking -= HandleRobotReadyForPicking;
                EventManager.Instance.OnPickingCompleted -= HandlePickingCompleted;
            }
        }

        private void OnDestroy()
        {
            if (EventManager.Instance != null)
            {
                EventManager.Instance.OnRobotReadyForPicking -= HandleRobotReadyForPicking;
                EventManager.Instance.OnPickingCompleted -= HandlePickingCompleted;
            }
        }

        #region Initialization
        public void Initialize(int id, IndoorSpace indoorSpace, MapGrid mapGrid, Vector3 basePosition)
        {
            this.Id = id;
            this.indoorSpace = indoorSpace;
            this.mapGrid = mapGrid;

            navAgent = gameObject.GetComponent<NavMeshAgent>();
            if (navAgent == null)
            {
                navAgent = gameObject.AddComponent<NavMeshAgent>();
                navAgent.radius = 0.4f;
                navAgent.height = 1.8f;
                navAgent.speed = movementSpeed;
                navAgent.angularSpeed = 120f;
                navAgent.acceleration = 8f;
                navAgent.stoppingDistance = 0.1f;
                navAgent.areaMask = NavMesh.AllAreas;
                
                // Avoidance settings
                navAgent.avoidancePriority = 50 - Id % 10;
                navAgent.obstacleAvoidanceType = ObstacleAvoidanceType.HighQualityObstacleAvoidance;

                navAgent.autoRepath = true;
                navAgent.autoBraking = true;
            }
            
            if (workAreaRadius <= 0f)
            {
                workAreaRadius = 10f;
                Debug.Log($"Picker {id}: default work area radius {workAreaRadius}");
            }
            
            if (baseLocation == null)
            {
                GameObject baseLocationObj = new GameObject($"Picker_{id}_BaseLocation");
                baseLocationObj.transform.SetParent(transform, false);
                baseLocation = baseLocationObj.transform;
            }
            
            baseLocation.position = basePosition;
            currentPosition = basePosition;
            
            SetState(PickerState.Idle, true);
            
            if (EventManager.Instance != null)
            {
                EventManager.Instance.OnRobotReadyForPicking += HandleRobotReadyForPicking;
                EventManager.Instance.OnPickingCompleted += HandlePickingCompleted;
            }
            
            Debug.Log($"Picker {Id} initialized at {basePosition}");
        }

        public void AssignPickingPoints(List<PickingPoint> pickingPoints)
        {
            assignedPickingPoints = pickingPoints;
            Debug.Log($"Picker {Id} assigned to picking points: {string.Join(", ", pickingPoints.Select(p => p.Id))}");
        }
        #endregion

        #region Picking Point Management
        public bool IsPickingPointInRange(PickingPoint pickingPoint)
        {
            if (pickingPoint == null)
            {
                Debug.LogError($"Picker {Id}: pickingPoint is null");
                return false;
            }
            
            if (indoorSpace == null)
            {
                Debug.LogError($"Picker {Id}: indoorSpace is null");

                var mapLoader = FindObjectOfType<MapLoader>();
                if (mapLoader != null)
                {
                    indoorSpace = mapLoader.indoorSpace;
                    Debug.Log($"Picker {Id}: try to recover indoorSpace - success: {indoorSpace != null}");
                }
                
                if (indoorSpace == null)
                    return false;
            }
            
            if (baseLocation == null)
            {
                Debug.LogError($"Picker {Id}: baseLocation is null");
                // Back up to current position
                return Vector3.Distance(transform.position, GetPickingPointPosition(pickingPoint)) <= workAreaRadius;
            }
            
            // Get picking point position
            Vector3 pickingPosition = GetPickingPointPosition(pickingPoint);
            
            // Calculate distance to picking point
            float distance = Vector3.Distance(baseLocation.position, pickingPosition);
            bool inRange = distance <= workAreaRadius;
            
            if (inRange)
            {
                Debug.Log($"Picker {Id}: picking point {pickingPoint.Id} in range (distance: {distance:F2}, work radius: {workAreaRadius:F2})");
            }
            else
            {
                Debug.Log($"Picker {Id}: picking point {pickingPoint.Id} out of range (distance: {distance:F2}, work radius: {workAreaRadius:F2})");
            }
            
            return inRange;
        }

        private Vector3 GetPickingPointPosition(PickingPoint pickingPoint)
        {
            try
            {
                CellSpace cellSpace = indoorSpace.GetCellSpaceFromId(pickingPoint.Id);
                if (cellSpace == null || cellSpace.Space == null || cellSpace.Space.Centroid == null)
                {
                    Debug.LogWarning($"Picker {Id}: cannot get the picking point {pickingPoint.Id} CellSpace");
                    return transform.position;
                }

                return new Vector3(
                    (float)cellSpace.Space.Centroid.X,
                    transform.position.y,
                    (float)cellSpace.Space.Centroid.Y
                );
            }
            catch (System.Exception ex)
            {
                Debug.LogError($"Picker {Id}: error getting picking point {pickingPoint.Id} position: {ex.Message}");
                return transform.position;
            }
        }
        #endregion

        #region Task management
        private void HandleRobotReadyForPicking(object sender, PickingEventArgs args)
        {
            Debug.Log($"Picker {Id} received event: Robot {args.RobotId} at {args.Location}");
            
            if (!IsPickingPointInRange(args.PickingPoint))
            {
                Debug.Log($"Picker {Id}: Picking point {args.PickingPoint.Id} is NOT in range");
                return;
            }
            Debug.Log($"Picker {Id}: Picking point {args.PickingPoint.Id} is in range, adding to queue");

            StartCoroutine(HandlePickingRequest(args));
        }

        private IEnumerator ProcessPickingTask()
        {
            isProcessingTasks = true;
            
            while (pickingQueue.Count > 0)
            {
                // 安全地获取队列首元素而不移除它
                PickingEventArgs currentTask = null;
                
                lock (pickingQueue) // 使用锁确保线程安全
                {
                    if (pickingQueue.Count > 0)
                    {
                        currentTask = pickingQueue.Peek();
                    }
                    else
                    {
                        break; // 队列为空，退出循环
                    }
                }
                
                if (currentTask == null)
                {
                    yield return null;
                    continue;
                }
                
                Debug.Log($"Picker {Id}: Processing picking task for Robot {currentTask.RobotId}");
                
                // 找到合适的拣货位置
                GameObject robotObj = GameObject.Find($"Robot_{currentTask.RobotId}");
                Vector3 targetPosition = currentTask.Location;
                
                if (robotObj != null)
                {
                    Vector3 robotPosition = robotObj.transform.position;
                    targetPosition = CalculateUniquePickingPosition(robotPosition, currentTask.PickingPoint);
                }
                
                // 移动到目标位置
                SetState(PickerState.Moving, true);
                Debug.Log($"Picker {Id} moving to robot {currentTask.RobotId} at {targetPosition}");
                yield return StartCoroutine(MoveToPosition(targetPosition));
                SetState(PickerState.Moving, false);
                
                // 拣货操作
                SetState(PickerState.Picking, true);
                Debug.Log($"Picker {Id} starting picking operation");
                yield return new WaitForSeconds(pickingTime);
                SetState(PickerState.Picking, false);
                
                // 通知系统拣货完成
                if (EventManager.Instance != null)
                {
                    EventManager.Instance.TriggerPickingCompleted(currentTask.RobotId, currentTask.PickingPoint, currentTask.Location);
                }
                
                // 等待事件处理完成，确保HandlePickingCompleted有时间处理
                yield return null;
                
                // 安全地移除任务(如果仍然是队列头部的任务)
                // 注意：这部分代码只是一个额外的安全保障，通常由HandlePickingCompleted处理
                lock (pickingQueue)
                {
                    if (pickingQueue.Count > 0 && pickingQueue.Peek().RobotId == currentTask.RobotId)
                    {
                        try
                        {
                            pickingQueue.Dequeue();
                            Debug.Log($"Picker {Id}: 任务已从队列移除: {currentTask.RobotId}");
                        }
                        catch (InvalidOperationException)
                        {
                            Debug.LogWarning($"Picker {Id}: 无法移除任务，队列可能为空");
                        }
                    }
                }
            }
            
            isProcessingTasks = false;
            Debug.Log($"Picker {Id}: All tasks completed");
        }

        private void HandlePickingCompleted(object sender, PickingEventArgs args)
        {
            lock (pickingQueue) // 使用锁确保线程安全
            {
                if (pickingQueue.Count > 0 && pickingQueue.Peek().RobotId == args.RobotId)
                {
                    Debug.Log($"Picker {Id}: Finished robot {args.RobotId}'s picking task");
                    
                    SetState(PickerState.Idle, true);
                    
                    try
                    {
                        pickingQueue.Dequeue(); // 安全移除任务
                    }
                    catch (InvalidOperationException)
                    {
                        Debug.LogWarning($"Picker {Id}: 尝试移除任务失败，队列可能为空");
                        // 继续执行，不抛出异常
                    }
                }
                
                // 检查是否有更多任务需要处理
                if (pickingQueue.Count > 0 && !isProcessingTasks)
                {
                    StartCoroutine(ProcessPickingTask());
                }
            }
        }

        public IEnumerator HandlePickingRequest(PickingEventArgs args)
        {
            lock (pickingQueue) // 使用锁确保线程安全
            {
                // 加入队列
                pickingQueue.Enqueue(args);
                Debug.Log($"Picker {Id} received picking request from Robot {args.RobotId} at position {args.Location}");
                
                // 如果没有正在处理任务，启动处理
                if (!isProcessingTasks)
                {
                    StartCoroutine(ProcessPickingTask());
                }
            }

            yield break;
        }
        # endregion

        #region Movement
        public IEnumerator MoveToPosition(Vector3 targetPosition)
        {
            // 保存MoveToPosition被调用前的状态
            bool wasMoving = IsMoving;
            
            // 确保NavMeshAgent可用
            if (navAgent == null)
            {
                Debug.LogError($"Picker {Id}: NavMeshAgent未初始化！");
                yield break;
            }
            
            // 检查NavMesh可达性
            NavMeshPath path = new NavMeshPath();
            if (!navAgent.CalculatePath(targetPosition, path) || path.status != NavMeshPathStatus.PathComplete)
            {
                Debug.LogWarning($"Picker {Id}: 无法找到到 {targetPosition} 的完整路径");
                
                // 尝试找到最近的可导航点
                NavMeshHit hit;
                if (NavMesh.SamplePosition(targetPosition, out hit, 5f, NavMesh.AllAreas))
                {
                    targetPosition = hit.position;
                    Debug.Log($"Picker {Id}: 找到替代目标位置 {targetPosition}");
                }
                else
                {
                    Debug.LogError($"Picker {Id}: 无法找到可达的替代位置");
                    yield break;
                }
            }
            
            // 设置状态 - 仅当之前未在移动时
            if (!wasMoving)
            {
                SetState(PickerState.Moving, true);
            }
            
            // 设置目标
            navAgent.SetDestination(targetPosition);
            
            // 等待到达目标
            while (!navAgent.pathPending && navAgent.remainingDistance > navAgent.stoppingDistance)
            {
                // 更新当前位置 - 与现有代码保持兼容
                currentPosition = transform.position;
                yield return null;
            }
            
            // 完成移动
            if (!wasMoving)
            {
                SetState(PickerState.Moving, false);
            }
            
            // 更新当前位置到最终位置
            currentPosition = transform.position;
            
            Debug.Log($"Picker {Id}: 已到达目标位置附近 {transform.position}");
        }

        private Vector3 CalculateUniquePickingPosition(Vector3 robotPosition, PickingPoint pickingPoint)
        {
            Vector3[] basePositions = new Vector3[]
            {
                new Vector3(0.8f, 0, 0.8f),   // 右前
                new Vector3(-0.8f, 0, 0.8f),  // 左前
                new Vector3(0.8f, 0, -0.8f),  // 右后
                new Vector3(-0.8f, 0, -0.8f), // 左后
                new Vector3(1.2f, 0, 0),      // 右侧
                new Vector3(-1.2f, 0, 0),     // 左侧
                new Vector3(0, 0, 1.2f),      // 前方
                new Vector3(0, 0, -1.2f)      // 后方
            };
            
            // 根据Picker ID选择不同位置
            int positionIndex = Id % basePositions.Length;
            Vector3 offset = basePositions[positionIndex];
            
            // 计算目标位置
            Vector3 targetPosition = robotPosition + offset;
            
            // 确保位置在NavMesh上
            NavMeshHit hit;
            if (NavMesh.SamplePosition(targetPosition, out hit, 2.0f, NavMesh.AllAreas))
            {
                return hit.position;
            }
            
            // 如果找不到有效位置，尝试其他位置
            for (int i = 0; i < basePositions.Length; i++)
            {
                if (i == positionIndex) continue; // 跳过已尝试的位置
                
                Vector3 alternatePosition = robotPosition + basePositions[i];
                if (NavMesh.SamplePosition(alternatePosition, out hit, 2.0f, NavMesh.AllAreas))
                {
                    return hit.position;
                }
            }
            
            // 如果所有位置都无效，返回机器人位置
            return robotPosition;
        }
        #endregion

        #region State Management
        private void SetState(PickerState state, bool value)
        {
            if (value)
                currentStates.Add(state);
            else
                currentStates.Remove(state);
        }
        #endregion

        #region Visualization
        private void OnDrawGizmos()
        {
            if (baseLocation != null)
            {
                // // Draw work area
                // Gizmos.color = Color.green;
                // Gizmos.DrawWireSphere(baseLocation.position, workAreaRadius);

                // // Draw current state
                // if (IsIdle)
                //     Gizmos.color = Color.gray;
                // else if (IsPicking)
                //     Gizmos.color = Color.yellow;
                // else if (IsMoving)
                //     Gizmos.color = Color.blue;

                // Gizmos.DrawWireSphere(transform.position, 0.5f);

                // Draw line to target position
                if (IsMoving)
                {
                    Gizmos.color = Color.white;
                    Gizmos.DrawLine(transform.position, CurrentPosition);
                }
            }
        }
        #endregion
    }
}