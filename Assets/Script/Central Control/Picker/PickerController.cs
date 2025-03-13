using UnityEngine;
using UnityEngine.AI;
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
        public float pickingTime = 10f;
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
                navAgent.avoidancePriority = 50; // The lower the value, the higher the priority
                navAgent.obstacleAvoidanceType = ObstacleAvoidanceType.HighQualityObstacleAvoidance;
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

            // 启动协程处理拣货请求
            StartCoroutine(HandlePickingRequest(args));
        }

        private IEnumerator ProcessPickingTask()
        {
            isProcessingTasks = true;

            while (pickingQueue.Count > 0)
            {
                PickingEventArgs currentTask = pickingQueue.Peek();
                Debug.Log($"Picker {Id}: Processing picking task for Robot {currentTask.RobotId}");
                
                // 找到合适的拣货位置 - 靠近机器人但避免碰撞
                GameObject robotObj = GameObject.Find($"Robot_{currentTask.RobotId}");
                Vector3 targetPosition = currentTask.Location;
                
                if (robotObj != null)
                {
                    // 使用NavMesh找到靠近机器人的可达位置
                    Vector3 robotPosition = robotObj.transform.position;
                    
                    // 在机器人周围采样几个点，找到最佳位置
                    Vector3[] sampleOffsets = new Vector3[]
                    {
                        new Vector3(1.2f, 0, 0),
                        new Vector3(-1.2f, 0, 0),
                        new Vector3(0, 0, 1.2f),
                        new Vector3(0, 0, -1.2f),
                        new Vector3(0.85f, 0, 0.85f),
                        new Vector3(-0.85f, 0, 0.85f),
                        new Vector3(0.85f, 0, -0.85f),
                        new Vector3(-0.85f, 0, -0.85f)
                    };
                    
                    bool foundValidPosition = false;
                    
                    foreach (Vector3 offset in sampleOffsets)
                    {
                        Vector3 samplePoint = robotPosition + offset;
                        NavMeshHit hit;
                        
                        // 找到最近的NavMesh点
                        if (NavMesh.SamplePosition(samplePoint, out hit, 1.0f, NavMesh.AllAreas))
                        {
                            targetPosition = hit.position;
                            foundValidPosition = true;
                            Debug.Log($"Picker {Id}: 找到靠近机器人 {currentTask.RobotId} 的有效位置: {targetPosition}");
                            break;
                        }
                    }
                    
                    if (!foundValidPosition)
                    {
                        Debug.LogWarning($"Picker {Id}: 无法找到靠近机器人的有效位置，使用原始位置");
                    }
                }
                
                // 移动到拣货位置
                SetState(PickerState.Moving, true);
                Debug.Log($"Picker {Id} moving to robot {currentTask.RobotId} at {targetPosition}");
                yield return StartCoroutine(MoveToPosition(targetPosition));
                SetState(PickerState.Moving, false);
                
                // 执行拣货
                SetState(PickerState.Picking, true);
                Debug.Log($"Picker {Id} starting picking operation");
                yield return new WaitForSeconds(pickingTime);
                SetState(PickerState.Picking, false);
                
                // 通知系统拣货完成
                if (EventManager.Instance != null)
                {
                    EventManager.Instance.TriggerPickingCompleted(currentTask.RobotId, currentTask.PickingPoint, currentTask.Location);
                    Debug.Log($"Picker {Id}: Notified robot {currentTask.RobotId} of picking completion");
                }
                
                // 准备下一个任务
                SetState(PickerState.Idle, true);
                Debug.Log($"Picker {Id} completed picking operation for Robot {currentTask.RobotId}");
                
                pickingQueue.Dequeue();
                Debug.Log($"Picker {Id}: Completed task for Robot {currentTask.RobotId}. Remaining tasks: {pickingQueue.Count}");
            }

            isProcessingTasks = false;
            Debug.Log($"Picker {Id}: All tasks completed");
        }

        public IEnumerator HandlePickingRequest(PickingEventArgs args)
        {
            // 加入队列并启动处理（如果尚未启动）
            pickingQueue.Enqueue(args);
            Debug.Log($"Picker {Id} received picking request from Robot {args.RobotId} at position {args.Location}");
            
            if (!isProcessingTasks)
            {
                StartCoroutine(ProcessPickingTask());
            }

            yield break;
        }

        private void HandlePickingCompleted(object sender, PickingEventArgs args)
        {
            if (pickingQueue.Count > 0 && pickingQueue.Peek().RobotId == args.RobotId)
            {
                Debug.Log($"Picker {Id}: Finished robot {args.RobotId}'s picking task");
                
                SetState(PickerState.Idle, true);
                
                if (pickingQueue.Count > 0)
                {
                    pickingQueue.Dequeue();
                }
                
                if (pickingQueue.Count > 0 && !isProcessingTasks)
                {
                    StartCoroutine(ProcessPickingTask());
                }
            }
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