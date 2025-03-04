using UnityEngine;
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
            // 确保默认ID不为0
            _id = -1;
        }

        private void Awake()
        {
            // 如果ID为0，给出警告
            if (_id == 0)
            {
                Debug.LogWarning($"警告: Picker的ID为0，这可能导致问题。GameObject: {gameObject.name}");
            }
            
            // 确保不会有未初始化就注册的事件处理器
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

            // Add task to queue
            pickingQueue.Enqueue(args);
            Debug.Log($"Picker {Id} received picking request from Robot {args.RobotId} at position {args.Location}");

            // Process tasks if not already processing
            if (!isProcessingTasks)
            {
                StartCoroutine(ProcessPickingTask());
            }
        }

        private IEnumerator ProcessPickingTask()
        {
            isProcessingTasks = true;

            while (pickingQueue.Count > 0)
            {
                PickingEventArgs currentTask = pickingQueue.Peek();
                Debug.Log($"Picker {Id}: Processing picking task for Robot {currentTask.RobotId}");

                yield return StartCoroutine(HandlePickingRequest(currentTask));

                if (EventManager.Instance != null)
                {
                    EventManager.Instance.TriggerPickingCompleted(currentTask.RobotId, currentTask.PickingPoint, currentTask.Location);
                    Debug.Log($"Picker {Id}: Notified robot {currentTask.RobotId} of picking completion");
                }

                pickingQueue.Dequeue();
                Debug.Log($"Picker {Id}: Completed task for Robot {currentTask.RobotId}. Remaining tasks: {pickingQueue.Count}");
            }

            isProcessingTasks = false;
            Debug.Log($"Picker {Id}: All tasks completed");
        }

        public IEnumerator HandlePickingRequest(PickingEventArgs args)
        {
            // Move to picking location
            SetState(PickerState.Moving, true);
            Debug.Log($"Picker {Id} moving to robot {args.RobotId}");
            yield return StartCoroutine(MoveToPosition(args.Location));
            SetState(PickerState.Moving, false);

            // Perform picking
            SetState(PickerState.Picking, true);
            Debug.Log($"Picker {Id} starting picking operation");
            yield return new WaitForSeconds(pickingTime);
            SetState(PickerState.Picking, false);

            // Signal completion
            EventManager.Instance.TriggerPickingCompleted(args.RobotId, args.PickingPoint, args.Location);
            Debug.Log($"Picker {Id} completed picking for robot {args.RobotId}");

            // Prepare for next task
            SetState(PickerState.Idle, true);

            // After completing picking
            EventManager.Instance.TriggerPickingCompleted(args.RobotId, args.PickingPoint, args.Location);
            Debug.Log($"Picker {Id} completed picking operation for Robot {args.RobotId}");
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
            float startTime = Time.time;
            Vector3 startPosition = transform.position;
            
            while (Vector3.Distance(transform.position, targetPosition) > 0.1f)
            {
                transform.position = Vector3.MoveTowards(
                    transform.position,
                    targetPosition,
                    movementSpeed * Time.deltaTime
                );
                
                yield return null;
            }
            
            transform.position = targetPosition;
            Debug.Log($"Picker {Id} reached target position {targetPosition}");
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
                // Draw work area
                Gizmos.color = Color.green;
                Gizmos.DrawWireSphere(baseLocation.position, workAreaRadius);

                // Draw current state
                if (IsIdle)
                    Gizmos.color = Color.gray;
                else if (IsPicking)
                    Gizmos.color = Color.yellow;
                else if (IsMoving)
                    Gizmos.color = Color.blue;

                Gizmos.DrawWireSphere(transform.position, 0.5f);

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