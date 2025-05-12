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
        private Vector3 targetPosition;
        private Queue<PickingTask> pickingQueue = new Queue<PickingTask>();
        private PickingTask currentTask;
        private bool isProcessingTasks = false;
        private bool isInitialized = false;
        private NavMeshAgent navAgent;
        private Transform baseLocation;
        private string currentFloor;
        private List<PickingPoint> assignedPickingPoints = new List<PickingPoint>();
        #endregion

        #region State Properties
        public bool IsIdle => !currentStates.Any() || (currentStates.Count == 1 && currentStates.Contains(PickerState.Idle));
        public bool IsMoving => currentStates.Contains(PickerState.Moving);
        public bool IsPicking => currentStates.Contains(PickerState.Picking);
        public string CurrentFloor => currentFloor;
        public Vector3 CurrentPosition => transform.position;
        public int QueuedTaskCount => pickingQueue.Count;
        #endregion

        #region Unity Methods
        void Awake()
        {
            navAgent = GetComponent<NavMeshAgent>();
            if (navAgent == null)
            {
                Debug.LogError($"Picker {Id}: No NavMeshAgent found");
            }
            
            baseLocation = transform;
        }

        void Start()
        {
            if (!isInitialized)
            {
                Debug.LogWarning($"Picker {Id} was not properly initialized");
            }
            
            // Register picker event handlers
            RegisterEventHandlers();
            
            // Set initial state
            SetState(PickerState.Idle, true);
        }

        void Update()
        {
            if (!isInitialized) return;
            
            UpdateCurrentPosition();
            
            // Process picking queue if not already processing and there are tasks in queue
            if (!isProcessingTasks && pickingQueue.Count > 0 && IsIdle)
            {
                StartCoroutine(ProcessPickingTask());
            }
        }

        void OnDestroy()
        {
            // Unregister from events
            if (EventManager.Instance != null)
            {
                EventManager.Instance.OnRobotReadyForPicking -= HandleRobotReadyForPicking;
                EventManager.Instance.OnPickingCompleted -= HandlePickingCompleted;
            }
        }
        #endregion

        #region Initialization
        public void Initialize(int id, IndoorSpace indoorSpace, MapGrid mapGrid, Vector3 basePosition, string floorLevel)
        {
            this.Id = id;
            this.indoorSpace = indoorSpace;
            this.mapGrid = mapGrid;
            this.currentFloor = floorLevel;
            
            // Set base location
            transform.position = basePosition;
            baseLocation = transform;
            
            if (navAgent != null)
            {
                navAgent.speed = movementSpeed;
            }
            
            isInitialized = true;
            Debug.Log($"Picker {Id} initialized at position {basePosition} on floor {currentFloor}");
        }

        private void RegisterEventHandlers()
        {
            if (EventManager.Instance != null)
            {
                EventManager.Instance.OnRobotReadyForPicking += HandleRobotReadyForPicking;
                EventManager.Instance.OnPickingCompleted += HandlePickingCompleted;
            }
            else
            {
                Debug.LogError("EventManager instance not found. Make sure it exists in the scene.");
            }
        }
        
        public void AssignPickingPoints(List<PickingPoint> pickingPoints)
        {
            assignedPickingPoints = pickingPoints ?? new List<PickingPoint>();
            // Debug.Log($"Picker {Id}: Assigned {assignedPickingPoints.Count} picking points");
        }
        
        public List<PickingPoint> GetPickingPoints()
        {
            return assignedPickingPoints;
        }
        
        // Method to check if a picking point is in range (used by PickerManager)
        public bool IsPickingPointInRange(PickingPoint pickingPoint)
        {
            if (pickingPoint == null) return false;
            
            string pointFloor = pickingPoint.FloorLevel;
            if (pointFloor != currentFloor)
            {
                return false;
            }
            
            Vector3 position = GetPickingPointPosition(pickingPoint);
            float distance = Vector2.Distance(
                new Vector2(transform.position.x, transform.position.z),
                new Vector2(position.x, position.z)
            );
            
            return distance <= workAreaRadius;
        }
        #endregion

        #region Task Management
        private void HandleRobotReadyForPicking(object sender, PickingEventArgs args)
        {
            Debug.Log($"Picker {Id} received event: Robot {args.RobotId} at {args.Location}");
            
            if (args.PickingPoint.FloorLevel != currentFloor)
            {
                Debug.Log($"Picker {Id}: Picking point {args.PickingPoint.Id} on floor {args.PickingPoint.FloorLevel} - different from picker floor {currentFloor}");
                return;
            }
            
            if (!IsPickingPointInRange(args.PickingPoint))
            {
                Debug.Log($"Picker {Id}: Picking point {args.PickingPoint.Id} is NOT in range");
                return;
            }
            Debug.Log($"Picker {Id}: Picking point {args.PickingPoint.Id} is in range, adding to queue");

            StartCoroutine(HandlePickingRequest(args));
        }

        private IEnumerator HandlePickingRequest(PickingEventArgs args)
        {
            // Short delay to prevent race conditions with multiple events
            yield return new WaitForSeconds(0.1f);
            
            lock (pickingQueue) // Using lock to ensure thread safety
            {
                string pickingPointId = args.PickingPoint.Id;
                
                // First check if this picking point is already in the queue
                bool alreadyInQueue = false;
                foreach (var task in pickingQueue)
                {
                    if (task.PickingPoint.Id == pickingPointId)
                    {
                        alreadyInQueue = true;
                        break;
                    }
                }
                
                if (!alreadyInQueue)
                {
                    // Create a new task - 确保任务中的位置是当前楼层
                    Vector3 taskLocation = args.Location;
                    taskLocation.y = transform.position.y; // 强制使用Picker所在楼层的高度
                    
                    PickingTask task = new PickingTask(
                        args.RobotId,
                        args.PickingPoint,
                        taskLocation,
                        currentFloor // 使用Picker的固定楼层
                    );
                    
                    // Add to queue
                    pickingQueue.Enqueue(task);
                    Debug.Log($"Picker {Id}: Added picking task for robot {args.RobotId} at {taskLocation}");
                    
                    // If idle, start processing tasks
                    if (IsIdle && !isProcessingTasks)
                    {
                        StartCoroutine(ProcessPickingTask());
                    }
                }
                else
                {
                    Debug.Log($"Picker {Id}: Picking point {pickingPointId} is already in queue");
                }
            }
        }

        private void HandlePickingCompleted(object sender, PickingEventArgs args)
        {
            lock (pickingQueue) // Using lock to ensure thread safety
            {
                if (pickingQueue.Count > 0 && pickingQueue.Peek().RobotId == args.RobotId)
                {
                    Debug.Log($"Picker {Id}: Finished robot {args.RobotId}'s picking task");
                    
                    SetState(PickerState.Idle, true);
                    
                    try
                    {
                        pickingQueue.Dequeue(); // Safely remove task
                    }
                    catch (InvalidOperationException)
                    {
                        Debug.LogWarning($"Picker {Id}: Attempted to remove task failed, queue may be empty");
                        // Continue execution, don't throw exception
                    }
                }
                
                // Check if there are more tasks to process
                if (pickingQueue.Count > 0 && !isProcessingTasks)
                {
                    StartCoroutine(ProcessPickingTask());
                }
            }
        }

        private IEnumerator ProcessPickingTask()
        {
            if (isProcessingTasks)
            {
                yield break;
            }
            
            isProcessingTasks = true;
            
            while (pickingQueue.Count > 0)
            {
                // Get next task
                lock (pickingQueue)
                {
                    currentTask = pickingQueue.Peek();
                }
                
                if (currentTask == null)
                {
                    Debug.LogWarning($"Picker {Id}: Current task is null");
                    yield break;
                }
                
                targetPosition = currentTask.Location;
                Debug.Log($"Picker {Id} processing task: Robot {currentTask.RobotId} at {targetPosition}");
                
                // Move to target position
                SetState(PickerState.Moving, true);
                Debug.Log($"Picker {Id} moving to robot {currentTask.RobotId} at {targetPosition}");
                yield return StartCoroutine(MoveToPosition(targetPosition));
                SetState(PickerState.Moving, false);
                
                // Picking operation
                SetState(PickerState.Picking, true);
                Debug.Log($"Picker {Id} starting picking operation");
                yield return new WaitForSeconds(pickingTime);
                SetState(PickerState.Picking, false);
                
                // Notify system that picking is completed
                if (EventManager.Instance != null)
                {
                    EventManager.Instance.TriggerPickingCompleted(currentTask.RobotId, currentTask.PickingPoint, currentTask.Location);
                }
                
                // Wait for event handling to complete, ensuring HandlePickingCompleted has time to process
                yield return null;
                
                // Safely remove task (if it's still the top task in the queue)
                // Note: This is just an extra safety measure, normally handled by HandlePickingCompleted
                lock (pickingQueue)
                {
                    if (pickingQueue.Count > 0 && pickingQueue.Peek().RobotId == currentTask.RobotId)
                    {
                        try
                        {
                            pickingQueue.Dequeue();
                            Debug.Log($"Picker {Id}: Task has been removed from queue: {currentTask.RobotId}");
                        }
                        catch (InvalidOperationException)
                        {
                            Debug.LogWarning($"Picker {Id}: Cannot remove task, queue may be empty");
                        }
                    }
                }
            }
            
            isProcessingTasks = false;
            Debug.Log($"Picker {Id}: All tasks completed");
        }
        #endregion

        #region Movement
        private IEnumerator MoveToPosition(Vector3 position)
        {
            if (navAgent == null)
            {
                Debug.LogError($"Picker {Id}: Nav agent is null");
                yield break;
            }
            
            position.y = transform.position.y;
            
            navAgent.SetDestination(position);
            Debug.Log($"Picker {Id}: Moving to {position}, remaining distance: {navAgent.remainingDistance}");
            
            // Wait until we're within stopping distance or can't move any closer
            float timeout = 60f; // Timeout after 60 seconds to prevent infinite loops
            float elapsedTime = 0f;
            
            while (navAgent.remainingDistance > navAgent.stoppingDistance && elapsedTime < timeout)
            {
                if (!navAgent.pathPending && navAgent.pathStatus == NavMeshPathStatus.PathInvalid)
                {
                    Debug.LogWarning($"Picker {Id}: Path invalid to {position}");
                    yield break;
                }
                
                yield return new WaitForSeconds(0.1f);
                elapsedTime += 0.1f;
            }
            
            if (elapsedTime >= timeout)
            {
                Debug.LogWarning($"Picker {Id}: Movement timed out");
            }
            else
            {
                Debug.Log($"Picker {Id}: Reached destination {position}");
            }
        }

        private void UpdateCurrentPosition()
        {
            if (navAgent != null)
            {
                currentPosition = transform.position;
            }
        }
        #endregion

        #region Helper Methods
        private Vector3 GetPickingPointPosition(PickingPoint pickingPoint)
        {
            if (pickingPoint == null) return transform.position;
            
            CellSpace cellSpace = indoorSpace.GetCellSpaceFromId(pickingPoint.Id);
            if (cellSpace == null || cellSpace.Space == null) return transform.position;
            
            return new Vector3(
                (float)cellSpace.Space.Centroid.X,
                transform.position.y,
                (float)cellSpace.Space.Centroid.Y
            );
        }

        private void SetState(PickerState state, bool value)
        {
            bool previousState = currentStates.Contains(state);
            
            if (value)
            {
                currentStates.Add(state);
            }
            else
            {
                currentStates.Remove(state);
            }
            
            // Log state changes
            if (previousState != value)
            {
                Debug.Log($"Picker {Id}: State {state} changed to {value}");
            }
        }
        #endregion
    }

    // Helper class for picking tasks
    public class PickingTask
    {
        public string RobotId { get; private set; }
        public PickingPoint PickingPoint { get; private set; }
        public Vector3 Location { get; private set; }
        public string FloorLevel { get; private set; }
        
        public PickingTask(string robotId, PickingPoint pickingPoint, Vector3 location, string floorLevel)
        {
            RobotId = robotId;
            PickingPoint = pickingPoint;
            Location = location;
            FloorLevel = floorLevel;
        }
    }
}