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
        public int Id { get; private set; }
        public float pickingTime = 10f;
        public float movementSpeed = 3f;
        public float workAreaRadius = 10f;
        #endregion

        #region Private Fields
        private IndoorSpace indoorSpace;
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

        private void Start()
        {
            EventManager.Instance.OnRobotReadyForPicking += HandleRobotReadyForPicking;
        }

        private void OnDestroy()
        {
            if (EventManager.Instance != null)
            {
                EventManager.Instance.OnRobotReadyForPicking -= HandleRobotReadyForPicking;
            }
        }

        #region Initialization
        public void Initialize(int id, IndoorSpace indoorSpace, Vector3 basePosition)
        {
            Id = id;
            this.indoorSpace = indoorSpace;
            
            GameObject baseLocationObj = new GameObject($"Picker_{id}_BaseLocation");
            baseLocationObj.transform.parent = this.transform;
            baseLocation = baseLocationObj.transform;
            baseLocation.position = basePosition;

            currentPosition = basePosition;
            SetState(PickerState.Idle, true);
            Debug.Log($"Picker {Id} initialized at position {basePosition}");
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
            CellSpace cellSpace = indoorSpace.GetCellSpaceFromId(pickingPoint.Id);
            if (cellSpace == null)
            {
                Debug.LogWarning($"Picker {Id}: Unable to find cell space for picking point {pickingPoint.Id}");
                return false;
            }

            Vector3 pickingPosition = new Vector3(
                (float)cellSpace.Space.Centroid.X,
                transform.position.y,
                (float)cellSpace.Space.Centroid.Y
            );

            float distance = Vector3.Distance(baseLocation.position, pickingPosition);
            Debug.Log($"Picker {Id}: Distance to picking point {pickingPoint.Id} is {distance}, threshold is {workAreaRadius}");
            return distance <= workAreaRadius;
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
                
                Debug.Log($"Picker {Id} moving: current position = {transform.position}, target = {targetPosition}, " +
                         $"distance = {Vector3.Distance(transform.position, targetPosition)}");
                
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