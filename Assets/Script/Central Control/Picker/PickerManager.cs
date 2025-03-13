using UnityEngine;
using UnityEngine.AI;
using System.Collections.Generic;
using System.Linq;
using Map;

namespace CentralControl.PickerControl
{
    public class PickerManager : MonoBehaviour
    {
        #region Inspector Fields
        public GameObject pickerPrefab;
        public int numberOfPickers = 5;
        public float minPickerSpacing = 10f;
        public Transform pickersParent;
        #endregion

        #region Private Fields
        private IndoorSpace indoorSpace;
        private MapGrid mapGrid;
        private List<PickerController> pickers = new List<PickerController>();
        private Dictionary<string, List<PickerController>> pickingPointAssignments = new Dictionary<string, List<PickerController>>();
        private bool isInitialized = false;
        #endregion

        #region Public Properties
        public int TotalPickers => pickers.Count;
        public int AvailablePickers => pickers.Count(p => p.IsIdle);
        #endregion

        #region Initialization
        public void InitializePickers(IndoorSpace indoorSpace, MapGrid mapGrid)
        {
            if (isInitialized)
            {
                Debug.LogWarning("Picker Manager is already initialized.");
                return;
            }

            this.indoorSpace = indoorSpace;
            this.mapGrid = mapGrid;
            CreatePickers();
            AssignPickingZones();
            isInitialized = true;

            Debug.Log($"Initialized {pickers.Count} pickers with assigned zones");
        }

        private void CreatePickers()
        {
            List<Vector3> basePositions = CalculatePickerBasePositions();

            pickers.Clear();

            for (int i = 0; i < numberOfPickers; i++)
            {
                if (i >= basePositions.Count)
                {
                    Debug.LogError($"Not enough valid base positions for picker {i + 1}");
                    continue;
                }

                Vector3 basePosition = basePositions[i];
                int pickerId = i + 1;

                NavMeshHit hit;
                if (NavMesh.SamplePosition(basePosition, out hit, 3.0f, NavMesh.AllAreas))
                {
                    basePosition = hit.position;
                }
                else
                {
                    Debug.LogWarning($"Picker {i+1}初始位置不在NavMesh上，尝试寻找替代位置");
                    basePosition = GetValidPositionOnNavMesh();
                    
                    if (basePosition == Vector3.zero)
                    {
                        Debug.LogError($"无法为Picker {i+1}找到有效的NavMesh位置");
                        continue;
                    }
                }
                
                // Create a container object for the picker
                GameObject pickerContainer = new GameObject($"Picker_{pickerId}_Container");
                pickerContainer.transform.SetParent(pickersParent);
                pickerContainer.transform.position = basePosition;
                pickerContainer.tag = "Picker";
                pickerContainer.layer = LayerMask.NameToLayer("PickerLayer");
                
                // Instantiate the picker model as a child of the container
                GameObject pickerModel = Instantiate(pickerPrefab, Vector3.zero, Quaternion.identity);
                pickerModel.name = $"Picker_{pickerId}_Model";
                pickerModel.transform.SetParent(pickerContainer.transform, false);
                pickerModel.transform.localPosition = Vector3.zero;
                
                // Add a BoxCollider and Rigidbody to the container
                BoxCollider collider = pickerContainer.AddComponent<BoxCollider>();
                collider.size = new Vector3(0.8f, 1.85f, 0.5f); // Human-like collider size
                collider.center = new Vector3(0, 0.9f, 0.05f); // Adjust center to match model
                collider.isTrigger = false;
                
                Rigidbody rb = pickerContainer.AddComponent<Rigidbody>();
                rb.isKinematic = false;
                rb.useGravity = false;
                rb.constraints = RigidbodyConstraints.FreezeRotation;
                rb.mass = 30f; // Human-like mass
                rb.drag = 10f; // Increase drag to simulate human-like movement
                rb.angularDrag = 0.05f;
                rb.collisionDetectionMode = CollisionDetectionMode.Continuous; // More accurate collision detection

                NavMeshAgent navAgent = pickerContainer.AddComponent<NavMeshAgent>();
                navAgent.radius = 0.4f;
                navAgent.height = 1.8f;
                navAgent.speed = 3.0f; // 使用你的movementSpeed
                navAgent.angularSpeed = 120f;
                navAgent.acceleration = 8f;
                navAgent.stoppingDistance = 0.1f;
                navAgent.avoidancePriority = 50;
                navAgent.obstacleAvoidanceType = ObstacleAvoidanceType.HighQualityObstacleAvoidance;
                
                // Add a ModelSynchronizer component to the container
                ModelSynchronizer synchronizer = pickerContainer.AddComponent<ModelSynchronizer>();
                synchronizer.pickerModel = pickerModel;

                // Add a PickerController component to the container
                PickerController picker = pickerContainer.AddComponent<PickerController>();
                
                if (picker != null)
                {
                    picker.Initialize(pickerId, indoorSpace, mapGrid, basePosition);
                    pickers.Add(picker);
                }
                else
                {
                    Debug.LogError($"Failed to add PickerController component for picker {pickerId}");
                }
            }
        }

        private Vector3 GetValidPositionOnNavMesh()
        {
            // 尝试多次寻找有效位置
            for (int attempts = 0; attempts < 30; attempts++)
            {
                Vector3 testPosition = new Vector3(
                    UnityEngine.Random.Range(5f, mapGrid.width - 5f),
                    0.1f,
                    UnityEngine.Random.Range(5f, mapGrid.length - 5f)
                );
                
                NavMeshHit hit;
                if (NavMesh.SamplePosition(testPosition, out hit, 5.0f, NavMesh.AllAreas))
                {
                    return hit.position;
                }
            }
            
            return Vector3.zero; // 失败
        }

        private List<Vector3> CalculatePickerBasePositions()
        {
            List<Vector3> positions = new List<Vector3>();
            List<PickingPoint> pickingPoints = indoorSpace.PickingPoints;

            if (pickingPoints.Count == 0)
            {
                Debug.LogError("No picking points found in indoor space");
                return positions;
            }

            // Calculate the center of all picking points
            Vector3 center = Vector3.zero;
            foreach (var point in pickingPoints)
            {
                CellSpace cellSpace = indoorSpace.GetCellSpaceFromId(point.Id);
                if (cellSpace != null)
                {
                    center += new Vector3(
                        (float)cellSpace.Space.Centroid.X,
                        mapGrid.height,
                        (float)cellSpace.Space.Centroid.Y
                    );
                }
            }
            center /= pickingPoints.Count;

            // Distribute pickers around the center
            float angleStep = 360f / numberOfPickers;
            float radius = minPickerSpacing;

            for (int i = 0; i < numberOfPickers; i++)
            {
                float angle = i * angleStep * Mathf.Deg2Rad;
                Vector3 position = center + new Vector3(
                    Mathf.Cos(angle) * radius,
                    0,
                    Mathf.Sin(angle) * radius
                );

                // Ensure position is within a valid cell space
                CellSpace cellSpace = indoorSpace.GetCellSpaceFromCoordinates(position);
                if (cellSpace != null && cellSpace.IsNavigable())
                {
                    positions.Add(position);
                }
                else
                {
                    // Find nearest valid position
                    position = FindNearestValidPosition(position);
                    positions.Add(position);
                }
            }

            return positions;
        }

        private Vector3 FindNearestValidPosition(Vector3 targetPosition)
        {
            float searchRadius = 1f;
            float maxSearchRadius = 20f;
            const int angleSteps = 12; // Search every 30 degrees
            float angleStep = 360f / angleSteps;
            
            while (searchRadius <= maxSearchRadius)
            {
                for (int i = 0; i < angleSteps; i++)
                {
                    float angle = i * angleStep * Mathf.Deg2Rad;
                    Vector3 testPosition = targetPosition + new Vector3(
                        Mathf.Cos(angle) * searchRadius,
                        0,
                        Mathf.Sin(angle) * searchRadius
                    );

                    CellSpace cellSpace = indoorSpace.GetCellSpaceFromCoordinates(testPosition);
                    if (cellSpace != null && cellSpace.IsNavigable())
                    {
                        return testPosition;
                    }
                }
                searchRadius += 1f;
            }

            Debug.LogError("Could not find valid position for picker");
            return targetPosition;
        }
        #endregion

        #region Zone Management
        private void AssignPickingZones()
        {
            // Clear existing assignments
            pickingPointAssignments.Clear();
            
            // Get all picking points
            List<PickingPoint> allPickingPoints = indoorSpace.PickingPoints;
            Debug.Log($"Total picking points: {allPickingPoints.Count}");
            
            // Check if any picker has a valid work area radius
            bool anyAssignmentsFound = false;
            
            // Update the picking point assignment dictionary
            foreach (var pickingPoint in allPickingPoints)
            {
                if (!pickingPointAssignments.ContainsKey(pickingPoint.Id))
                {
                    pickingPointAssignments[pickingPoint.Id] = new List<PickerController>();
                }
                
                foreach (var picker in pickers)
                {
                    bool inRange = false;
                    try
                    {
                        inRange = picker.IsPickingPointInRange(pickingPoint);
                    }
                    catch (System.Exception ex)
                    {
                        Debug.LogError($"Picker {picker.Id} threw an exception: {ex.Message}");
                        continue;
                    }
                    
                    if (inRange)
                    {
                        pickingPointAssignments[pickingPoint.Id].Add(picker);
                        anyAssignmentsFound = true;
                        Debug.Log($"Picking point {pickingPoint.Id} is asssigned to Picker {picker.Id}");
                    }
                }
            }
            
            // Auto-adjustment logic for work area radius
            if (!anyAssignmentsFound)
            {
                Debug.LogWarning("No assignments found. Increasing work area radius for all pickers.");
                
                // Auto-increase the work area radius for all pickers
                foreach (var picker in pickers)
                {
                    picker.workAreaRadius *= 2.0f;
                    Debug.Log($"Picker {picker.Id}'s work area raidus is increasing to {picker.workAreaRadius}");
                }
                
                // Re-assign picking zones
                AssignPickingZones();
                return;
            }

            // Assign Picking Points for each picker and log
            Dictionary<int, int> pickerAssignmentCounts = new Dictionary<int, int>();
            
            foreach (var picker in pickers)
            {
                var assignedPoints = allPickingPoints
                    .Where(p => IsPickerAssignedToPoint(picker, p))
                    .ToList();
                    
                picker.AssignPickingPoints(assignedPoints);
                pickerAssignmentCounts[picker.Id] = assignedPoints.Count;
                
                Debug.Log($"Picker {picker.Id} has assigned picking point: {string.Join(", ", assignedPoints.Select(p => p.Id))}");
            }
            
            foreach (var picker in pickers)
            {
                Debug.Log($"Picker {picker.Id} is assigned {pickerAssignmentCounts[picker.Id]} picking points");
            }
        }

        private bool IsPickerAssignedToPoint(PickerController picker, PickingPoint point)
        {
            if (pickingPointAssignments.TryGetValue(point.Id, out var assignedPickers))
            {
                return assignedPickers.Contains(picker);
            }
            return false;
        }
        #endregion

        #region Broadcast Events
        public PickerController GetBestPickerForPoint(PickingPoint pickingPoint, Vector3 location)
        {
            // Get pickers in range
            var availablePickers = pickers.Where(p => 
                p.IsPickingPointInRange(pickingPoint) && 
                p.IsIdle).ToList();
            
            if (availablePickers.Count == 0)
            {
                // If no idle pickers, get pickers with shortest queue
                availablePickers = pickers.Where(p => 
                    p.IsPickingPointInRange(pickingPoint)).OrderBy(p => 
                    p.QueuedTaskCount).ToList();
                    
                if (availablePickers.Count == 0)
                    return null;
            }
            
            // Find closest picker or picker with least workload
            return availablePickers
                .OrderBy(p => Vector3.Distance(p.CurrentPosition, location) * (1 + p.QueuedTaskCount * 0.5f))
                .FirstOrDefault();
        }
        #endregion

        #region Status Updates
        private void Update()
        {
            if (!isInitialized) return;

            MonitorPickerStatus();
        }

        private void MonitorPickerStatus()
        {
            foreach (var picker in pickers)
            {
                // Future: Implement idle time tracking and workload balancing
                if (picker.IsIdle)
                {
                    // Could implement optimization here
                }
            }
        }
        #endregion

        #region Public Methods
        public List<PickerController> GetAllPickers()
        {
            return new List<PickerController>(pickers);
        }

        public List<PickerController> GetIdlePickers()
        {
            return pickers.Where(p => p.IsIdle).ToList();
        }

        public List<PickerController> GetPickersForPoint(PickingPoint pickingPoint)
        {
            if (pickingPointAssignments.TryGetValue(pickingPoint.Id, out var assignedPickers))
            {
                return assignedPickers;
            }
            return new List<PickerController>();
        }

        public void ReassignPickingZones()
        {
            if (!isInitialized)
            {
                Debug.LogWarning("Cannot reassign zones: Picker Manager not initialized");
                return;
            }

            AssignPickingZones();
        }

        public void ResetPickers()
        {
            foreach (var picker in pickers)
            {
                // Return picker to base position
                StartCoroutine(picker.MoveToPosition(picker.baseLocation.position));
            }
        }
        #endregion

        #region Visualization
        private void OnDrawGizmos()
        {
            if (!isInitialized) return;

            foreach (var picker in pickers)
            {
                // Visualization of picker zones and assignments
                Gizmos.color = Color.yellow;
                Gizmos.DrawWireSphere(picker.transform.position, 0.5f);

                // Draw lines to assigned picking points
                if (picker.baseLocation != null)
                {
                    Gizmos.color = new Color(1, 1, 0, 0.2f);
                    Gizmos.DrawWireSphere(picker.baseLocation.position, picker.workAreaRadius);
                }
            }
        }
        #endregion
    }
}