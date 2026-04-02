using UnityEngine;
using UnityEngine.AI;
using System.Collections.Generic;
using System.Linq;
using NetTopologySuite.Geometries;
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
        
        [Header("Floor Settings")]
        [Tooltip("Whether to distribute pickers evenly across floors")]
        public bool distributeEvenly = true;
        
        [Tooltip("Custom floor distribution ratios (used when distributeEvenly is false)")]
        public List<PickerFloorDistribution> customPickerFloorDistributions = new List<PickerFloorDistribution>();
        
        [Tooltip("Floor height in world units")]
        public float floorHeight = 3.0f;
        
        [Tooltip("Base height offset")]
        public float baseHeight = 0.05f;
        #endregion

        #region Private Fields
        private List<PickerController> pickers = new List<PickerController>();
        private Dictionary<string, List<PickerController>> pickersByFloor = new Dictionary<string, List<PickerController>>();
        private Dictionary<string, List<PickerController>> pickingPointAssignments = new Dictionary<string, List<PickerController>>();
        private IndoorSpace indoorSpace;
        private MapGrid mapGrid;
        private bool isInitialized = false;
        private HashSet<CellSpace> usedSpawnPoints = new HashSet<CellSpace>();
        private Dictionary<string, HashSet<CellSpace>> usedSpawnPointsByFloor = new Dictionary<string, HashSet<CellSpace>>();
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
            Debug.Log($"PickerManager initialized with {pickers.Count} pickers across {pickersByFloor.Count} floors");
        }
        
        private void CreatePickers()
        {
            if (pickerPrefab == null)
            {
                Debug.LogError("Picker prefab is not assigned!");
                return;
            }
            
            if (pickersParent == null)
            {
                GameObject container = new GameObject("Pickers");
                pickersParent = container.transform;
            }
            
            // Clear existing pickers
            pickers.Clear();
            pickersByFloor.Clear();
            usedSpawnPointsByFloor.Clear();
            
            // Get all available floors
            List<string> floors = indoorSpace.GetAllFloorLevels();
            if (floors == null || floors.Count == 0)
            {
                Debug.LogWarning("No floors found in IndoorSpace. Defaulting to single floor (1F).");
                floors = new List<string> { "1F" };
            }
            
            Debug.Log($"Available floors: {string.Join(", ", floors)}");
            
            // Calculate picker distribution by floor
            Dictionary<string, int> pickerDistribution = CalculatePickerDistribution(floors, numberOfPickers);
            
            // Create pickers for each floor according to the distribution
            int pickerId = 1;
            foreach (var floorEntry in pickerDistribution)
            {
                string floorLevel = floorEntry.Key;
                int pickersOnFloor = floorEntry.Value;
                
                Debug.Log($"Creating {pickersOnFloor} pickers on floor {floorLevel}");
                
                // Initialize used spawn points tracking for this floor
                if (!usedSpawnPointsByFloor.ContainsKey(floorLevel))
                {
                    usedSpawnPointsByFloor[floorLevel] = new HashSet<CellSpace>();
                }
                
                for (int i = 0; i < pickersOnFloor; i++)
                {
                    // Get valid position on this floor
                    Vector3 basePosition = GetValidInitialPosition(floorLevel);
                    
                    if (basePosition == Vector3.zero)
                    {
                        Debug.LogError($"Failed to find valid position for picker {pickerId} on floor {floorLevel}");
                        continue;
                    }
                    
                    // Create picker
                    CreatePicker(pickerId, basePosition, floorLevel);
                    pickerId++;
                }
            }
            
            Debug.Log($"Created {pickers.Count} pickers across {pickersByFloor.Count} floors");
        }
        
        private void CreatePicker(int id, Vector3 position, string floorLevel)
        {
            // Create a container object for the picker
            GameObject pickerContainer = new GameObject($"Picker_{id}_Container");
            pickerContainer.transform.SetParent(pickersParent);
            pickerContainer.transform.position = position;
            pickerContainer.tag = "Picker";
            
            // Set layer if it exists
            int pickerLayerIndex = LayerMask.NameToLayer("PickerLayer");
            if (pickerLayerIndex != -1)
            {
                pickerContainer.layer = pickerLayerIndex;
            }
            
            // Instantiate the picker model as a child of the container
            GameObject pickerModel = Instantiate(pickerPrefab, Vector3.zero, Quaternion.identity);
            pickerModel.name = $"Picker_{id}_Model";
            pickerModel.transform.SetParent(pickerContainer.transform, false);
            pickerModel.transform.localPosition = Vector3.zero;
            
            // Add NavMeshAgent to the container
            NavMeshAgent navAgent = pickerContainer.AddComponent<NavMeshAgent>();
            navAgent.radius = 0.4f;
            navAgent.height = 1.8f;
            navAgent.speed = 3.0f;
            navAgent.angularSpeed = 120f;
            navAgent.acceleration = 8f;
            navAgent.stoppingDistance = 0.1f;
            navAgent.avoidancePriority = 50;
            navAgent.obstacleAvoidanceType = ObstacleAvoidanceType.HighQualityObstacleAvoidance;
            
            // Add a ModelSynchronizer component to the container (if needed)
            if (pickerContainer.GetComponent<ModelSynchronizer>() == null)
            {
                ModelSynchronizer synchronizer = pickerContainer.AddComponent<ModelSynchronizer>();
                synchronizer.pickerModel = pickerModel;
            }

            // Add a PickerController component to the container
            PickerController picker = pickerContainer.AddComponent<PickerController>();
            
            if (picker != null)
            {
                // Initialize the picker with the floor level
                picker.Initialize(id, indoorSpace, mapGrid, position, floorLevel);
                pickers.Add(picker);
                
                // Add to floor-specific list
                if (!pickersByFloor.ContainsKey(floorLevel))
                {
                    pickersByFloor[floorLevel] = new List<PickerController>();
                }
                pickersByFloor[floorLevel].Add(picker);
                
                Debug.Log($"Created picker {id} on floor {floorLevel} at position {position}");
            }
            else
            {
                Debug.LogError($"Failed to add PickerController component for picker {id}");
            }
        }
        
        private Vector3 GetValidInitialPosition(string floorLevel)
        {
            // Initialize used spawn points set for this floor if needed
            if (!usedSpawnPointsByFloor.ContainsKey(floorLevel))
            {
                usedSpawnPointsByFloor[floorLevel] = new HashSet<CellSpace>();
            }
            
            // Try to get navigable cell spaces first (preferred spawn locations)
            List<CellSpace> spawnPoints = indoorSpace.GetNavigableCellSpacesOnFloor(floorLevel);
            
            // If no navigable spaces found, try layer-based lookup
            if (spawnPoints == null || spawnPoints.Count == 0)
            {
                spawnPoints = indoorSpace.GetCellSpaceFromLayerTypeAndFloor("aisle", floorLevel);
                
                if (spawnPoints == null || spawnPoints.Count == 0)
                {
                    // Fallback - get any cell space on this floor
                    spawnPoints = indoorSpace.GetCellSpacesOnFloor(floorLevel);
                    
                    if (spawnPoints == null || spawnPoints.Count == 0)
                    {
                        Debug.LogError($"No cell spaces found on floor {floorLevel}");
                        return Vector3.zero;
                    }
                }
            }
            
            // Filter out already used spawn points
            List<CellSpace> availableSpawnPoints = spawnPoints
                .Where(sp => !usedSpawnPointsByFloor[floorLevel].Contains(sp))
                .ToList();
            
            // If all spawn points are used, reset and use all of them
            if (availableSpawnPoints.Count == 0)
            {
                Debug.LogWarning($"All spawn points on floor {floorLevel} already used, reusing points");
                usedSpawnPointsByFloor[floorLevel].Clear();
                availableSpawnPoints = spawnPoints;
            }
            
            // Select a random spawn point
            CellSpace selectedSpawnPoint = availableSpawnPoints[Random.Range(0, availableSpawnPoints.Count)];
            usedSpawnPointsByFloor[floorLevel].Add(selectedSpawnPoint);
            
            // Calculate position from the selected cell space
            Point centroid = selectedSpawnPoint.Space?.Centroid as Point;
            if (centroid == null)
            {
                Debug.LogError($"Selected cell space {selectedSpawnPoint.Id} has no valid centroid");
                return Vector3.zero;
            }
            
            // Calculate Y coordinate based on floor level
            int floorNum = ParseFloorLevel(floorLevel);
            float y = ((floorNum - 1) * floorHeight) + baseHeight;
            
            Vector3 position = new Vector3((float)centroid.X, y, (float)centroid.Y);
            
            // Ensure position is on NavMesh if needed
            NavMeshHit hit;
            if (NavMesh.SamplePosition(position, out hit, 3.0f, NavMesh.AllAreas))
            {
                position = hit.position;
            }
            
            return position;
        }
        
        private Dictionary<string, int> CalculatePickerDistribution(List<string> floors, int totalPickers)
        {
            Dictionary<string, int> distribution = new Dictionary<string, int>();
            
            if (distributeEvenly)
            {
                // Distribute pickers evenly across floors
                int pickersPerFloor = totalPickers / floors.Count;
                int remainder = totalPickers % floors.Count;
                
                foreach (string floor in floors)
                {
                    distribution[floor] = pickersPerFloor;
                }
                
                // Distribute remainder pickers
                for (int i = 0; i < remainder; i++)
                {
                    string floor = floors[i % floors.Count];
                    distribution[floor]++;
                }
            }
            else
            {
                // Use custom distribution ratios
                Dictionary<string, float> ratios = new Dictionary<string, float>();
                
                // Extract ratios from custom distribution
                foreach (var dist in customPickerFloorDistributions)
                {
                    if (floors.Contains(dist.floorLevel))
                    {
                        ratios[dist.floorLevel] = dist.ratio;
                    }
                }
                
                // If not all floors have ratios, distribute remaining ratio evenly
                float configuredRatioSum = ratios.Values.Sum();
                float remainingRatio = Mathf.Max(0, 1.0f - configuredRatioSum);
                int unconfiguredFloors = floors.Count - ratios.Count;
                
                if (unconfiguredFloors > 0 && remainingRatio > 0)
                {
                    float ratioPerFloor = remainingRatio / unconfiguredFloors;
                    foreach (string floor in floors)
                    {
                        if (!ratios.ContainsKey(floor))
                        {
                            ratios[floor] = ratioPerFloor;
                        }
                    }
                }
                else if (configuredRatioSum <= 0)
                {
                    // If no valid configuration, use equal distribution
                    foreach (string floor in floors)
                    {
                        ratios[floor] = 1.0f / floors.Count;
                    }
                }
                
                // Calculate pickers per floor based on ratios
                int assignedPickers = 0;
                
                foreach (var entry in ratios)
                {
                    int pickersOnFloor = Mathf.FloorToInt(totalPickers * entry.Value);
                    distribution[entry.Key] = pickersOnFloor;
                    assignedPickers += pickersOnFloor;
                }
                
                // Distribute any remaining pickers
                int remainingPickers = totalPickers - assignedPickers;
                if (remainingPickers > 0)
                {
                    var sortedByRatio = ratios.OrderByDescending(r => r.Value).Select(r => r.Key).ToList();
                    for (int i = 0; i < remainingPickers; i++)
                    {
                        distribution[sortedByRatio[i % sortedByRatio.Count]]++;
                    }
                }
            }
            
            return distribution;
        }
        #endregion

        #region Picking Zone Assignment
        public void AssignPickingZones()
        {
            // Clear existing assignments
            pickingPointAssignments.Clear();
            
            // Reset all picker assignments
            foreach (var picker in pickers)
            {
                picker.AssignPickingPoints(new List<PickingPoint>());
            }
            
            // Get all picking points by floor
            Dictionary<string, List<PickingPoint>> pickingPointsByFloor = GetPickingPointsByFloor();
            
            // For each floor, assign picking points to pickers
            foreach (var floorEntry in pickingPointsByFloor)
            {
                string floorLevel = floorEntry.Key;
                List<PickingPoint> floorPickingPoints = floorEntry.Value;
                
                if (floorPickingPoints.Count == 0) continue;
                
                // Get pickers on this floor
                List<PickerController> floorPickers = GetPickersOnFloor(floorLevel);
                
                // If no pickers on this floor, try to find pickers on adjacent floors
                if (floorPickers.Count == 0)
                {
                    floorPickers = FindPickersForFloor(floorLevel);
                }
                
                if (floorPickers.Count == 0)
                {
                    Debug.LogWarning($"No pickers available for floor {floorLevel}");
                    continue;
                }
                
                // Assign picking points to pickers on this floor
                AssignPickingPointsToPickers(floorPickingPoints, floorPickers);
            }
        }
        
        private Dictionary<string, List<PickingPoint>> GetPickingPointsByFloor()
        {
            Dictionary<string, List<PickingPoint>> result = new Dictionary<string, List<PickingPoint>>();
            
            // Get all floors
            List<string> floors = indoorSpace.GetAllFloorLevels();
            
            // Initialize dictionary for each floor
            foreach (string floor in floors)
            {
                result[floor] = new List<PickingPoint>();
            }
            
            // Get picking points for each floor
            foreach (string floor in floors)
            {
                result[floor] = indoorSpace.GetPickingPointsOnFloor(floor);
            }
            
            return result;
        }
        
        private void AssignPickingPointsToPickers(List<PickingPoint> pickingPoints, List<PickerController> availablePickers)
        {
            if (pickingPoints == null || pickingPoints.Count == 0 || availablePickers == null || availablePickers.Count == 0)
            {
                return;
            }
            
            // Initialize assignments dictionary
            foreach (var point in pickingPoints)
            {
                if (!pickingPointAssignments.ContainsKey(point.Id))
                {
                    pickingPointAssignments[point.Id] = new List<PickerController>();
                }
            }
            
            // First pass: assign each picking point to pickers that have it in range
            foreach (var point in pickingPoints)
            {
                foreach (var picker in availablePickers)
                {
                    if (picker.IsPickingPointInRange(point))
                    {
                        // Add picker to point's assignments
                        pickingPointAssignments[point.Id].Add(picker);
                        
                        // Add point to picker's list
                        List<PickingPoint> pickerPoints = picker.GetPickingPoints() ?? new List<PickingPoint>();
                        if (!pickerPoints.Contains(point))
                        {
                            pickerPoints.Add(point);
                            picker.AssignPickingPoints(pickerPoints);
                        }
                        
                        // Debug.Log($"Assigned picking point {point.Id} to Picker {picker.Id} (in range)");
                    }
                }
            }
            
            // Second pass: for unassigned points, find the closest picker
            var unassignedPoints = pickingPoints.Where(p => 
                !pickingPointAssignments.ContainsKey(p.Id) || 
                pickingPointAssignments[p.Id].Count == 0).ToList();
            
            foreach (var point in unassignedPoints)
            {
                PickerController closestPicker = FindClosestPicker(point, availablePickers);
                if (closestPicker != null)
                {
                    // Add picker to point's assignments
                    pickingPointAssignments[point.Id].Add(closestPicker);
                    
                    // Add point to picker's list
                    List<PickingPoint> pickerPoints = closestPicker.GetPickingPoints() ?? new List<PickingPoint>();
                    if (!pickerPoints.Contains(point))
                    {
                        pickerPoints.Add(point);
                        closestPicker.AssignPickingPoints(pickerPoints);
                    }
                    
                    // Debug.Log($"Assigned picking point {point.Id} to closest Picker {closestPicker.Id}");
                }
            }
        }
        
        private PickerController FindClosestPicker(PickingPoint pickingPoint, List<PickerController> availablePickers)
        {
            if (availablePickers == null || availablePickers.Count == 0) return null;
            
            CellSpace pointCellSpace = indoorSpace.GetCellSpaceFromId(pickingPoint.Id);
            if (pointCellSpace == null || pointCellSpace.Space == null) return availablePickers[0];
            
            Vector2 pointPos = new Vector2(
                (float)pointCellSpace.Space.Centroid.X,
                (float)pointCellSpace.Space.Centroid.Y
            );
            
            PickerController closest = null;
            float minDistance = float.MaxValue;
            
            foreach (var picker in availablePickers)
            {
                Vector2 pickerPos = new Vector2(picker.transform.position.x, picker.transform.position.z);
                float distance = Vector2.Distance(pickerPos, pointPos);
                
                // Add floor difference penalty
                if (picker.CurrentFloor != pickingPoint.FloorLevel)
                {
                    int floorDiff = Mathf.Abs(ParseFloorLevel(picker.CurrentFloor) - ParseFloorLevel(pickingPoint.FloorLevel));
                    distance += floorDiff * 50f; // Higher penalty for closest picker calculation
                }
                
                if (distance < minDistance)
                {
                    minDistance = distance;
                    closest = picker;
                }
            }
            
            return closest;
        }
        
        private List<PickerController> FindPickersForFloor(string floorLevel)
        {
            // Get list of directly reachable floors from the target floor
            List<string> reachableFloors = indoorSpace.GetDirectlyReachableFloors(floorLevel);
            List<PickerController> candidatePickers = new List<PickerController>();
            
            // Try to get pickers from directly reachable floors
            foreach (string reachableFloor in reachableFloors)
            {
                if (pickersByFloor.ContainsKey(reachableFloor))
                {
                    candidatePickers.AddRange(pickersByFloor[reachableFloor]);
                }
            }
            
            // If no pickers found on reachable floors, use all pickers
            if (candidatePickers.Count == 0)
            {
                candidatePickers = pickers;
            }
            
            // Sort by floor distance to target floor
            int targetFloorLevel = ParseFloorLevel(floorLevel);
            return candidatePickers.OrderBy(p => 
                Mathf.Abs(ParseFloorLevel(p.CurrentFloor) - targetFloorLevel))
                .ToList();
        }
        #endregion

        #region Status Monitoring
        void Update()
        {
            if (isInitialized)
            {
                MonitorPickerStatus();
            }
        }

        private void MonitorPickerStatus()
        {
            // Monitor picker status (idle time, workload, etc.)
            foreach (var picker in pickers)
            {
                // Future implementation for optimization
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
            if (pickingPoint == null) return new List<PickerController>();
            
            if (pickingPointAssignments.TryGetValue(pickingPoint.Id, out var assignedPickers))
            {
                return assignedPickers;
            }
            return new List<PickerController>();
        }
        
        public List<PickerController> GetPickersOnFloor(string floorLevel)
        {
            if (string.IsNullOrEmpty(floorLevel)) return new List<PickerController>();
            
            if (pickersByFloor.TryGetValue(floorLevel, out var floorPickers))
            {
                return floorPickers;
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
        
        public PickerController GetBestPickerForPoint(PickingPoint pickingPoint, Vector3 location)
        {
            if (pickingPoint == null) return null;
            
            // First try assigned pickers
            List<PickerController> assignedPickers = GetPickersForPoint(pickingPoint);
            List<PickerController> availablePickers = assignedPickers
                .Where(p => p.IsIdle)
                .ToList();
            
            // If no idle assigned pickers, try any picker that has this point in range
            if (availablePickers.Count == 0)
            {
                availablePickers = pickers
                    .Where(p => p.IsPickingPointInRange(pickingPoint))
                    .ToList();
                
                if (availablePickers.Count == 0)
                {
                    // If still no pickers, try any picker with lowest queue
                    return pickers
                        .OrderBy(p => p.QueuedTaskCount)
                        .ThenBy(p => Vector3.Distance(p.CurrentPosition, location))
                        .FirstOrDefault();
                }
            }
            
            // Find best picker based on distance and workload
            return availablePickers
                .OrderBy(p => Vector3.Distance(p.CurrentPosition, location) * (1 + p.QueuedTaskCount * 0.5f))
                .FirstOrDefault();
        }
        
        // For testing - force create pickers on a specific floor
        public void ForceCreatePickersOnFloor(string floorLevel, int count)
        {
            if (!isInitialized)
            {
                Debug.LogWarning("Cannot create pickers: Picker Manager not initialized");
                return;
            }
            
            Debug.Log($"Force creating {count} pickers on floor {floorLevel}");
            int startId = pickers.Count + 1;
            
            for (int i = 0; i < count; i++)
            {
                Vector3 position = GetValidInitialPosition(floorLevel);
                if (position != Vector3.zero)
                {
                    CreatePicker(startId + i, position, floorLevel);
                }
                else
                {
                    Debug.LogError($"Could not find valid position for picker {startId + i} on floor {floorLevel}");
                }
            }
            
            // Reassign picking zones
            AssignPickingZones();
        }
        #endregion
        
        #region Helper Methods
        private int ParseFloorLevel(string floorLevel)
        {
            return Map.FloorLevelUtils.Parse(floorLevel, defaultValue: 1);
        }
        #endregion
    }

    // Helper class for floor distribution setting
    [System.Serializable]
    public class PickerFloorDistribution
    {
        public string floorLevel = "1F";
        [Range(0, 1)]
        public float ratio = 0.5f;
    }
}
