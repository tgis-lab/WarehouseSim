using UnityEngine;
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
        private List<PickerController> pickers = new List<PickerController>();
        private Dictionary<string, List<PickerController>> pickingPointAssignments = new Dictionary<string, List<PickerController>>();
        private bool isInitialized = false;
        #endregion

        #region Public Properties
        public int TotalPickers => pickers.Count;
        public int AvailablePickers => pickers.Count(p => p.IsIdle);
        #endregion

        #region Initialization
        public void InitializePickers(IndoorSpace indoorSpace)
        {
            if (isInitialized)
            {
                Debug.LogWarning("Picker Manager is already initialized.");
                return;
            }

            this.indoorSpace = indoorSpace;
            CreatePickers();
            AssignPickingZones();
            isInitialized = true;

            Debug.Log($"Initialized {pickers.Count} pickers with assigned zones");
        }

        private void CreatePickers()
        {
            List<Vector3> basePositions = CalculatePickerBasePositions();

            for (int i = 0; i < numberOfPickers; i++)
            {
                if (i >= basePositions.Count)
                {
                    Debug.LogError($"Not enough valid base positions for picker {i + 1}");
                    continue;
                }

                Vector3 basePosition = basePositions[i];
                GameObject pickerObj = Instantiate(pickerPrefab, basePosition, Quaternion.identity, pickersParent);
                pickerObj.name = $"Picker_{i + 1}";

                PickerController picker = pickerObj.GetComponent<PickerController>();
                if (picker != null)
                {
                    picker.Initialize(i + 1, indoorSpace, basePosition);
                    pickers.Add(picker);
                }
                else
                {
                    Debug.LogError($"Failed to get PickerController component for picker {i + 1}");
                }
            }
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
                        0,
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
            List<PickingPoint> allPickingPoints = indoorSpace.PickingPoints;
            pickingPointAssignments.Clear();

            // First, assign picking points to their nearest pickers
            foreach (var pickingPoint in allPickingPoints)
            {
                var eligiblePickers = pickers
                    .Where(p => p.IsPickingPointInRange(pickingPoint))
                    .ToList();

                if (eligiblePickers.Any())
                {
                    pickingPointAssignments[pickingPoint.Id] = eligiblePickers;
                }
                else
                {
                    Debug.LogWarning($"No eligible pickers found for picking point {pickingPoint.Id}");
                }
            }

            // Then update each picker's assigned points
            foreach (var picker in pickers)
            {
                var assignedPoints = allPickingPoints
                    .Where(p => picker.IsPickingPointInRange(p))
                    .ToList();

                picker.AssignPickingPoints(assignedPoints);

                if (assignedPoints.Count == 0)
                {
                    Debug.LogWarning($"Picker {picker.Id} has no assigned picking points");
                }
                else
                {
                    Debug.Log($"Picker {picker.Id} assigned to {assignedPoints.Count} picking points");
                }
            }
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