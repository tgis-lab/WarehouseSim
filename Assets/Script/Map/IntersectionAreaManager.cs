using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using CentralControl;

namespace Map
{
    public class IntersectionAreaManager : MonoBehaviour
    {
        public IndoorSpace indoorSpace;

        private Dictionary<string, IntersectionArea> _intersectionAreas;
        private bool _isInitialized;
        private float _floorHeight = 3.0f;
        private float _baseHeight = 0.05f;

        public void InitializeIntersectionAreas(IndoorSpace indoorSpace, float floorHeight = 3.0f, float baseHeight = 0.05f)
        {
            this.indoorSpace = indoorSpace;
            this._floorHeight = floorHeight;
            this._baseHeight = baseHeight;

            _intersectionAreas = new Dictionary<string, IntersectionArea>();
            var processedCells = new HashSet<string>();
            var intersectionLayers = indoorSpace.GetLayerFromType("intersection");

            // Debug.Log($"Found {intersectionLayers.Count} intersection layers");

            foreach (var layer in intersectionLayers)
            {
                // Debug.Log($"Processing {layer.CellSpaces.Count} cells in layer {layer.Id}");
                foreach (var cellId in layer.CellSpaces)
                {
                    if (processedCells.Contains(cellId))
                        continue;

                    CreateIntersectionArea(cellId, processedCells);
                }
            }

            _isInitialized = true;
            // Debug.Log($"Successfully initialized {_intersectionAreas.Count} intersection areas");
        }

        private void CreateIntersectionArea(string startCellId, HashSet<string> processedCells)
        {
            var intersectionArea = new IntersectionArea($"intersection_{_intersectionAreas.Count + 1}");
            var queue = new Queue<string>();
            
            queue.Enqueue(startCellId);
            processedCells.Add(startCellId);

            // Record the floor of this intersection area
            string floorLevel = "1F"; // Default value
            var startCell = indoorSpace.GetCellSpaceFromId(startCellId);
            if (startCell != null)
            {
                floorLevel = startCell.GetLocatedFloor();
            }
            intersectionArea.FloorLevel = floorLevel;

            // Debug.Log($"Creating intersection area, start cell: {startCellId}, floor: {floorLevel}");

            while (queue.Count > 0)
            {
                var currentCellId = queue.Dequeue();
                var currentCell = indoorSpace.GetCellSpaceFromId(currentCellId);
                
                if (currentCell != null)
                {
                    intersectionArea.AddIntersectionCell(currentCell);
                }
                else
                {
                    Debug.LogWarning($"Unable to find cell {currentCellId}");
                    continue;
                }

                ProcessConnectedCells(currentCellId, intersectionArea, queue, processedCells);
            }

            if (intersectionArea.IntersectionCells.Count > 0)
            {
                _intersectionAreas.Add(intersectionArea.Id, intersectionArea);
                // Debug.Log($"Adding intersection area {intersectionArea.Id}, containing {intersectionArea.IntersectionCells.Count} cells");
            }
        }

        private void ProcessConnectedCells(string currentCellId, IntersectionArea intersectionArea, 
            Queue<string> queue, HashSet<string> processedCells)
        {
            var boundaries = indoorSpace.CellBoundaries.Where(b => 
                b.Source == currentCellId || b.Target == currentCellId);

            foreach (var boundary in boundaries)
            {
                // Skip vertical boundaries
                if (boundary.IsVertical())
                {
                    continue;
                }

                var nextCellId = boundary.Source == currentCellId ? boundary.Target : boundary.Source;
                var nextCell = indoorSpace.GetCellSpaceFromId(nextCellId);
                
                if (nextCell == null)
                {
                    Debug.LogWarning($"Unable to find adjacent cell {nextCellId}");
                    continue;
                }
                
                // Ensure adjacent cells are on the same floor
                string nextCellFloor = nextCell.GetLocatedFloor();
                if (nextCellFloor != intersectionArea.FloorLevel)
                {
                    continue;
                }

                var nextCellLayer = indoorSpace.GetLayerFromCellSpaceId(nextCellId);

                if (nextCellLayer != null && nextCellLayer.IsIntersection())
                {
                    if (!processedCells.Contains(nextCellId))
                    {
                        queue.Enqueue(nextCellId);
                        processedCells.Add(nextCellId);
                    }
                }
                else
                {
                    var connectionPoint = indoorSpace.GetConnectionPointFromCellBoundary(boundary.Id);
                    if (connectionPoint != null)
                    {
                        var waypoint = Waypoint.FromConnectionPoint(connectionPoint, indoorSpace, _floorHeight, _baseHeight);
                        if (boundary.Source == currentCellId)
                        {
                            intersectionArea.AddExitPoint(waypoint);
                        }
                        else
                        {
                            intersectionArea.AddEntryPoint(waypoint);
                        }
                    }
                }
            }
        }

        public bool IsRobotInIntersection(int robotId)
        {
            if (!_isInitialized)
            {
                Debug.LogWarning("Intersection areas not initialized");
                return false;
            }

            // Get robot position
            GameObject robotObj = GameObject.Find($"Robot_{robotId}");
            if (robotObj == null)
            {
                Debug.LogWarning($"Cannot find robot {robotId}");
                return false;
            }
                
            Vector3 robotPosition = robotObj.transform.position;
            
            // Extract floor information
            string floorLevel = GetFloorLevelFromPosition(robotPosition);
            
            // Check if robot is inside any intersection area
            foreach (var intersection in _intersectionAreas.Values)
            {
                // Only check intersection areas on the same floor
                if (intersection.FloorLevel == floorLevel && intersection.Contains(robotPosition))
                {
                    return true;
                }
            }
            
            return false;
        }

        public bool IsNearIntersection(Vector3 position, float activationDistance)
        {
            if (!_isInitialized)
            {
                Debug.LogWarning("Intersection areas not initialized");
                return false;
            }

            string floorLevel = GetFloorLevelFromPosition(position);

            foreach (var intersection in _intersectionAreas.Values)
            {
                // Only check intersection areas on the same floor
                if (intersection.FloorLevel == floorLevel && intersection.IsNear(position, activationDistance))
                {
                    return true;
                }
            }
            
            return false;
        }

        public IntersectionArea GetIntersectionAtPosition(Vector3 position)
        {
            if (!_isInitialized)
            {
                Debug.LogWarning("Intersection areas not initialized");
                return null;
            }

            string floorLevel = GetFloorLevelFromPosition(position);

            foreach (var intersection in _intersectionAreas.Values)
            {
                // Only check intersection areas on the same floor
                if (intersection.FloorLevel == floorLevel && intersection.Contains(position))
                {
                    return intersection;
                }
            }
            
            return null;
        }

        // Maintain interface compatibility but remove DWA logic
        public bool ShouldActivateDWA(Vector3 position, out IntersectionArea activeIntersection)
        {
            activeIntersection = GetNearestIntersection(position);
            return activeIntersection != null;
        }

        private IntersectionArea GetNearestIntersection(Vector3 position)
        {
            if (!_isInitialized)
            {
                Debug.LogError("Intersection areas not initialized");
                return null;
            }

            string floorLevel = GetFloorLevelFromPosition(position);
            IntersectionArea nearestIntersection = null;
            float minDistance = float.MaxValue;

            foreach (var intersection in _intersectionAreas.Values)
            {
                // Only check intersection areas on the same floor
                if (intersection.FloorLevel != floorLevel)
                    continue;
                    
                if (intersection.IsNear(position, intersection.ActivationDistance))
                {
                    float y = position.y; // Keep original height
                    var centroidPos = new Vector3(
                        (float)intersection.IntersectionGeometry.Centroid.X, 
                        y, 
                        (float)intersection.IntersectionGeometry.Centroid.Y);
                    
                    var distance = Vector3.Distance(position, centroidPos);
                    
                    if (distance < minDistance)
                    {
                        minDistance = distance;
                        nearestIntersection = intersection;
                    }
                }
            }

            return nearestIntersection;
        }

        public Dictionary<string, IntersectionArea> GetAllIntersections()
        {
            if (!_isInitialized)
            {
                Debug.LogError("Intersection areas not initialized");
                return new Dictionary<string, IntersectionArea>();
            }
            return _intersectionAreas;
        }
        
        // Get floor level based on Y coordinate
        private string GetFloorLevelFromPosition(Vector3 position)
        {
            if (indoorSpace == null)
                return "1F"; // Default
                
            float floorHeight = 3.0f; // Default floor height
            
            // Try to get floor height setting from CentralController
            var centralController = FindObjectOfType<CentralControl.CentralController>();
            if (centralController != null)
            {
                floorHeight = centralController.floorHeight;
            }
            
            // Calculate floor number
            int floorNumber = Mathf.RoundToInt(position.y / floorHeight);
            
            if (floorNumber <= 0)
                return "1F"; // Default to 1F for ground or basement
                
            return $"{floorNumber + 1}F"; // +1 because 1F is usually the lowest floor
        }
    }
}