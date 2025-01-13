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

        public void InitializeIntersectionAreas(IndoorSpace indoorSpace)
        {
            this.indoorSpace = indoorSpace;

            _intersectionAreas = new Dictionary<string, IntersectionArea>();
            var processedCells = new HashSet<string>();
            var intersectionLayers = indoorSpace.GetLayerFromType("intersection");

            foreach (var layer in intersectionLayers)
            {
                foreach (var cellId in layer.CellSpaces)
                {
                    if (processedCells.Contains(cellId))
                        continue;

                    CreateIntersectionArea(cellId, processedCells);
                }
            }

            _isInitialized = true;
            Debug.Log($"Successfully initialized {_intersectionAreas.Count} intersection areas");
        }

        private void CreateIntersectionArea(string startCellId, HashSet<string> processedCells)
        {
            var intersectionArea = new IntersectionArea($"intersection_{_intersectionAreas.Count + 1}");
            var queue = new Queue<string>();
            
            queue.Enqueue(startCellId);
            processedCells.Add(startCellId);

            while (queue.Count > 0)
            {
                var currentCellId = queue.Dequeue();
                var currentCell = indoorSpace.GetCellSpaceFromId(currentCellId);
                intersectionArea.AddIntersectionCell(currentCell);

                ProcessConnectedCells(currentCellId, intersectionArea, queue, processedCells);
            }

            if (intersectionArea.IntersectionCells.Count > 0)
            {
                _intersectionAreas.Add(intersectionArea.Id, intersectionArea);
            }
        }

        private void ProcessConnectedCells(string currentCellId, IntersectionArea intersectionArea, 
            Queue<string> queue, HashSet<string> processedCells)
        {
            var boundaries = indoorSpace.CellBoundaries.Where(b => 
                b.Source == currentCellId || b.Target == currentCellId);

            foreach (var boundary in boundaries)
            {
                var nextCellId = boundary.Source == currentCellId ? boundary.Target : boundary.Source;
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
                    var connectionPoint = boundary.ConvertToConnectionPoint();
                    if (boundary.Source == currentCellId)
                    {
                        intersectionArea.AddExitPoint(connectionPoint);
                    }
                    else
                    {
                        intersectionArea.AddEntryPoint(connectionPoint);
                    }
                }
            }
        }

        public IntersectionArea GetIntersectionAtPosition(Vector3 position)
        {
            if (!_isInitialized)
            {
                Debug.LogError("Intersection areas not initialized");
                return null;
            }

            foreach (var intersection in _intersectionAreas.Values)
            {
                if (intersection.Contains(position))
                {
                    return intersection;
                }
            }
            return null;
        }

        public bool ShouldActivateDWA(Vector3 position, out IntersectionArea activeIntersection)
        {
            activeIntersection = GetNearestIntersection(position);
            if (activeIntersection == null)
                return false;

            return activeIntersection.ShouldActivateDWA(position);
        }

        private IntersectionArea GetNearestIntersection(Vector3 position)
        {
            if (!_isInitialized)
            {
                Debug.LogError("Intersection areas not initialized");
                return null;
            }

            IntersectionArea nearestIntersection = null;
            float minDistance = float.MaxValue;

            foreach (var intersection in _intersectionAreas.Values)
            {
                if (intersection.IsNear(position, intersection.ActivationDistance))
                {
                    var distance = Vector3.Distance(position, 
                        new Vector3((float)intersection.IntersectionGeometry.Centroid.X, 0, 
                            (float)intersection.IntersectionGeometry.Centroid.Y));
                    
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
                return null;
            }
            return _intersectionAreas;
        }
    }
}