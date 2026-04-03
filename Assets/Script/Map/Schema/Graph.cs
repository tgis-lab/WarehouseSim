using UnityEngine;
using UnityEngine.UI;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using Newtonsoft.Json;
using NetTopologySuite.Geometries;
using NetTopologySuite.IO;

namespace Map {
    public class Graph
    {
        public List<RoutePoint> RoutePoints { get; private set; }
        private IndoorSpace indoorSpace;
        private Dictionary<string, RoutePoint> _routePointIndex;
        public float FloorHeight { get; set; } = 3.0f;

        public Graph(IndoorSpace indoorSpace, float floorHeight = 3.0f)
        {
            this.indoorSpace = indoorSpace;
            RoutePoints = new List<RoutePoint>();
            _routePointIndex = new Dictionary<string, RoutePoint>();
            this.FloorHeight = floorHeight;
        }
        
        private float GetFloorHeight(string floorLevel)
        {
            int level = ParseFloorLevel(floorLevel);
            if (level > 0)
            {
                return (level - 1) * FloorHeight;
            }
            return level * FloorHeight;
        }

        public void AddNode(RoutePoint node)
        {
            if (!RoutePoints.Contains(node))
            {
                RoutePoints.Add(node);
                if (node.ConnectionPoint != null)
                    _routePointIndex[node.ConnectionPoint.Id] = node;
            }
        }

        public RoutePoint GetRoutePointFromConnectionPoint(ConnectionPoint connectionPoint)
        {
            if (connectionPoint == null) return null;
            if (_routePointIndex.TryGetValue(connectionPoint.Id, out var rp))
                return rp;
            Debug.Log("RoutePoint not found");
            return null;
        }

        public Vector3 GetCoordinatesFromConnectionPoint(ConnectionPoint connectionPoint)
        {
            if (connectionPoint == null || !_routePointIndex.ContainsKey(connectionPoint.Id))
            {
                Debug.Log("Coordinates not found");
                return Vector3.zero;
            }

            Point point = (Point)connectionPoint.Point;
            float y = 0f; // Default height

            // Determine the floor where the connection point is located
            string floorLevel = GetFloorLevelForConnectionPoint(connectionPoint);
            if (!string.IsNullOrEmpty(floorLevel))
            {
                y = GetFloorHeight(floorLevel);
            }

            return new Vector3((float)point.X, y, (float)point.Y);
        }
        
        // Gets the floor where the connection point is located
        private string GetFloorLevelForConnectionPoint(ConnectionPoint connectionPoint)
        {
            // Special handling for vertical connection points
            if (connectionPoint.IsVertical())
            {
                CellSpace sourceCell = indoorSpace.GetCellSpaceFromId(connectionPoint.CellBoundary.Source);
                CellSpace targetCell = indoorSpace.GetCellSpaceFromId(connectionPoint.CellBoundary.Target);
                
                if (sourceCell != null && targetCell != null)
                {
                    string sourceFloor = sourceCell.GetLocatedFloor();
                    string targetFloor = targetCell.GetLocatedFloor();
                    
                    // Returns the floor with the smaller number (usually the lower floor)
                    if (!string.IsNullOrEmpty(sourceFloor) && !string.IsNullOrEmpty(targetFloor))
                    {
                        int sourceFloorNum = ParseFloorLevel(sourceFloor);
                        int targetFloorNum = ParseFloorLevel(targetFloor);
                        
                        return sourceFloorNum <= targetFloorNum ? sourceFloor : targetFloor;
                    }
                }
            }
            
            // For regular connection points, get the floor of the connected CellSpace
            CellSpace cell = indoorSpace.GetCellSpaceFromConnectionPoint(connectionPoint, true);
            if (cell != null)
            {
                return cell.GetLocatedFloor();
            }
            
            return null;
        }
        
        private int ParseFloorLevel(string floorLevel)
        {
            return FloorLevelUtils.Parse(floorLevel);
        }

        public RoutePoint GetRoutePointFromCoordinate(Vector3 position, bool sequence)
        {
            ConnectionPoint connectionPoint = this.GetConnectionPointFromCoordinates(position, sequence);
            if (connectionPoint != null)
            {
                return this.GetRoutePointFromConnectionPoint(connectionPoint);
            }
            Debug.Log("Point is not inside any CellSpace");
            return null;
        }

        public Vector3 GetCellSpaceCoordinatesFromConnectionPoint(ConnectionPoint connectionPoint, bool sequence = true)
        {
            foreach (var cellSpace in this.indoorSpace.CellSpaces)
            {
                if ((bool)cellSpace.Properties["navigable"])
                {
                    ConnectionPoint cp = this.indoorSpace.GetConnectionPointFromCellSpace(cellSpace.Id, sequence);
                    if (cp.Id == connectionPoint.Id)
                    {
                        Point point = (Point)cellSpace.Node;
                        float y = 0f;
                        
                        // Get the world height of the floor where the CellSpace is located
                        string floorLevel = cellSpace.GetLocatedFloor();
                        if (!string.IsNullOrEmpty(floorLevel))
                        {
                            y = GetFloorHeight(floorLevel);
                        }
                        
                        return new Vector3((float)point.X, y, (float)point.Y);
                    }
                }
                else{
                    continue;
                }
            }
            Debug.Log("Coordinates not found");
            return Vector3.zero;
        }

        public Layer GetLayerFromConnectionPoint(ConnectionPoint connectionPoint, bool sequence)
        {
            CellSpace cellSpace = this.indoorSpace.GetCellSpaceFromConnectionPoint(connectionPoint, sequence);
            foreach (var layer in this.indoorSpace.Layers)
            {
                if (layer.CellSpaces.Contains(cellSpace.Id))
                {
                    return layer;
                }
            }
            Debug.Log(connectionPoint.Id + "Layer not found");
            return null;
        }

        public ConnectionPoint GetConnectionPointFromCoordinates(Vector3 position, bool sequence)
        {
            // Estimate floor
            string floorLevel = GetFloorLevelFromPosition(position);
            Point point = new Point(position.x, position.z);
            CellSpace foundCellSpace = null;

            // Search in the estimated floor first
            if (!string.IsNullOrEmpty(floorLevel))
            {
                var cellSpacesOnFloor = indoorSpace.GetCellSpacesOnFloor(floorLevel);
                foreach (var cellSpace in cellSpacesOnFloor)
                {
                    if (cellSpace.IsNavigable())
                    {
                        Polygon polygon = (Polygon)cellSpace.Space;
                        if (polygon.Contains(point))
                        {
                            foundCellSpace = cellSpace;
                            ConnectionPoint connectionPoint = this.indoorSpace.GetConnectionPointFromCellSpace(cellSpace.Id, sequence);
                            if (connectionPoint != null)
                            {
                                return connectionPoint;
                            }
                            // If ConnectionPoint is null, continue searching other CellSpaces
                            Debug.Log($"CellSpace {cellSpace.Id} contains point but has no ConnectionPoint (sequence={sequence}), searching alternatives...");
                        }
                    }
                }
            }

            // If not found, try all CellSpaces
            foreach (var cellSpace in this.indoorSpace.CellSpaces)
            {
                if (cellSpace.IsNavigable())
                {
                    Polygon polygon = (Polygon)cellSpace.Space;
                    if (polygon.Contains(point))
                    {
                        foundCellSpace = cellSpace;
                        ConnectionPoint connectionPoint = this.indoorSpace.GetConnectionPointFromCellSpace(cellSpace.Id, sequence);
                        if (connectionPoint != null)
                        {
                            return connectionPoint;
                        }
                        // If ConnectionPoint is null, continue searching
                    }
                }
            }

            // If we found a CellSpace but no ConnectionPoint, try finding the nearest navigable CellSpace with a ConnectionPoint
            if (foundCellSpace != null)
            {
                Debug.LogWarning($"Position {position} is in CellSpace {foundCellSpace.Id} but no ConnectionPoint found. Trying nearest CellSpace with valid ConnectionPoint...");
                return FindNearestConnectionPoint(position, floorLevel, sequence);
            }

            Debug.LogWarning($"Position {position} is not inside any navigable CellSpace on floor {floorLevel}");
            return null;
        }

        /// <summary>
        /// Find the nearest ConnectionPoint to the given position
        /// </summary>
        private ConnectionPoint FindNearestConnectionPoint(Vector3 position, string floorLevel, bool sequence)
        {
            float minDistance = float.MaxValue;
            ConnectionPoint nearestConnectionPoint = null;

            var cellSpacesToSearch = string.IsNullOrEmpty(floorLevel)
                ? indoorSpace.CellSpaces
                : indoorSpace.GetCellSpacesOnFloor(floorLevel);

            foreach (var cellSpace in cellSpacesToSearch)
            {
                if (!cellSpace.IsNavigable()) continue;

                ConnectionPoint cp = indoorSpace.GetConnectionPointFromCellSpace(cellSpace.Id, sequence);
                if (cp == null) continue;

                Point cellCenter = (Point)cellSpace.Node;
                float distance = Vector2.Distance(
                    new Vector2(position.x, position.z),
                    new Vector2((float)cellCenter.X, (float)cellCenter.Y)
                );

                if (distance < minDistance)
                {
                    minDistance = distance;
                    nearestConnectionPoint = cp;
                }
            }

            if (nearestConnectionPoint != null)
            {
                Debug.Log($"Found nearest ConnectionPoint at distance {minDistance:F2}m");
            }

            return nearestConnectionPoint;
        }
        
        // Estimate potential floor from height
        private string GetFloorLevelFromPosition(Vector3 position)
        {
            float baseHeight = 0.05f;
            float floorHeight = FloorHeight;

            int floorLevel = Mathf.RoundToInt((position.y - baseHeight) / floorHeight) + 1;
            
            if (floorLevel <= 0)
            {
                // Basement floors
                return $"B{Math.Abs(floorLevel)}F";
            }
            else
            {
                // Regular floors
                return $"{floorLevel}F";
            }
        }

        public enum PointLocation
        {
            InAisle, 
            ApproachingIntersection,
            InIntersection,
            OutIntersection,
        }

        public PointLocation GetPointLocation(ConnectionPoint connectionPoint, bool sequence = false)
        {
            CellSpace nextCellSpace = indoorSpace.GetCellSpaceFromConnectionPoint(connectionPoint, sequence);
            CellSpace pastCellSpace = indoorSpace.GetCellSpaceFromConnectionPoint(connectionPoint, !sequence);

            if (nextCellSpace == null || pastCellSpace == null)
            {
                return PointLocation.InAisle;
            }

            Layer nextLayer = indoorSpace.GetLayerFromCellSpaceId(nextCellSpace.Id);
            Layer pastLayer = indoorSpace.GetLayerFromCellSpaceId(pastCellSpace.Id);

            if (nextLayer == null || pastLayer == null)
            {
                return PointLocation.InAisle;
            }

            if (pastLayer.IsAisle() && nextLayer.IsAisle())
            {
                return PointLocation.InAisle;
            }
            else if (!pastLayer.IsIntersection() && nextLayer.IsIntersection())
            {
                return PointLocation.ApproachingIntersection;
            }
            else if (pastLayer.IsIntersection() && nextLayer.IsIntersection())
            {
                return PointLocation.InIntersection;
            }
            else if (pastLayer.IsIntersection() && !nextLayer.IsIntersection())
            {
                return PointLocation.OutIntersection;
            }

            return PointLocation.InAisle;
        }
        
        // Check if two connection points are on the same floor
        public bool AreOnSameFloor(ConnectionPoint cp1, ConnectionPoint cp2)
        {
            string floor1 = GetFloorLevelForConnectionPoint(cp1);
            string floor2 = GetFloorLevelForConnectionPoint(cp2);
            
            return !string.IsNullOrEmpty(floor1) && !string.IsNullOrEmpty(floor2) && floor1 == floor2;
        }
        
        // Get the floor difference between two connection points
        public int GetFloorDifference(ConnectionPoint cp1, ConnectionPoint cp2)
        {
            string floor1 = GetFloorLevelForConnectionPoint(cp1);
            string floor2 = GetFloorLevelForConnectionPoint(cp2);
            
            if (string.IsNullOrEmpty(floor1) || string.IsNullOrEmpty(floor2))
                return 0;
                
            int floor1Num = ParseFloorLevel(floor1);
            int floor2Num = ParseFloorLevel(floor2);
            
            return Math.Abs(floor1Num - floor2Num);
        }
        
        // Find vertical connection points between two floors
        public List<ConnectionPoint> FindVerticalConnectionsBetweenFloors(string fromFloor, string toFloor)
        {
            List<ConnectionPoint> result = new List<ConnectionPoint>();
            
            foreach (var routePoint in RoutePoints)
            {
                ConnectionPoint cp = routePoint.ConnectionPoint;
                if (cp.IsVertical())
                {
                    CellSpace sourceCell = indoorSpace.GetCellSpaceFromId(cp.CellBoundary.Source);
                    CellSpace targetCell = indoorSpace.GetCellSpaceFromId(cp.CellBoundary.Target);
                    
                    if (sourceCell != null && targetCell != null)
                    {
                        string sourceFloorLevel = sourceCell.GetLocatedFloor();
                        string targetFloorLevel = targetCell.GetLocatedFloor();
                        
                        if ((sourceFloorLevel == fromFloor && targetFloorLevel == toFloor) ||
                            (sourceFloorLevel == toFloor && targetFloorLevel == fromFloor))
                        {
                            result.Add(cp);
                        }
                    }
                }
            }
            
            return result;
        }
    }
}