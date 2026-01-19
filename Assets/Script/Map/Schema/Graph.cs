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
        public float FloorHeight { get; set; } = 3.0f;

        public Graph(IndoorSpace indoorSpace, float floorHeight = 3.0f)
        {
            this.indoorSpace = indoorSpace;
            RoutePoints = new List<RoutePoint>();
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
            }
        }

        public RoutePoint GetRoutePointFromConnectionPoint(ConnectionPoint connectionPoint)
        {
            foreach (RoutePoint routePoint in RoutePoints)
            {
                if (routePoint.ConnectionPoint.Id == connectionPoint.Id)
                {
                    return routePoint;
                }
            }
            Debug.Log("RoutePoint not found");
            return null;
        }

        public Vector3 GetCoordinatesFromConnectionPoint(ConnectionPoint connectionPoint)
        {
            foreach (var routePoint in RoutePoints)
            {
                ConnectionPoint cp = routePoint.ConnectionPoint;
                if (cp.Id == connectionPoint.Id)
                {
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
            }
            Debug.Log("Coordinates not found");
            return Vector3.zero;
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
        
        // Parses floor level string into an integer (e.g., "1F" -> 1, "B1" -> -1)
        private int ParseFloorLevel(string floorLevel)
        {
            if (string.IsNullOrEmpty(floorLevel))
                return 0;
                
            if (floorLevel.StartsWith("B") || floorLevel.StartsWith("b"))
            {
                // Basement floors
                if (int.TryParse(floorLevel.Substring(1).Replace("F", "").Replace("f", ""), out int level))
                {
                    return -level;
                }
            }
            else
            {
                // Regular floors
                if (int.TryParse(floorLevel.Replace("F", "").Replace("f", ""), out int level))
                {
                    return level;
                }
            }
            
            return 0; // Default
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
                            ConnectionPoint connectionPoint = this.indoorSpace.GetConnectionPointFromCellSpace(cellSpace.Id, sequence);
                            return connectionPoint;
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
                        ConnectionPoint connectionPoint = this.indoorSpace.GetConnectionPointFromCellSpace(cellSpace.Id, sequence);
                        return connectionPoint;
                    }
                }
            }
            
            Debug.Log("Point is not inside any CellSpace");
            return null;
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
            Layer nextLayer = indoorSpace.GetLayerFromCellSpaceId(nextCellSpace.Id);
            Layer pastLayer = indoorSpace.GetLayerFromCellSpaceId(pastCellSpace.Id);

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

            throw new System.Exception($"Unable to determine location for ConnectionPoint {connectionPoint.Id}");
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