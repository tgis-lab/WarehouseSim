using UnityEngine;
using UnityEngine.UI;
using System;
using System.Collections.Generic;
using System.IO;
using Newtonsoft.Json;
using NetTopologySuite.Geometries;
using NetTopologySuite.IO;

namespace Map {
    public class Graph
    {
        public List<RoutePoint> RoutePoints { get; private set; }
        private IndoorSpace indoorSpace;

        public Graph(IndoorSpace indoorSpace)
        {
            this.indoorSpace = indoorSpace;
            RoutePoints = new List<RoutePoint>();
        }

        public void AddNode(RoutePoint node) 
        {
            if (!RoutePoints.Contains(node)) 
            {
                RoutePoints.Add(node);
            }
        }

        public RoutePoint GetRoutePointFormConnectionPoint(ConnectionPoint connectionPoint)
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

        public RoutePoint GetRoutePointFromCoordinate(Vector3 position, bool sequence)
        {
            ConnectionPoint connectionPoint = this.GetConnectionPointFromCoordinates(position, sequence);
            if (connectionPoint != null)
            {
                return this.GetRoutePointFormConnectionPoint(connectionPoint);
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
                        return new Vector3((float)point.X, 0f, (float)point.Y);
                    }
                }
                else{
                    continue;
                }
            }
            Debug.Log("Coordinates not found");
            return Vector3.zero;
        }

        public Vector3 GetCoordinatesFromConnectionPoint(ConnectionPoint connectionPoint)
        {
            foreach (var routePoint in RoutePoints)
            {
                ConnectionPoint cp = routePoint.ConnectionPoint;
                if (cp.Id == connectionPoint.Id)
                {
                    Point point = (Point)connectionPoint.Point;
                    return new Vector3((float)point.X, 0f, (float)point.Y);
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
            Point point = new Point(position.x, position.z);

            foreach (var cellSpace in this.indoorSpace.CellSpaces)
            {
                Polygon polygon = (Polygon)cellSpace.Space;

                if (polygon.Contains(point))
                {
                    ConnectionPoint connectionPoint = this.indoorSpace.GetConnectionPointFromCellSpace(cellSpace.Id, sequence);
                    return connectionPoint;
                }
            }
            Debug.Log("Point is not inside any CellSpace");
            return null;
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
    }
}