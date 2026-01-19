using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using NetTopologySuite.Geometries;
using NetTopologySuite.Index.Strtree;

namespace Map
{
    /// <summary>
    /// SpatialIndex - Pure spatial index data structure, supports 2D and 3D (multi-floor) queries
    /// </summary>
    public class SpatialIndex
    {
        private STRtree<CellSpace> tree;
        private Dictionary<string, Envelope> cellSpaceEnvelopes;
        private Dictionary<string, HashSet<string>> cellSpaceFloors;
        private bool enable3D;

        /// <summary>
        /// Create spatial index
        /// </summary>
        public SpatialIndex(bool enable3D = false)
        {
            tree = new STRtree<CellSpace>();
            cellSpaceEnvelopes = new Dictionary<string, Envelope>();
            cellSpaceFloors = new Dictionary<string, HashSet<string>>();
            this.enable3D = enable3D;
        }

        /// <summary>
        /// Enable or disable 3D mode
        /// </summary>
        public void SetEnable3D(bool enable)
        {
            this.enable3D = enable;
        }

        /// <summary>
        /// Add CellSpace to spatial index
        /// </summary>
        public void Insert(CellSpace cellSpace)
        {
            if (cellSpace == null || string.IsNullOrEmpty(cellSpace.Id))
                return;

            if (cellSpaceEnvelopes.ContainsKey(cellSpace.Id))
            {
                Debug.LogWarning($"CellSpace {cellSpace.Id} already exists in the index");
                return;
            }

            // Get bounding box of CellSpace
            var envelope = cellSpace.Space.EnvelopeInternal;
            cellSpaceEnvelopes[cellSpace.Id] = envelope;
            
            // Add to STRtree
            tree.Insert(envelope, cellSpace);
            
            // If 3D enabled, store floor information
            if (enable3D)
            {
                // Store floor information (extract only, no processing)
                var floors = new HashSet<string>();
                string locatedFloor = cellSpace.GetLocatedFloor();
                
                if (!string.IsNullOrEmpty(locatedFloor))
                {
                    floors.Add(locatedFloor);
                }
                
                // For vertical connection elements, multiple floors may be associated
                if (cellSpace.IsVerticalConnection())
                {
                    var verticalFeatures = cellSpace.GetVerticalFeatures();
                    if (verticalFeatures?.FloorAssociations != null)
                    {
                        foreach (var association in verticalFeatures.FloorAssociations)
                        {
                            if (!string.IsNullOrEmpty(association.FloorLevel))
                            {
                                floors.Add(association.FloorLevel);
                            }
                        }
                    }
                }
                
                // Save floor information
                cellSpaceFloors[cellSpace.Id] = floors;
            }
        }

        /// <summary>
        /// Remove CellSpace from index
        /// </summary>
        public void Remove(string cellSpaceId)
        {
            if (string.IsNullOrEmpty(cellSpaceId))
                return;
                
            cellSpaceEnvelopes.Remove(cellSpaceId);
            cellSpaceFloors.Remove(cellSpaceId);
            // STRtree does not support direct deletion
        }

        /// <summary>
        /// Clear index
        /// </summary>
        public void Clear()
        {
            tree = new STRtree<CellSpace>();
            cellSpaceEnvelopes.Clear();
            cellSpaceFloors.Clear();
        }

        /// <summary>
        /// Rebuild index
        /// </summary>
        public void RebuildIndex(List<CellSpace> allCellSpaces)
        {
            Clear();
            
            if (allCellSpaces != null)
            {
                foreach (var cellSpace in allCellSpaces)
                {
                    Insert(cellSpace);
                }
            }
        }

        /// <summary>
        /// Find all CellSpaces containing specified point
        /// </summary>
        public List<CellSpace> QueryPoint(Vector2 point)
        {
            // Create bounding box for point query
            var queryEnvelope = new Envelope(point.x, point.x, point.y, point.y);
            
            // Get potential CellSpaces containing this point
            var candidates = tree.Query(queryEnvelope);
            
            // Filter out items removed from index
            var validCandidates = candidates.Where(cell => 
                cellSpaceEnvelopes.ContainsKey(cell.Id)).ToList();
            
            // Create point geometry object
            var searchPoint = new Point(point.x, point.y);
            
            // Exact test (point inside polygon)
            var result = new List<CellSpace>();
            
            foreach (var cell in validCandidates)
            {
                if (cell.Space.Contains(searchPoint))
                {
                    result.Add(cell);
                }
            }
            
            return result;
        }

        /// <summary>
        /// Find all CellSpaces containing specified 3D point
        /// </summary>
        public List<CellSpace> QueryPoint(Vector3 point, string floor)
        {
            if (!enable3D || string.IsNullOrEmpty(floor))
            {
                return QueryPoint(new Vector2(point.x, point.y));
            }
            
            // Create bounding box for point query
            var queryEnvelope = new Envelope(point.x, point.x, point.y, point.y);
            
            // Get potential CellSpaces containing this point
            var candidates = tree.Query(queryEnvelope);
            
            // Filter out items removed from index or not on specified floor
            var validCandidates = candidates.Where(cell => 
                cellSpaceEnvelopes.ContainsKey(cell.Id) && 
                (cellSpaceFloors.TryGetValue(cell.Id, out var floors) && floors.Contains(floor))).ToList();
            
            // Create point geometry object
            var searchPoint = new Point(point.x, point.y);
            
            // Exact test (point inside polygon)
            var result = new List<CellSpace>();
            
            foreach (var cell in validCandidates)
            {
                if (cell.Space.Contains(searchPoint))
                {
                    result.Add(cell);
                }
            }
            
            return result;
        }

        /// <summary>
        /// Find all CellSpaces intersecting with specified bounding box
        /// </summary>
        public List<CellSpace> QueryIntersect(Envelope envelope)
        {
            if (envelope == null)
                return new List<CellSpace>();
                
            // Execute query
            var candidates = tree.Query(envelope);
            
            // Filter out items removed from index
            return candidates.Where(cell => cellSpaceEnvelopes.ContainsKey(cell.Id)).ToList();
        }

        /// <summary>
        /// Find all CellSpaces intersecting with specified bounding box, filtered by floor
        /// </summary>
        public List<CellSpace> QueryIntersect(Envelope envelope, params string[] floors)
        {
            if (envelope == null)
                return new List<CellSpace>();
                
            if (!enable3D || floors == null || floors.Length == 0)
            {
                return QueryIntersect(envelope);
            }
            
            // Execute query
            var candidates = tree.Query(envelope);
            
            // Filter out items removed from index or not on specified floor
            return candidates.Where(cell => 
                cellSpaceEnvelopes.ContainsKey(cell.Id) && 
                cellSpaceFloors.TryGetValue(cell.Id, out var cellFloors) && 
                floors.Any(f => cellFloors.Contains(f))).ToList();
        }

        /// <summary>
        /// Find all CellSpaces intersecting with specified geometry
        /// </summary>
        public List<CellSpace> QueryIntersect(Geometry geometry)
        {
            if (geometry == null)
                return new List<CellSpace>();
                
            var envelope = geometry.EnvelopeInternal;
            var candidatesList = QueryIntersect(envelope);
            
            return candidatesList.Where(cell => cell.Space.Intersects(geometry)).ToList();
        }

        /// <summary>
        /// Find all CellSpaces intersecting with specified geometry, filtered by floor
        /// </summary>
        public List<CellSpace> QueryIntersect(Geometry geometry, params string[] floors)
        {
            if (geometry == null)
                return new List<CellSpace>();
                
            if (!enable3D || floors == null || floors.Length == 0)
            {
                return QueryIntersect(geometry);
            }
            
            var envelope = geometry.EnvelopeInternal;
            var candidatesList = QueryIntersect(envelope, floors);
            
            return candidatesList.Where(cell => cell.Space.Intersects(geometry)).ToList();
        }

        /// <summary>
        /// Find all CellSpaces intersecting with specified path
        /// </summary>
        public List<CellSpace> QueryPath(List<Vector2> path)
        {
            if (path == null || path.Count < 2)
                return new List<CellSpace>();
                
            // Calculate bounding box of path
            double minX = path.Min(p => p.x);
            double minY = path.Min(p => p.y);
            double maxX = path.Max(p => p.x);
            double maxY = path.Max(p => p.y);
            
            // Create bounding box for query
            var queryEnvelope = new Envelope(minX, maxX, minY, maxY);
            
            // Get potentially intersecting CellSpaces
            var candidatesList = QueryIntersect(queryEnvelope);
            
            // Create geometry object representing path
            var geometryFactory = new GeometryFactory();
            var coordinates = new Coordinate[path.Count];
            
            for (int i = 0; i < path.Count; i++)
            {
                coordinates[i] = new Coordinate(path[i].x, path[i].y);
            }
            
            var pathLineString = geometryFactory.CreateLineString(coordinates);
            
            // Exact intersection test
            return candidatesList.Where(cell => cell.Space.Intersects(pathLineString)).ToList();
        }

        /// <summary>
        /// Find all CellSpaces intersecting with specified 3D path
        /// </summary>
        public List<CellSpace> QueryPath(List<Vector3> path, params string[] floors)
        {
            if (path == null || path.Count < 2)
                return new List<CellSpace>();
                
            if (!enable3D || floors == null || floors.Length == 0)
            {
                // Convert to 2D path
                var path2D = path.Select(p => new Vector2(p.x, p.y)).ToList();
                return QueryPath(path2D);
            }
            
            // Calculate bounding box of path
            double minX = path.Min(p => p.x);
            double minY = path.Min(p => p.y);
            double maxX = path.Max(p => p.x);
            double maxY = path.Max(p => p.y);
            
            // Create bounding box for query
            var queryEnvelope = new Envelope(minX, maxX, minY, maxY);
            
            // Get potentially intersecting CellSpaces
            var candidatesList = QueryIntersect(queryEnvelope, floors);
            
            // Create geometry object representing path
            var geometryFactory = new GeometryFactory();
            var coordinates = new Coordinate[path.Count];
            
            for (int i = 0; i < path.Count; i++)
            {
                coordinates[i] = new Coordinate(path[i].x, path[i].y);
            }
            
            var pathLineString = geometryFactory.CreateLineString(coordinates);
            
            // Exact intersection test
            return candidatesList.Where(cell => cell.Space.Intersects(pathLineString)).ToList();
        }
    }
}