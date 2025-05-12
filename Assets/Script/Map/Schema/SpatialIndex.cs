using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using NetTopologySuite.Geometries;
using NetTopologySuite.Index.Strtree;

namespace Map
{
    /// <summary>
    /// SpatialIndex - 纯粹的空间索引数据结构，支持2D和3D(多楼层)查询
    /// </summary>
    public class SpatialIndex
    {
        private STRtree<CellSpace> tree;
        private Dictionary<string, Envelope> cellSpaceEnvelopes;
        private Dictionary<string, HashSet<string>> cellSpaceFloors;
        private bool enable3D;

        /// <summary>
        /// 创建空间索引
        /// </summary>
        public SpatialIndex(bool enable3D = false)
        {
            tree = new STRtree<CellSpace>();
            cellSpaceEnvelopes = new Dictionary<string, Envelope>();
            cellSpaceFloors = new Dictionary<string, HashSet<string>>();
            this.enable3D = enable3D;
        }

        /// <summary>
        /// 启用或禁用3D模式
        /// </summary>
        public void SetEnable3D(bool enable)
        {
            this.enable3D = enable;
        }

        /// <summary>
        /// 将CellSpace添加到空间索引
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

            // 获取CellSpace的边界框
            var envelope = cellSpace.Space.EnvelopeInternal;
            cellSpaceEnvelopes[cellSpace.Id] = envelope;
            
            // 添加到STRtree
            tree.Insert(envelope, cellSpace);
            
            // 如果启用3D，存储楼层信息
            if (enable3D)
            {
                // 存储楼层信息（仅提取不处理）
                var floors = new HashSet<string>();
                string locatedFloor = cellSpace.GetLocatedFloor();
                
                if (!string.IsNullOrEmpty(locatedFloor))
                {
                    floors.Add(locatedFloor);
                }
                
                // 对于垂直连接元素，可能关联多个楼层
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
                
                // 保存楼层信息
                cellSpaceFloors[cellSpace.Id] = floors;
            }
        }

        /// <summary>
        /// 从索引中移除CellSpace
        /// </summary>
        public void Remove(string cellSpaceId)
        {
            if (string.IsNullOrEmpty(cellSpaceId))
                return;
                
            cellSpaceEnvelopes.Remove(cellSpaceId);
            cellSpaceFloors.Remove(cellSpaceId);
            // STRtree不支持直接删除
        }

        /// <summary>
        /// 清空索引
        /// </summary>
        public void Clear()
        {
            tree = new STRtree<CellSpace>();
            cellSpaceEnvelopes.Clear();
            cellSpaceFloors.Clear();
        }

        /// <summary>
        /// 重建索引
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
        /// 查找包含指定点的所有CellSpace
        /// </summary>
        public List<CellSpace> QueryPoint(Vector2 point)
        {
            // 创建点查询的包围盒
            var queryEnvelope = new Envelope(point.x, point.x, point.y, point.y);
            
            // 获取潜在的包含此点的CellSpace
            var candidates = tree.Query(queryEnvelope);
            
            // 过滤掉已从索引中删除的项
            var validCandidates = candidates.Where(cell => 
                cellSpaceEnvelopes.ContainsKey(cell.Id)).ToList();
            
            // 创建点几何对象
            var searchPoint = new Point(point.x, point.y);
            
            // 精确测试（点在多边形内）
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
        /// 查找包含指定3D点的所有CellSpace
        /// </summary>
        public List<CellSpace> QueryPoint(Vector3 point, string floor)
        {
            if (!enable3D || string.IsNullOrEmpty(floor))
            {
                return QueryPoint(new Vector2(point.x, point.y));
            }
            
            // 创建点查询的包围盒
            var queryEnvelope = new Envelope(point.x, point.x, point.y, point.y);
            
            // 获取潜在的包含此点的CellSpace
            var candidates = tree.Query(queryEnvelope);
            
            // 过滤掉已从索引中删除的项和不在指定楼层的项
            var validCandidates = candidates.Where(cell => 
                cellSpaceEnvelopes.ContainsKey(cell.Id) && 
                (cellSpaceFloors.TryGetValue(cell.Id, out var floors) && floors.Contains(floor))).ToList();
            
            // 创建点几何对象
            var searchPoint = new Point(point.x, point.y);
            
            // 精确测试（点在多边形内）
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
        /// 查找与指定边界框相交的所有CellSpace
        /// </summary>
        public List<CellSpace> QueryIntersect(Envelope envelope)
        {
            if (envelope == null)
                return new List<CellSpace>();
                
            // 执行查询
            var candidates = tree.Query(envelope);
            
            // 过滤掉已从索引中删除的项
            return candidates.Where(cell => cellSpaceEnvelopes.ContainsKey(cell.Id)).ToList();
        }

        /// <summary>
        /// 查找与指定边界框相交的所有CellSpace，按楼层过滤
        /// </summary>
        public List<CellSpace> QueryIntersect(Envelope envelope, params string[] floors)
        {
            if (envelope == null)
                return new List<CellSpace>();
                
            if (!enable3D || floors == null || floors.Length == 0)
            {
                return QueryIntersect(envelope);
            }
            
            // 执行查询
            var candidates = tree.Query(envelope);
            
            // 过滤掉已从索引中删除的项和不在指定楼层的项
            return candidates.Where(cell => 
                cellSpaceEnvelopes.ContainsKey(cell.Id) && 
                cellSpaceFloors.TryGetValue(cell.Id, out var cellFloors) && 
                floors.Any(f => cellFloors.Contains(f))).ToList();
        }

        /// <summary>
        /// 查找与指定几何图形相交的所有CellSpace
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
        /// 查找与指定几何图形相交的所有CellSpace，按楼层过滤
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
        /// 查找与指定路径相交的所有CellSpace
        /// </summary>
        public List<CellSpace> QueryPath(List<Vector2> path)
        {
            if (path == null || path.Count < 2)
                return new List<CellSpace>();
                
            // 计算路径的包围盒
            double minX = path.Min(p => p.x);
            double minY = path.Min(p => p.y);
            double maxX = path.Max(p => p.x);
            double maxY = path.Max(p => p.y);
            
            // 创建查询的包围盒
            var queryEnvelope = new Envelope(minX, maxX, minY, maxY);
            
            // 获取可能相交的CellSpace
            var candidatesList = QueryIntersect(queryEnvelope);
            
            // 创建表示路径的几何对象
            var geometryFactory = new GeometryFactory();
            var coordinates = new Coordinate[path.Count];
            
            for (int i = 0; i < path.Count; i++)
            {
                coordinates[i] = new Coordinate(path[i].x, path[i].y);
            }
            
            var pathLineString = geometryFactory.CreateLineString(coordinates);
            
            // 精确测试相交
            return candidatesList.Where(cell => cell.Space.Intersects(pathLineString)).ToList();
        }

        /// <summary>
        /// 查找与指定3D路径相交的所有CellSpace
        /// </summary>
        public List<CellSpace> QueryPath(List<Vector3> path, params string[] floors)
        {
            if (path == null || path.Count < 2)
                return new List<CellSpace>();
                
            if (!enable3D || floors == null || floors.Length == 0)
            {
                // 转换为2D路径
                var path2D = path.Select(p => new Vector2(p.x, p.y)).ToList();
                return QueryPath(path2D);
            }
            
            // 计算路径的包围盒
            double minX = path.Min(p => p.x);
            double minY = path.Min(p => p.y);
            double maxX = path.Max(p => p.x);
            double maxY = path.Max(p => p.y);
            
            // 创建查询的包围盒
            var queryEnvelope = new Envelope(minX, maxX, minY, maxY);
            
            // 获取可能相交的CellSpace
            var candidatesList = QueryIntersect(queryEnvelope, floors);
            
            // 创建表示路径的几何对象
            var geometryFactory = new GeometryFactory();
            var coordinates = new Coordinate[path.Count];
            
            for (int i = 0; i < path.Count; i++)
            {
                coordinates[i] = new Coordinate(path[i].x, path[i].y);
            }
            
            var pathLineString = geometryFactory.CreateLineString(coordinates);
            
            // 精确测试相交
            return candidatesList.Where(cell => cell.Space.Intersects(pathLineString)).ToList();
        }
    }
}