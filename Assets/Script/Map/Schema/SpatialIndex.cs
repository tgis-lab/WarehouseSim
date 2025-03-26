using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using NetTopologySuite.Geometries;
using NetTopologySuite.Index.Strtree;

namespace Map
{
    /// <summary>
    /// 使用NetTopologySuite的STRtree实现的空间索引
    /// </summary>
    public class SpatialIndex
    {
        private STRtree<CellSpace> tree;
        private readonly Dictionary<string, Envelope> cellSpaceEnvelopes;

        public SpatialIndex()
        {
            tree = new STRtree<CellSpace>();
            cellSpaceEnvelopes = new Dictionary<string, Envelope>();
        }

        /// <summary>
        /// 将CellSpace添加到空间索引
        /// </summary>
        public void Insert(CellSpace cellSpace)
        {
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
        }

        /// <summary>
        /// 从索引中移除CellSpace（需要通过重建实现）
        /// </summary>
        public void Remove(string cellSpaceId)
        {
            if (cellSpaceEnvelopes.ContainsKey(cellSpaceId))
            {
                cellSpaceEnvelopes.Remove(cellSpaceId);
                // STRtree不支持直接删除，需要重建
            }
        }

        /// <summary>
        /// 清空索引
        /// </summary>
        public void Clear()
        {
            // STRtree可能没有Clear方法，创建新实例
            tree = new STRtree<CellSpace>();
            cellSpaceEnvelopes.Clear();
        }

        /// <summary>
        /// 重建索引
        /// </summary>
        public void RebuildIndex(List<CellSpace> allCellSpaces)
        {
            // 创建新的树实例而不是调用Clear
            tree = new STRtree<CellSpace>();
            cellSpaceEnvelopes.Clear();
            
            foreach (var cellSpace in allCellSpaces)
            {
                Insert(cellSpace);
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
            
            // 明确转换为List (解决IList到List的转换问题)
            var candidatesList = candidates.ToList();
            
            // 精确测试（点在多边形内）
            var result = new List<CellSpace>();
            var searchPoint = new Point(point.x, point.y);
            
            foreach (var cell in candidatesList)
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
            // 显式转换为List
            return tree.Query(envelope).ToList();
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
            
            // 获取可能与路径相交的CellSpace
            var candidatesList = tree.Query(queryEnvelope).ToList();
            
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
        /// 查找与指定几何图形相交的所有CellSpace
        /// </summary>
        public List<CellSpace> QueryIntersect(Geometry geometry)
        {
            var envelope = geometry.EnvelopeInternal;
            var candidatesList = tree.Query(envelope).ToList();
            
            return candidatesList.Where(cell => cell.Space.Intersects(geometry)).ToList();
        }
    }
}