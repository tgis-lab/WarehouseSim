using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace Map {
    public class GraphConnectivityChecker
    {
        private Graph graph;

        public GraphConnectivityChecker(Graph graph)
        {
            this.graph = graph;
        }

        public bool CheckConnectivityUsingFloydWarshall()
        {
            int n = graph.RoutePoints.Count;
            bool[,] reachable = new bool[n, n];
            
            // 创建节点到索引的映射
            Dictionary<string, int> nodeToIndex = new Dictionary<string, int>();
            for (int i = 0; i < n; i++)
            {
                nodeToIndex[graph.RoutePoints[i].ConnectionPoint.Id] = i;
            }
            
            // 初始化可达矩阵
            for (int i = 0; i < n; i++)
            {
                for (int j = 0; j < n; j++)
                {
                    reachable[i, j] = (i == j); // 自己到自己可达
                }
            }
            
            // 填充直接连接
            foreach (var point in graph.RoutePoints)
            {
                int i = nodeToIndex[point.ConnectionPoint.Id];
                foreach (var child in point.Children)
                {
                    var childPoint = graph.GetRoutePointFormConnectionPoint(child);
                    if (childPoint != null)
                    {
                        int j = nodeToIndex[childPoint.ConnectionPoint.Id];
                        reachable[i, j] = true;
                    }
                }
            }
            
            // Floyd-Warshall核心算法
            for (int k = 0; k < n; k++)
            {
                for (int i = 0; i < n; i++)
                {
                    for (int j = 0; j < n; j++)
                    {
                        reachable[i, j] = reachable[i, j] || (reachable[i, k] && reachable[k, j]);
                    }
                }
            }
            
            // 检查是否所有点对都相互可达
            bool allReachable = true;
            for (int i = 0; i < n && allReachable; i++)
            {
                for (int j = 0; j < n && allReachable; j++)
                {
                    if (!reachable[i, j])
                    {
                        Debug.LogWarning($"Node {graph.RoutePoints[i].ConnectionPoint.Id} cannot reach {graph.RoutePoints[j].ConnectionPoint.Id}");
                        allReachable = false;
                    }
                }
            }
            
            return allReachable;
        }
    }
}