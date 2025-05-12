using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using NetTopologySuite.Geometries;
using Map;

namespace CentralControl.OrderAssignment
{
    public class MultiFloorOrderGenerator
    {
        private IndoorSpace indoorSpace;
        private System.Random random = new System.Random();
        
        private Dictionary<string, float> defaultFloorDistribution = new Dictionary<string, float>
        {
            { "1F", 0.5f },  // 50%的订单在1楼
            { "2F", 0.3f },  // 30%的订单在2楼
            { "3F", 0.2f }   // 20%的订单在3楼
        };
        
        public MultiFloorOrderGenerator(IndoorSpace indoorSpace)
        {
            this.indoorSpace = indoorSpace;
        }
        
        // 生成跨多个楼层的订单
        public List<Order> GenerateOrders(int count)
        {
            return GenerateOrders(count, defaultFloorDistribution);
        }
        
        // 使用指定楼层分布生成订单
        public List<Order> GenerateOrders(int count, Dictionary<string, float> floorDistribution)
        {
            List<Order> generatedOrders = new List<Order>();
            
            var allPickingPoints = indoorSpace.PickingPoints;
            
            var floorLevels = indoorSpace.GetAllFloorLevels();
            foreach (var floor in floorLevels)
            {
                var pickingPointsOnFloor = allPickingPoints.Where(p => p.FloorLevel == floor).ToList();
            }
            
            for (int i = 0; i < count; i++)
            {
                try
                {
                    // 根据分布选择楼层
                    string targetFloor = SelectFloorByDistribution(floorDistribution);
                    
                    // 在选定楼层上找到合适的拣货点
                    PickingPoint pickingPoint = FindRandomPickingPointOnFloor(targetFloor);
                    
                    if (pickingPoint != null)
                    {
                        
                        // 创建订单
                        Order order = CreateOrder(pickingPoint, targetFloor);
                        if (order != null)
                        {
                            generatedOrders.Add(order);
                        }
                        else
                        {
                            Debug.LogError($"创建订单失败，拣货点: {pickingPoint.Id}");
                        }
                    }
                    else
                    {
                        Debug.LogWarning($"在楼层 {targetFloor} 找不到拣货点");
                    }
                }
                catch (Exception ex)
                {
                    Debug.LogError($"生成订单时发生异常: {ex.Message}\n{ex.StackTrace}");
                }
            }
            
            Debug.Log($"成功生成 {generatedOrders.Count} 个订单");
            return generatedOrders;
        }
        
        // 在指定楼层生成订单
        public List<Order> GenerateOrdersOnFloor(int count, string floorLevel)
        {
            List<Order> generatedOrders = new List<Order>();
            
            for (int i = 0; i < count; i++)
            {
                PickingPoint pickingPoint = FindRandomPickingPointOnFloor(floorLevel);
                if (pickingPoint != null)
                {
                    Order order = CreateOrder(pickingPoint, floorLevel);
                    if (order != null)
                    {
                        generatedOrders.Add(order);
                    }
                }
            }
            
            return generatedOrders;
        }
        
        // 根据概率分布选择楼层
        private string SelectFloorByDistribution(Dictionary<string, float> distribution)
        {
            // 计算总概率
            float totalProbability = distribution.Values.Sum();
            float randomValue = (float)random.NextDouble() * totalProbability;
            
            float cumulativeProbability = 0;
            foreach (var pair in distribution)
            {
                cumulativeProbability += pair.Value;
                if (randomValue <= cumulativeProbability)
                    return pair.Key;
            }
            
            // 默认返回第一个楼层
            return distribution.Keys.FirstOrDefault() ?? "1F";
        }
        
        // 在指定楼层找随机拣货点
        private PickingPoint FindRandomPickingPointOnFloor(string floorLevel)
        {
            // 获取该楼层的所有拣货点
            var pickingPoints = indoorSpace.GetPickingPointsOnFloor(floorLevel);
            
            if (pickingPoints == null || pickingPoints.Count == 0)
            {
                Debug.LogWarning($"No picking points found on floor {floorLevel}");
                return null;
            }
            
            // 随机选择一个拣货点
            int index = random.Next(pickingPoints.Count);
            return pickingPoints[index];
        }
        
        // 创建订单
        private Order CreateOrder(PickingPoint pickingPoint, string floorLevel)
        {
            string id = $"Order_{Guid.NewGuid().ToString().Substring(0, 8)}";
            
            // 获取拣货点位置
            CellSpace cell = indoorSpace.GetCellSpaceFromId(pickingPoint.Id);
            Point point = cell?.Node as Point;
            
            if (point == null)
            {
                Debug.LogError($"Could not get position for picking point {pickingPoint.Id}");
                return null;
            }
            
            // 计算Y坐标
            float y = GetFloorHeight(floorLevel);
            
            // 创建目标位置
            Vector3 destination = new Vector3(
                (float)point.X,
                y,
                (float)point.Y
            );
            
            float executionTime = UnityEngine.Random.Range(30f, 90f);
            
            return new Order(id, destination, pickingPoint, executionTime, floorLevel);
        }
        
        private float GetFloorHeight(string floorLevel)
        {
            float baseHeight = 0.05f;
            float floorHeight = 3.0f;
            
            int level = ParseFloorLevel(floorLevel);
            if (level > 0)
            {
                return ((level - 1) * floorHeight) + baseHeight;
            }
            return (level * floorHeight) + baseHeight;
        }
        
        private int ParseFloorLevel(string floorLevel)
        {
            if (string.IsNullOrEmpty(floorLevel)) 
                return 1;
            
            if (floorLevel.StartsWith("F") || floorLevel.EndsWith("F"))
            {
                string numPart = floorLevel.Replace("F", "");
                if (int.TryParse(numPart, out int level))
                {
                    return level;
                }
            }
            
            if (floorLevel.StartsWith("B") || floorLevel.StartsWith("b"))
            {
                string numPart = floorLevel.Substring(1).Replace("F", "").Replace("f", "");
                if (int.TryParse(numPart, out int level))
                {
                    return -level;
                }
            }

            if (int.TryParse(floorLevel, out int simpleLevel))
            {
                return simpleLevel;
            }
            
            return 1;
        }
    }
}