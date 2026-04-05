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
            { "1F", 0.5f }, 
            { "2F", 0.3f }, 
            { "3F", 0.2f } 
        };
        
        public MultiFloorOrderGenerator(IndoorSpace indoorSpace)
        {
            this.indoorSpace = indoorSpace;
        }
        
        public List<Order> GenerateOrders(int count)
        {
            return GenerateOrders(count, defaultFloorDistribution);
        }
        
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
                    string targetFloor = SelectFloorByDistribution(floorDistribution);
                    
                    PickingPoint pickingPoint = FindRandomPickingPointOnFloor(targetFloor);
                    
                    if (pickingPoint != null)
                    {
                        
                        Order order = CreateOrder(pickingPoint, targetFloor);
                        if (order != null)
                        {
                            generatedOrders.Add(order);
                        }
                        else
                        {
                            Debug.LogError($"Order generation failed at Picking Point: {pickingPoint.Id}");
                        }
                    }
                    else
                    {
                        Debug.LogWarning($"Cannot find the Picking Point at {targetFloor} Floor");
                    }
                }
                catch (Exception ex)
                {
                    Debug.LogError($"Error when generate order: {ex.Message}\n{ex.StackTrace}");
                }
            }
            
            return generatedOrders;
        }
        
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
        
        private string SelectFloorByDistribution(Dictionary<string, float> distribution)
        {
            // Calculate total probability
            float totalProbability = distribution.Values.Sum();
            float randomValue = (float)random.NextDouble() * totalProbability;
            
            float cumulativeProbability = 0;
            foreach (var pair in distribution)
            {
                cumulativeProbability += pair.Value;
                if (randomValue <= cumulativeProbability)
                    return pair.Key;
            }
            
            return distribution.Keys.FirstOrDefault() ?? "1F";
        }
        
        private PickingPoint FindRandomPickingPointOnFloor(string floorLevel)
        {
            var pickingPoints = indoorSpace.GetPickingPointsOnFloor(floorLevel);
            
            if (pickingPoints == null || pickingPoints.Count == 0)
            {
                Debug.LogWarning($"No picking points found on floor {floorLevel}");
                return null;
            }
            
            int index = random.Next(pickingPoints.Count);
            return pickingPoints[index];
        }
        
        private Order CreateOrder(PickingPoint pickingPoint, string floorLevel)
        {
            string id = $"Order_{Guid.NewGuid().ToString().Substring(0, 8)}";
            
            CellSpace cell = indoorSpace.GetCellSpaceFromId(pickingPoint.Id);
            Point point = cell?.Node as Point;
            
            if (point == null)
            {
                Debug.LogError($"Could not get position for picking point {pickingPoint.Id}");
                return null;
            }
            
            float y = GetFloorHeight(floorLevel);
            
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
            return Map.FloorLevelUtils.GetFloorHeight(floorLevel);
        }
        
        private int ParseFloorLevel(string floorLevel)
        {
            return Map.FloorLevelUtils.Parse(floorLevel, defaultValue: 1);
        }
    }
}