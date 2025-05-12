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

            // Debug.Log($"找到 {intersectionLayers.Count} 个交叉区域图层");

            foreach (var layer in intersectionLayers)
            {
                // Debug.Log($"处理图层 {layer.Id} 中的 {layer.CellSpaces.Count} 个单元格");
                foreach (var cellId in layer.CellSpaces)
                {
                    if (processedCells.Contains(cellId))
                        continue;

                    CreateIntersectionArea(cellId, processedCells);
                }
            }

            _isInitialized = true;
            // Debug.Log($"成功初始化 {_intersectionAreas.Count} 个交叉区域");
        }

        private void CreateIntersectionArea(string startCellId, HashSet<string> processedCells)
        {
            var intersectionArea = new IntersectionArea($"intersection_{_intersectionAreas.Count + 1}");
            var queue = new Queue<string>();
            
            queue.Enqueue(startCellId);
            processedCells.Add(startCellId);

            // 记录该交叉区域的楼层
            string floorLevel = "1F"; // 默认值
            var startCell = indoorSpace.GetCellSpaceFromId(startCellId);
            if (startCell != null)
            {
                floorLevel = startCell.GetLocatedFloor();
            }
            intersectionArea.FloorLevel = floorLevel;

            // Debug.Log($"创建交叉区域，起始单元格: {startCellId}，楼层: {floorLevel}");

            while (queue.Count > 0)
            {
                var currentCellId = queue.Dequeue();
                var currentCell = indoorSpace.GetCellSpaceFromId(currentCellId);
                
                if (currentCell != null)
                {
                    intersectionArea.AddIntersectionCell(currentCell);
                }
                else
                {
                    Debug.LogWarning($"无法找到单元格 {currentCellId}");
                    continue;
                }

                ProcessConnectedCells(currentCellId, intersectionArea, queue, processedCells);
            }

            if (intersectionArea.IntersectionCells.Count > 0)
            {
                _intersectionAreas.Add(intersectionArea.Id, intersectionArea);
                // Debug.Log($"添加交叉区域 {intersectionArea.Id}，包含 {intersectionArea.IntersectionCells.Count} 个单元格");
            }
        }

        private void ProcessConnectedCells(string currentCellId, IntersectionArea intersectionArea, 
            Queue<string> queue, HashSet<string> processedCells)
        {
            var boundaries = indoorSpace.CellBoundaries.Where(b => 
                b.Source == currentCellId || b.Target == currentCellId);

            foreach (var boundary in boundaries)
            {
                // 跳过垂直边界
                if (boundary.IsVertical())
                {
                    continue;
                }

                var nextCellId = boundary.Source == currentCellId ? boundary.Target : boundary.Source;
                var nextCell = indoorSpace.GetCellSpaceFromId(nextCellId);
                
                if (nextCell == null)
                {
                    Debug.LogWarning($"无法找到相邻单元格 {nextCellId}");
                    continue;
                }
                
                // 确保相邻单元格在同一楼层
                string nextCellFloor = nextCell.GetLocatedFloor();
                if (nextCellFloor != intersectionArea.FloorLevel)
                {
                    continue;
                }

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
                    var connectionPoint = indoorSpace.GetConnectionPointFromCellBoundary(boundary.Id);
                    if (connectionPoint != null)
                    {
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
        }

        public bool IsRobotInIntersection(int robotId)
        {
            if (!_isInitialized)
            {
                Debug.LogWarning("交叉区域未初始化");
                return false;
            }

            // 获取机器人位置
            GameObject robotObj = GameObject.Find($"Robot_{robotId}");
            if (robotObj == null)
            {
                Debug.LogWarning($"找不到机器人 {robotId}");
                return false;
            }
                
            Vector3 robotPosition = robotObj.transform.position;
            
            // 提取楼层信息
            string floorLevel = GetFloorLevelFromPosition(robotPosition);
            
            // 检查机器人是否在任何交叉区域内
            foreach (var intersection in _intersectionAreas.Values)
            {
                // 只检查同一楼层的交叉区域
                if (intersection.FloorLevel == floorLevel && intersection.Contains(robotPosition))
                {
                    return true;
                }
            }
            
            return false;
        }

        public bool IsNearIntersection(Vector3 position, float activationDistance)
        {
            if (!_isInitialized)
            {
                Debug.LogWarning("交叉区域未初始化");
                return false;
            }

            string floorLevel = GetFloorLevelFromPosition(position);

            foreach (var intersection in _intersectionAreas.Values)
            {
                // 只检查同一楼层的交叉区域
                if (intersection.FloorLevel == floorLevel && intersection.IsNear(position, activationDistance))
                {
                    return true;
                }
            }
            
            return false;
        }

        public IntersectionArea GetIntersectionAtPosition(Vector3 position)
        {
            if (!_isInitialized)
            {
                Debug.LogWarning("交叉区域未初始化");
                return null;
            }

            string floorLevel = GetFloorLevelFromPosition(position);

            foreach (var intersection in _intersectionAreas.Values)
            {
                // 只检查同一楼层的交叉区域
                if (intersection.FloorLevel == floorLevel && intersection.Contains(position))
                {
                    return intersection;
                }
            }
            
            return null;
        }

        // 保留接口兼容性，但移除DWA逻辑
        public bool ShouldActivateDWA(Vector3 position, out IntersectionArea activeIntersection)
        {
            activeIntersection = GetNearestIntersection(position);
            return activeIntersection != null;
        }

        private IntersectionArea GetNearestIntersection(Vector3 position)
        {
            if (!_isInitialized)
            {
                Debug.LogError("交叉区域未初始化");
                return null;
            }

            string floorLevel = GetFloorLevelFromPosition(position);
            IntersectionArea nearestIntersection = null;
            float minDistance = float.MaxValue;

            foreach (var intersection in _intersectionAreas.Values)
            {
                // 只检查同一楼层的交叉区域
                if (intersection.FloorLevel != floorLevel)
                    continue;
                    
                if (intersection.IsNear(position, intersection.ActivationDistance))
                {
                    float y = position.y; // 保持原始高度
                    var centroidPos = new Vector3(
                        (float)intersection.IntersectionGeometry.Centroid.X, 
                        y, 
                        (float)intersection.IntersectionGeometry.Centroid.Y);
                    
                    var distance = Vector3.Distance(position, centroidPos);
                    
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
                Debug.LogError("交叉区域未初始化");
                return new Dictionary<string, IntersectionArea>();
            }
            return _intersectionAreas;
        }
        
        // 根据Y坐标获取楼层
        private string GetFloorLevelFromPosition(Vector3 position)
        {
            if (indoorSpace == null)
                return "1F"; // 默认
                
            float floorHeight = 3.0f; // 默认楼层高度
            
            // 尝试从CentralController获取楼层高度设置
            var centralController = FindObjectOfType<CentralControl.CentralController>();
            if (centralController != null)
            {
                floorHeight = centralController.floorHeight;
            }
            
            // 计算楼层数
            int floorNumber = Mathf.RoundToInt(position.y / floorHeight);
            
            if (floorNumber <= 0)
                return "1F"; // 地面或地下默认为1F
                
            return $"{floorNumber + 1}F"; // +1因为通常1F是最底层
        }
    }
}