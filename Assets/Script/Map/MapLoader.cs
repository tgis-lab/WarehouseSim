using UnityEngine;
using UnityEngine.UI;
using UnityEngine.AI;
using System.Linq;
using System;
using System.IO;
using System.Collections.Generic;
using System.Threading.Tasks;
using Newtonsoft.Json;
using NetTopologySuite.Geometries;
using NetTopologySuite.IO;
using PathPlanning;
using CentralControl;

namespace Map {
    public class MapLoader : MonoBehaviour
    {
        public TextAsset jsonFile;
        public IndoorSpace indoorSpace;
        public MapVisualizer mapVisualizer;
        public MapGrid mapGrid;
        public GameObject mapBoundaryPrefab;
        public bool IsMapLoaded { get; private set; } = false;
        
        [Range(0, 100)]
        public float width;
        [Range(0, 100)]
        public float length;
        
        [Range(1, 10)]
        public float floorHeight = 3.0f;
        
        [Range(3, 20)]
        public float floorSeparation = 3.0f;
        
        public bool separateNavMeshPerFloor = true;

        private Graph graph;
        private Dictionary<string, NavMeshSurface> navMeshSurfaces = new Dictionary<string, NavMeshSurface>();
        public bool IsNavMeshReady { get; private set; } = false;

        private async void Start()
        {
            Debug.Log("开始加载地图...");
            indoorSpace = await LoadJsonAsync();

            if (indoorSpace == null)
            {
                throw new ArgumentNullException(nameof(indoorSpace), "IndoorSpace不能为null.");
            }
            
            if (indoorSpace.CellSpaces == null)
            {
                throw new ArgumentNullException(nameof(indoorSpace.CellSpaces), "CellSpaces不能为null.");
            }
            
            if (mapVisualizer != null)
            {
                mapVisualizer.floorHeight = floorHeight;
                mapVisualizer.floorSeparation = floorSeparation;
                
                mapVisualizer.VisualizeMap(indoorSpace);
            }
            else
            {
                Debug.LogError("MapVisualizer未分配!");
            }
            
            this.graph = GenerateRouteGraph(indoorSpace);
            
            this.mapGrid = SetMapGrid(width, floorHeight, length);

            CreateMapBoundary();

            // 设置并构建NavMesh
            SetupNavMesh();
            
            // 现在NavMesh已构建完成，添加垂直连接
            if (IsNavMeshReady)
            {
                CreateVerticalNavMeshLinks();
            }
            else
            {
                Debug.LogError("NavMesh未准备好，无法创建垂直连接");
            }

            // 检查NavMesh可用性
            CheckNavMeshAvailability();
            Debug.Log("NavMesh可用性检查完成");

            IsMapLoaded = true;
            
            if (EventManager.Instance != null)
            {
                EventManager.Instance.TriggerMapLoaded();
            }
            Debug.Log("地图加载完成");
        }

        # region JSON Loading
        public async Task<IndoorSpace> LoadJsonAsync()
        {
            if (jsonFile == null)
            {
                Debug.LogError("JSON file is not assigned!");
                return null;
            }
            
            string jsonText = jsonFile.text;
            Debug.Log($"JSON file size: {jsonText.Length} characters");
            
            return await Task.Run(() =>
            {
                try 
                {
                    JsonSerializerSettings settings = new JsonSerializerSettings
                    {
                        Converters = new List<JsonConverter> { new GeometryConverter() },
                        MissingMemberHandling = MissingMemberHandling.Ignore,
                        NullValueHandling = NullValueHandling.Ignore,
                        Error = (sender, args) => 
                        {
                            Debug.LogError($"JSON Error: {args.ErrorContext.Error.Message}, Path: {args.ErrorContext.Path}");
                            args.ErrorContext.Handled = true;
                        }
                    };

                    var indoorSpace = JsonConvert.DeserializeObject<IndoorSpace>(jsonText, settings);
                    if (indoorSpace != null)
                    {
                        indoorSpace.LoadProperties();
                        Debug.Log($"JSON Deserialized successfully: {indoorSpace.CellSpaces.Count} cell spaces, {indoorSpace.CellBoundaries.Count} cell boundaries");
                    }
                    else
                    {
                        Debug.LogError("Deserialized IndoorSpace is null");
                    }
                    return indoorSpace;
                }
                catch (Exception ex)
                {
                    Debug.LogError($"Error deserializing JSON: {ex.Message}\n{ex.StackTrace}");
                    throw;
                }
            });
        }
        # endregion

        # region Graph Generation
        public Graph GenerateRouteGraph(IndoorSpace indoorSpace)
        {
            if (indoorSpace == null)
            {
                throw new ArgumentNullException(nameof(indoorSpace), "IndoorSpace must not be null.");
            }

            Graph graph = new Graph(indoorSpace);
            Hypergraph hypergraph = indoorSpace.GetHypergraph();

            if (hypergraph == null || hypergraph.HyperNodes == null || hypergraph.HyperEdges == null)
            {
                throw new InvalidOperationException("Hypergraph or its components must not be null.");
            }
            
            // 为所有连接点创建RoutePoint
            Debug.Log($"Creating RoutePoints for {hypergraph.HyperNodes.Count} connection points");
            foreach(var cpNode in hypergraph.HyperNodes)
            {
                RoutePoint rpNode = new RoutePoint(cpNode);
                graph.AddNode(rpNode);
            }
            
            // 为所有RLineGroup创建边
            int edgeCount = 0;
            Debug.Log($"Processing {hypergraph.HyperEdges.Count} RLineGroups");
            foreach(var rlineGroup in hypergraph.HyperEdges)
            {
                foreach(var innerEdge in rlineGroup.InnerEdges)
                {
                    if (innerEdge.Count < 2)
                    {
                        Debug.LogWarning($"Invalid inner edge in RLineGroup {rlineGroup.Id}: not enough points");
                        continue;
                    }
                    
                    string sourceId = innerEdge[0];
                    string targetId = innerEdge[1];
                    
                    ConnectionPoint sourceCP = null;
                    ConnectionPoint targetCP = null;
                    
                    // 查找ConnectionPoint
                    foreach (var cp in hypergraph.HyperNodes)
                    {
                        if (cp.Id == sourceId)
                            sourceCP = cp;
                        else if (cp.Id == targetId)
                            targetCP = cp;
                        
                        if (sourceCP != null && targetCP != null)
                            break;
                    }
                    
                    if (sourceCP != null && targetCP != null)
                    {
                        RoutePoint sourceRP = graph.GetRoutePointFromConnectionPoint(sourceCP);
                        RoutePoint targetRP = graph.GetRoutePointFromConnectionPoint(targetCP);
                        
                        if (sourceRP != null && targetRP != null)
                        {
                            sourceRP.AddChild(targetCP);
                            targetRP.AddParent(sourceCP);
                            edgeCount++;
                        }
                        else
                        {
                            Debug.LogWarning($"Failed to find RoutePoint for ConnectionPoint {sourceId} or {targetId}");
                        }
                    }
                    else
                    {
                        Debug.LogWarning($"Failed to find ConnectionPoint {sourceId} or {targetId}");
                    }
                }
            }
            
            Debug.Log($"Created {edgeCount} graph edges");
            return graph;
        }
        # endregion

        # region Map Boundary
        private void CreateMapBoundary()
        {
            if (mapBoundaryPrefab == null)
            {
                Debug.LogWarning("Map boundary prefab is not assigned.");
                return;
            }
            
            GameObject boundaryObject = Instantiate(mapBoundaryPrefab, Vector3.zero, Quaternion.identity);
            boundaryObject.name = "MapBoundary";
            
            MapBoundary mapBoundary = boundaryObject.GetComponent<MapBoundary>();
            if (mapBoundary != null)
            {
                mapBoundary.Initialize(indoorSpace);
            }
            else
            {
                Debug.LogError("MapBoundary component not found on the instantiated prefab.");
            }
        }
        # endregion

        # region Map Grid
        public MapGrid SetMapGrid(float width, float height, float length)
        {
            MapGrid mapGrid = new MapGrid(width, height, length, floorHeight);
            mapGrid.InitializeGrid(indoorSpace);
            return mapGrid;
        }
        # endregion

        # region Getters
        public Graph GetGraph()
        {
            return graph;
        }

        public MapGrid GetMapGrid()
        {
            return mapGrid;
        }

        public Dictionary<string, Vector3Int> GetCellSpaceGridPositions()
        {
            if (indoorSpace == null || indoorSpace.CellSpaces == null || mapGrid == null)
            {
                Debug.LogError("IndoorSpace, CellSpaces, or MapGrid is null in MapLoader");
                return new Dictionary<string, Vector3Int>();
            }

            return mapGrid.GetCellSpaceGridPositions(indoorSpace);
        }
        # endregion

        # region NavMesh
        public void BuildNavMesh()
        {
            Debug.Log("Rebuilding NavMesh...");
            
            try
            {
                if (separateNavMeshPerFloor)
                {
                    // 重建各楼层的NavMesh
                    foreach (var kvp in navMeshSurfaces)
                    {
                        string floorLevel = kvp.Key;
                        NavMeshSurface surface = kvp.Value;
                        
                        if (surface != null)
                        {
                            surface.BuildNavMesh();
                            Debug.Log($"Rebuilt NavMesh for floor {floorLevel}");
                        }
                    }
                }
                else
                {
                    // 重建统一的NavMesh
                    if (navMeshSurfaces.Count > 0)
                    {
                        navMeshSurfaces.First().Value.BuildNavMesh();
                        Debug.Log("Rebuilt unified NavMesh for all floors");
                    }
                }
                
                Debug.Log("NavMesh rebuilding complete");
            }
            catch (Exception ex)
            {
                Debug.LogError($"Error rebuilding NavMesh: {ex.Message}\n{ex.StackTrace}");
            }
        }

        private void SetupNavMesh()
        {
            try
            {
                Debug.Log("开始设置NavMesh...");
                
                // 获取所有楼层
                List<string> floorLevels = indoorSpace.GetAllFloorLevels();
                Debug.Log($"为 {floorLevels.Count} 个楼层设置NavMesh: {string.Join(", ", floorLevels)}");
                
                if (separateNavMeshPerFloor)
                {
                    // 清除现有的NavMeshSurfaces（如果有）
                    foreach (var surface in navMeshSurfaces.Values)
                    {
                        if (surface != null)
                        {
                            Destroy(surface.gameObject);
                        }
                    }
                    navMeshSurfaces.Clear();
                    
                    // 为每个楼层创建单独的NavMeshSurface
                    foreach (var floorLevel in floorLevels)
                    {
                        // 创建楼层导航网格对象（不依赖于Floor_X容器）
                        GameObject navMeshObj = new GameObject($"NavMesh_{floorLevel}");
                        navMeshObj.transform.SetParent(transform);
                        
                        // 获取该楼层的高度
                        int floorIndex = ParseFloorLevel(floorLevel);
                        float floorY = floorIndex * floorSeparation;
                        navMeshObj.transform.position = new Vector3(0, floorY, 0);
                        
                        // 添加NavMeshSurface组件
                        NavMeshSurface navMeshSurface = navMeshObj.AddComponent<NavMeshSurface>();
                        
                        // 配置NavMeshSurface，使用ALL以确保捕获更多对象
                        navMeshSurface.collectObjects = CollectObjects.All;
                        navMeshSurface.layerMask = LayerMask.GetMask("Default", "GroundLayer");
                        navMeshSurface.useGeometry = NavMeshCollectGeometry.PhysicsColliders;
                        navMeshSurface.defaultArea = 0; // Walkable
                        
                        // 存储引用
                        navMeshSurfaces[floorLevel] = navMeshSurface;
                        
                        // 烘焙NavMesh
                        navMeshSurface.BuildNavMesh();
                        Debug.Log($"已为楼层 {floorLevel} 构建NavMesh");
                    }
                }
                else
                {
                    // 使用单一NavMeshSurface
                    GameObject navMeshObj = new GameObject("NavMesh");
                    navMeshObj.transform.SetParent(transform);
                    
                    NavMeshSurface navMeshSurface = navMeshObj.AddComponent<NavMeshSurface>();
                    navMeshSurface.collectObjects = CollectObjects.All;
                    navMeshSurface.layerMask = LayerMask.GetMask("Default", "GroundLayer");
                    navMeshSurface.useGeometry = NavMeshCollectGeometry.PhysicsColliders;
                    
                    // 存储引用
                    navMeshSurfaces["unified"] = navMeshSurface;
                    
                    navMeshSurface.BuildNavMesh();
                    Debug.Log("已为所有楼层构建统一的NavMesh");
                }
                
                // 设置标志表明NavMesh已准备好
                IsNavMeshReady = true;
                
                Debug.Log("NavMesh设置完成");
            }
            catch (Exception ex)
            {
                Debug.LogError($"设置NavMesh时出错: {ex.Message}\n{ex.StackTrace}");
                IsNavMeshReady = false;
            }
        }

        private void CreateVerticalNavMeshLinks()
        {
            Debug.Log("创建垂直NavMesh链接...");
            
            if (indoorSpace == null || !IsNavMeshReady) 
            {
                Debug.LogWarning("无法创建垂直连接：IndoorSpace为空或NavMesh未准备好");
                return;
            }
            
            // 获取所有垂直连接
            var elevators = indoorSpace.CellSpaces.Where(cs => cs.IsElevator()).ToList();
            var verticalBoundaries = indoorSpace.CellBoundaries.Where(cb => cb.IsVertical()).ToList();
            
            // 创建电梯的NavMesh链接
            foreach (var elevator in elevators)
            {
                var connectedFloors = elevator.GetConnectedFloors();
                if (connectedFloors.Count >= 2)
                {
                    CreateNavMeshLinksForVerticalSpace(elevator, connectedFloors, "elevator");
                }
            }
            
            // 创建楼梯的NavMesh链接
            var stairLayers = indoorSpace.GetStairLayers();
            int stairCount = 0;
            
            if (stairLayers != null)
            {
                foreach (var stairLayer in stairLayers)
                {
                    List<CellSpace> stairCellSpaces = indoorSpace.GetStairGroupFromLayerId(stairLayer.Id);
                    
                    if (stairCellSpaces.Count >= 2)
                    {
                        List<string> floors = new List<string>();
                        foreach (var stairCellSpace in stairCellSpaces)
                        {
                            string floorLevel = stairCellSpace.GetLocatedFloor();
                            if (!string.IsNullOrEmpty(floorLevel) && !floors.Contains(floorLevel))
                            {
                                floors.Add(floorLevel);
                            }
                        }
                        
                        if (floors.Count >= 2)
                        {
                            CellSpace referenceStair = stairCellSpaces[0];
                            CreateNavMeshLinksForVerticalSpace(referenceStair, floors, "stair");
                            stairCount++;
                            
                            Debug.Log($"为楼梯层 {stairLayer.Id} 创建了连接楼层的NavMesh链接: {string.Join(", ", floors)}");
                        }
                    }
                }
            }
            
            // 创建垂直边界（自动扶梯等）的NavMesh链接
            foreach (var boundary in verticalBoundaries)
            {
                CellSpace sourceCell = indoorSpace.GetCellSpaceFromId(boundary.Source);
                CellSpace targetCell = indoorSpace.GetCellSpaceFromId(boundary.Target);
                
                if (sourceCell != null && targetCell != null)
                {
                    string sourceFloor = sourceCell.GetLocatedFloor();
                    string targetFloor = targetCell.GetLocatedFloor();
                    
                    if (sourceFloor != targetFloor)
                    {
                        // 从边界创建连接点
                        ConnectionPoint connectionPoint = indoorSpace.GetConnectionPointFromCellBoundary(boundary.Id);
                        
                        if (connectionPoint != null && connectionPoint.Point != null)
                        {
                            Point sourcePosGeo = (Point)sourceCell.Node;
                            Point targetPosGeo = (Point)targetCell.Node;
                            Vector3 sourcePos = new Vector3((float)sourcePosGeo.X, 0, (float)sourcePosGeo.Y);
                            Vector3 targetPos = new Vector3((float)targetPosGeo.X, 0, (float)targetPosGeo.Y);
                            
                            float sourceY = ParseFloorLevel(sourceFloor) * floorHeight + 0.05f;
                            float targetY = ParseFloorLevel(targetFloor) * floorHeight + 0.05f;
                            
                            sourcePos.y = sourceY;
                            targetPos.y = targetY;
                            
                            // 创建楼层间的NavMesh链接
                            CreateNavMeshLink(
                                sourcePos,
                                targetPos,
                                sourceFloor,
                                targetFloor,
                                "escalator");
                        }
                    }
                }
            }
            
            Debug.Log($"已为 {elevators.Count} 个电梯, {stairCount} 个楼梯, 和 {verticalBoundaries.Count} 个垂直边界创建NavMesh链接");
        }

        private void CreateNavMeshLinksForVerticalSpace(CellSpace space, List<string> connectedFloors, string linkType)
        {
            if (connectedFloors.Count < 2) return;
            
            // 获取中心位置
            Point centerPos = (Point)space.Node;
            Vector3 position = new Vector3((float)centerPos.X, 0, (float)centerPos.Y);
            
            // 在所有连接的楼层之间创建链接
            for (int i = 0; i < connectedFloors.Count; i++)
            {
                for (int j = i + 1; j < connectedFloors.Count; j++)
                {
                    string floor1 = connectedFloors[i];
                    string floor2 = connectedFloors[j];
                    
                    // 计算楼层高度
                    float height1 = ParseFloorLevel(floor1) * floorHeight + 0.05f;
                    float height2 = ParseFloorLevel(floor2) * floorHeight + 0.05f;
                    
                    // 创建具有适当Y坐标的位置
                    Vector3 pos1 = new Vector3(position.x, height1, position.z);
                    Vector3 pos2 = new Vector3(position.x, height2, position.z);
                    
                    // 创建双向链接
                    CreateNavMeshLink(pos1, pos2, floor1, floor2, linkType);
                    CreateNavMeshLink(pos2, pos1, floor2, floor1, linkType);
                }
            }
        }

        private void CreateNavMeshLink(Vector3 start, Vector3 end, string startFloor, string endFloor, string linkType)
        {
            // 为链接创建新的GameObject
            GameObject linkObject = new GameObject($"NavMeshLink_{linkType}_{startFloor}_to_{endFloor}");
            linkObject.transform.SetParent(transform);
            
            // 添加NavMeshLink组件
            NavMeshLink link = linkObject.AddComponent<NavMeshLink>();
            link.startPoint = start;
            link.endPoint = end;
            link.width = 1.0f;
            link.bidirectional = false; // 我们为每个方向创建单独的链接
            link.costModifier = linkType == "elevator" ? 1 : 2; // 电梯比楼梯/自动扶梯便宜
            link.agentTypeID = NavMesh.GetSettingsByIndex(0).agentTypeID;
        }

        public void CheckNavMeshAvailability()
        {
            Debug.Log("检查NavMesh可用性...");
            
            if (indoorSpace == null)
            {
                Debug.LogError("IndoorSpace为空，无法检查NavMesh");
                return;
            }
            
            List<string> floorLevels = indoorSpace.GetAllFloorLevels();
            Debug.Log($"找到 {floorLevels.Count} 个楼层: {string.Join(", ", floorLevels)}");
            
            foreach (string floor in floorLevels)
            {
                // 获取该楼层上的一个CellSpace
                var cellSpacesOnFloor = indoorSpace.CellSpaces
                    .Where(cs => cs.GetLocatedFloor() == floor && !cs.IsVerticalConnection())
                    .ToList();
                
                if (cellSpacesOnFloor.Count > 0)
                {
                    var testCell = cellSpacesOnFloor[0];
                    Point center = (Point)testCell.Node;
                    
                    // 计算楼层高度
                    float floorY = ParseFloorLevel(floor) * floorHeight + 0.05f;
                    Vector3 testPos = new Vector3((float)center.X, floorY, (float)center.Y);
                    
                    // 测试NavMesh可用性
                    NavMeshHit hit;
                    bool hasNavMesh = NavMesh.SamplePosition(testPos, out hit, 5.0f, NavMesh.AllAreas);
                    
                    string status = hasNavMesh ? "有效" : "无效";
                    Debug.Log($"楼层 {floor} 上的NavMesh状态: {status}");
                    
                    if (hasNavMesh)
                    {
                        Debug.Log($"   最近的点: {hit.position}, 距离: {hit.distance}");
                    }
                }
                else
                {
                    Debug.LogWarning($"楼层 {floor} 上没有找到可导航的CellSpace");
                }
            }
        }
        # endregion

        # region Helper Methods
        private int ParseFloorLevel(string floorLevel)
        {
            if (string.IsNullOrEmpty(floorLevel))
                return 0;
                
            if (floorLevel.StartsWith("B") || floorLevel.StartsWith("b"))
            {
                if (int.TryParse(floorLevel.Substring(1).Replace("F", "").Replace("f", ""), out int level))
                {
                    return -level;
                }
            }
            else
            {
                if (int.TryParse(floorLevel.Replace("F", "").Replace("f", ""), out int level))
                {
                    return level;
                }
            }
            
            return 0;
        }
        # endregion
    }
}