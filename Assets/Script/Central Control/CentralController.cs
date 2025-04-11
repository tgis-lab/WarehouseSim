using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.AI;
using Map;
using CentralControl.OrderAssignment;
using CentralControl.RobotControl;
using CentralControl.PickerControl;

namespace CentralControl
{
    public class CentralController : MonoBehaviour
    {
        #region Inspector Fields
        [Header("Components")]
        public OrderManager orderManager;
        public RobotManager robotManager;
        public PickerManager pickerManager;
        public MapLoader mapLoader;
        public IntersectionAreaManager intersectionAreaManager;
        
        [Header("Map Settings")]
        public float width, length;
        
        [Header("Order Generation")]
        public float interval = 5f;
        public int orderCount = 10;
        
        [Header("Navigation Settings")]
        public float correctionFactor = 0.1f;
        public float dynamicUpdateInterval = 0.5f;
        public float windowDuration = 300f;
        public float timeStep = 0.1f;
        
        [Header("NavMesh Settings")]
        public bool autoRebuildNavMesh = true;
        public float navMeshRebuildInterval = 60f;
        public NavMeshSurface navMeshSurface;
        #endregion

        #region Private Fields
        // System status
        private bool isSystemInitialized = false;
        private bool isFullyInitialized = false;
        private bool isInitialDispatchDone = false;
        private bool generatingOrders = false;
        
        // Navigation
        private Graph graph;
        private IndoorSpace indoorSpace;
        private GraphConnectivityChecker graphConnectivityChecker;
        private DynamicOccupancyLayer globalOccupancyLayer;
        private bool navMeshInitialized = false;
        private float lastUpdateTime;

        // Conflict resolution
        private ConflictResolver conflictResolver;
        
        // Order management
        private OrderAssignmentSelector orderAssignmentSelector;
        private List<Order> pendingOrders = new List<Order>();
        private List<Order> unassignedOrders = new List<Order>();
        private readonly object dispatchLock = new object();
        
        // Coroutines
        private Coroutine systemCheckCoroutine;
        private Coroutine navMeshUpdateCoroutine;
        private Coroutine orderGenerationCoroutine;

        [SerializeField]
        private CellSpaceConflictDetector cellSpaceConflictDetector;
        private Dictionary<string, List<PathConflictInfo>> detectedConflicts = new Dictionary<string, List<PathConflictInfo>>();
        private bool hasNewConflicts = false;
        #endregion

        #region Events
        public static event EventHandler<RobotStatusEventArgs> OnRobotBecameFree;
        #endregion

        #region Unity Lifecycle
        private void Start()
        {
            Debug.Log("CentralController启动...");
            
            // 注册事件
            RegisterEvents();
            
            // 初始化订单分配器
            orderAssignmentSelector = new OrderAssignmentSelector();
            
            // 启动系统初始化序列
            StartCoroutine(InitializeSystemSequence());
        }

        private void Update()
        {
            // 确保只有在完全初始化后才执行更新
            if (!isFullyInitialized)
                return;
            
            // 调试快捷键
            HandleDebugKeys();

            // 更新冲突检测
            UpdateConflictDetection();
        }

        private void OnDestroy()
        {
            // 清理事件注册
            UnregisterEvents();
            
            // 停止所有协程
            StopAllCoroutines();
        }
        #endregion

        #region Initialization
        private void RegisterEvents()
        {
            if (EventManager.Instance != null)
            {
                EventManager.Instance.OnMapLoaded += OnMapLoaded;
                EventManager.Instance.OnRobotsInitialized += OnRobotsInitialized;
            }
            else
            {
                Debug.LogError("EventManager未找到，事件系统将不可用");
            }
            
            OnRobotBecameFree += HandleRobotBecameFree;
        }

        private void UnregisterEvents()
        {
            if (EventManager.Instance != null)
            {
                EventManager.Instance.OnMapLoaded -= OnMapLoaded;
                EventManager.Instance.OnRobotsInitialized -= OnRobotsInitialized;
            }
            
            OnRobotBecameFree -= HandleRobotBecameFree;
        }

        private IEnumerator InitializeSystemSequence()
        {
            Debug.Log("系统初始化序列启动...");
            
            // 等待地图加载完成
            while (mapLoader == null || !mapLoader.IsMapLoaded)
            {
                Debug.Log("等待地图加载...");
                yield return new WaitForSeconds(1f);
            }
            
            Debug.Log("地图已加载，初始化NavMesh...");

            // 检查地图连接性
            graph = mapLoader.GetGraph();
            indoorSpace = mapLoader.indoorSpace;
            graphConnectivityChecker = new GraphConnectivityChecker(graph);
            if (!graphConnectivityChecker.CheckConnectivityUsingFloydWarshall())
            {
                Debug.LogError("地图连接性验证失败，请检查地图数据");
                yield break;
            }
            
            // 初始化和构建NavMesh
            yield return StartCoroutine(InitializeNavMeshCoroutine());
            
            // 初始化全局占用层
            Debug.Log("初始化全局占用层...");
            InitializeGlobalOccupancyLayer();
            
            // 初始化交叉区域
            Debug.Log("初始化交叉区域...");
            if (intersectionAreaManager != null)
            {
                intersectionAreaManager.InitializeIntersectionAreas(mapLoader.indoorSpace);
            }
            
            // 初始化Pickers
            Debug.Log("初始化拣货员...");
            if (pickerManager != null)
            {
                pickerManager.InitializePickers(mapLoader.indoorSpace, mapLoader.GetMapGrid());
            }
            
            // 初始化Robots
            Debug.Log("初始化机器人...");
            if (robotManager != null)
            {
                robotManager.InitializeRobots(mapLoader.indoorSpace, mapLoader.GetGraph(), 
                                           mapLoader.GetMapGrid(), globalOccupancyLayer, 
                                           intersectionAreaManager);
            }
            
            // 检查系统状态
            yield return StartCoroutine(VerifySystemReadiness());
            
            // 如果系统就绪，启动运行时循环
            if (isSystemInitialized)
            {
                Debug.Log("初始化碰撞检测系统...");
                InitializeConflictDetector();
                conflictResolver = new ConflictResolver(globalOccupancyLayer, cellSpaceConflictDetector, indoorSpace);
                
                // 标记为完全初始化
                isFullyInitialized = true;
                
                StartRuntimeCycles();
            }
            else
            {
                Debug.LogError("系统初始化失败，请检查日志获取详细信息");
            }
        }

        private IEnumerator VerifySystemReadiness()
        {
            Debug.Log("验证系统就绪状态...");
            
            // 验证重要组件
            bool mapReady = mapLoader != null && mapLoader.IsMapLoaded;
            bool navMeshReady = navMeshInitialized;
            bool robotsReady = robotManager != null && robotManager.TotalRobots > 0;
            bool pickersReady = pickerManager != null && pickerManager.TotalPickers > 0;
            
            Debug.Log($"系统状态：地图就绪={mapReady}, NavMesh就绪={navMeshReady}, " +
                    $"机器人就绪={robotsReady}({robotManager?.TotalRobots ?? 0}个), " + 
                    $"拣货员就绪={pickersReady}({pickerManager?.TotalPickers ?? 0}个)");
            
            // 如果机器人未就绪，尝试再次创建
            if (!robotsReady && robotManager != null)
            {
                Debug.LogWarning("未检测到机器人，尝试强制创建...");
                TryForceCreateRobots();
                
                // 等待片刻再次检查
                yield return new WaitForSeconds(2.0f);
                robotsReady = robotManager.TotalRobots > 0;
                Debug.Log($"强制创建后，机器人就绪={robotsReady}({robotManager.TotalRobots}个)");
            }
            
            // 设置系统状态
            isSystemInitialized = mapReady && navMeshReady && robotsReady;
            
            if (isSystemInitialized)
            {
                Debug.Log("系统验证完成：系统已就绪");
            }
            else
            {
                Debug.LogError("系统验证完成：系统未就绪，缺少必要组件");
            }
        }
        
        private void TryForceCreateRobots()
        {
            try
            {
                // 从地图获取一些有效位置
                List<Vector3> positions = new List<Vector3>();
                
                if (mapLoader != null && mapLoader.indoorSpace != null)
                {
                    // 从可导航区域获取位置
                    var navigableCells = mapLoader.indoorSpace.CellSpaces
                        .Where(c => c.IsNavigable() && !c.IsBusinesspoint() && !c.IsPickingPoint())
                        .Take(10)
                        .ToList();
                        
                    foreach (var cell in navigableCells)
                    {
                        positions.Add(new Vector3(
                            (float)cell.Space.Centroid.X,
                            0.05f,
                            (float)cell.Space.Centroid.Y
                        ));
                    }
                }
                
                // 如果没有足够位置，添加一些固定位置
                if (positions.Count < 5)
                {
                    for (int i = 0; i < 5; i++)
                    {
                        positions.Add(new Vector3(10 + i * 5, 0.05f, 10 + i * 5));
                    }
                }
                
                // 要求机器人管理器创建机器人
                Debug.Log($"尝试在{positions.Count}个位置创建机器人");
                if (robotManager != null)
                {
                    robotManager.ForceCreateRobots(positions);
                }
            }
            catch (Exception ex)
            {
                Debug.LogError($"强制创建机器人时出错: {ex.Message}\n{ex.StackTrace}");
            }
        }

        private void StartRuntimeCycles()
        {
            Debug.Log("启动运行时循环...");
            
            // 开始生成订单
            GenerateInitialOrders();
            
            // 启动周期性任务
            systemCheckCoroutine = StartCoroutine(PeriodicSystemCheck());
            orderGenerationCoroutine = StartCoroutine(StartGeneratingOrders());
            
            if (autoRebuildNavMesh)
            {
                navMeshUpdateCoroutine = StartCoroutine(PeriodicNavMeshUpdate());
            }
            
            Debug.Log("所有运行时循环已启动");
        }
        #endregion
        
        #region NavMesh Management
        private IEnumerator InitializeNavMeshCoroutine()
        {
            if (navMeshSurface == null)
            {
                navMeshSurface = gameObject.AddComponent<NavMeshSurface>();
            }
            
            ConfigureNavMeshSurface();
            
            // 构建NavMesh
            navMeshSurface.BuildNavMesh();
            
            // 等待一帧确保NavMesh已构建
            yield return null;
            
            // 验证NavMesh
            ValidateNavMesh();
            
            navMeshInitialized = true;
            Debug.Log("NavMesh初始化完成");
        }
        
        private void ConfigureNavMeshSurface()
        {
            navMeshSurface.collectObjects = CollectObjects.All;
            navMeshSurface.layerMask = LayerMask.GetMask("GroundLayer", "Default");
            navMeshSurface.useGeometry = NavMeshCollectGeometry.PhysicsColliders;
            
            // 配置代理参数
            NavMeshBuildSettings settings = navMeshSurface.GetBuildSettings();
            settings.agentTypeID = 0; // 默认代理类型
            settings.agentRadius = 0.4f;
            settings.agentHeight = 1.8f;
            settings.agentSlope = 45f;
            settings.agentClimb = 0.4f;
            
            // 增强设置
            settings.minRegionArea = 1f; // 较小值以包括小区域
            settings.overrideVoxelSize = true;
            settings.voxelSize = 0.1f; // 更精细的体素尺寸
            
            navMeshSurface.defaultArea = 0; // 可行走区域
        }
        
        private void ValidateNavMesh()
        {
            NavMeshTriangulation triangulation = NavMesh.CalculateTriangulation();
            
            if (triangulation.vertices.Length == 0)
            {
                Debug.LogError("NavMesh构建失败！没有生成任何可导航区域");
                return;
            }
            
            // 计算NavMesh覆盖面积
            float area = CalculateNavMeshArea(triangulation);
            Debug.Log($"NavMesh构建成功: {triangulation.vertices.Length}个顶点, 覆盖面积: {area}平方单位");
            
            // 尝试采样关键点
            if (mapLoader != null && mapLoader.indoorSpace != null)
            {
                SampleKeyLocations(mapLoader.indoorSpace);
            }
        }
        
        private float CalculateNavMeshArea(NavMeshTriangulation triangulation)
        {
            float area = 0;
            for (int i = 0; i < triangulation.indices.Length; i += 3)
            {
                Vector3 p1 = triangulation.vertices[triangulation.indices[i]];
                Vector3 p2 = triangulation.vertices[triangulation.indices[i + 1]];
                Vector3 p3 = triangulation.vertices[triangulation.indices[i + 2]];
                
                area += Vector3.Cross(p2 - p1, p3 - p1).magnitude * 0.5f;
            }
            return area;
        }
        
        private void SampleKeyLocations(IndoorSpace indoorSpace)
        {
            int validPoints = 0;
            int totalPoints = 0;
            
            // 检查拣货点
            foreach (var pickingPoint in indoorSpace.PickingPoints)
            {
                totalPoints++;
                CellSpace cellSpace = indoorSpace.GetCellSpaceFromId(pickingPoint.Id);
                if (cellSpace != null)
                {
                    Vector3 point = new Vector3(
                        (float)cellSpace.Space.Centroid.X,
                        0.1f,
                        (float)cellSpace.Space.Centroid.Y
                    );
                    
                    NavMeshHit hit;
                    if (NavMesh.SamplePosition(point, out hit, 1.0f, NavMesh.AllAreas))
                    {
                        validPoints++;
                    }
                    else
                    {
                        Debug.LogWarning($"拣货点 {pickingPoint.Id} 不在NavMesh上! 位置: {point}");
                    }
                }
            }
            
            Debug.Log($"NavMesh覆盖测试: {validPoints}/{totalPoints} 个拣货点可到达");
        }
        
        private IEnumerator PeriodicNavMeshUpdate()
        {
            while (true)
            {
                yield return new WaitForSeconds(navMeshRebuildInterval);
                
                if (navMeshInitialized)
                {
                    Debug.Log("定期更新NavMesh...");
                    navMeshSurface.UpdateNavMesh(navMeshSurface.navMeshData);
                    ValidateNavMesh();
                }
            }
        }
        #endregion

        #region Global Occupancy Layer
        private void InitializeGlobalOccupancyLayer()
        {
            globalOccupancyLayer = new DynamicOccupancyLayer(
                mapLoader.indoorSpace,
                windowDuration,
                Time.time
            );
        }

        private IEnumerator UpdateGlobalOccupancyLayerRoutine()
        {
            while (true)
            {
                yield return new WaitForSeconds(dynamicUpdateInterval);
                UpdateGlobalOccupancyLayer();
            }
        }

        private void UpdateGlobalOccupancyLayer()
        {
            float currentTime = Time.time;
            float windowStart = currentTime - windowDuration;

            CleanupGlobalOccupancyLayer(windowStart);
            
            MergeRobotOccupancyLayer();
        }

        private void CleanupGlobalOccupancyLayer(float cleanupBeforeTime)
        {
            globalOccupancyLayer.ClearOccupancyInTimeWindow(float.MinValue, cleanupBeforeTime, null);
        }

        private void MergeRobotOccupancyLayer()
        {
            foreach (var robot in robotManager.GetAllRobots())
            {
                if (robot == null) continue;
                
                var personalLayer = robot.GetPersonalOccupancyLayer();
                if (personalLayer != null)
                {
                    globalOccupancyLayer.MergeLayer(personalLayer, Time.time - windowDuration, Time.time + 60);
                }
            }
        }

        public DynamicOccupancyLayer GetGlobalOccupancyLayer()
        {
            return globalOccupancyLayer;
        }
        #endregion

        #region Conflict Detection
        // 初始化碰撞检测器
        private void InitializeConflictDetector()
        {
            if (cellSpaceConflictDetector == null)
            {
                cellSpaceConflictDetector = new CellSpaceConflictDetector();
            }
            
            // 注册事件监听
            if (robotManager != null)
            {
                foreach (var robot in robotManager.GetAllRobots())
                {
                    robot.OnPathPlanned += HandleRobotPathPlanned;
                }
            }
        }
        
        // 检测指定路径的碰撞
        public bool DetectPathConflicts(string robotId, List<ConnectionPoint> path, List<float> times, string planId = null)
        {
            if (path == null || path.Count < 2 || times == null || times.Count < path.Count)
            {
                Debug.LogWarning($"无效的路径数据: 路径点{path?.Count ?? 0}, 时间点{times?.Count ?? 0}");
                return false;
            }
            
            // 合并所有机器人的占用层，获取最新全局状态
            MergeRobotOccupancyLayer();

            try
            {
                // 使用CellSpaceConflictDetector检测冲突
                Dictionary<string, List<TimeOccupancy>> conflicts = cellSpaceConflictDetector.DetectConflicts(
                    globalOccupancyLayer,
                    indoorSpace,
                    path,
                    times,
                    robotId);
                
                bool hasConflict = conflicts.Count > 0;
                
                if (hasConflict)
                {
                    // 记录冲突信息
                    RecordCellSpaceConflict(robotId, planId, path, times, conflicts);

                    // 尝试使用时间偏移解决冲突
                    var (success, newTimes, shift) = conflictResolver.ResolveConflictByTimeShift(
                        robotId, path, times, conflicts);
                    
                    if (success)
                    {
                        Debug.Log($"通过时间偏移成功解决冲突: 机器人 {robotId}, 偏移量 {shift}s");
                        
                        // 使用修改后的时间更新路径预留
                        ReserveCellSpacePath(robotId, planId, path, newTimes);
                        
                        // 通知机器人路径已被修改
                        NotifyRobotPathModified(robotId, planId, path, newTimes, "time_shift", shift);
                        
                        return false; // 冲突已解决
                    }
                    
                    // 时间偏移失败，记录冲突信息
                    RecordCellSpaceConflict(robotId, planId, path, times, conflicts);
                    return true;
                }
                
                // 如果没有冲突，预留路径占用
                ReserveCellSpacePath(robotId, planId, path, times);
                return false;
            }
            catch (Exception ex)
            {
                Debug.LogError($"检测路径冲突时发生错误: {ex.Message}\n{ex.StackTrace}");
                return false;
            }
        }
        
        // 记录路径冲突信息
        private void RecordCellSpaceConflict(string robotId, string planId, 
                                        List<ConnectionPoint> path, 
                                        List<float> times, 
                                        Dictionary<string, List<TimeOccupancy>> conflicts)
        {
            if (!detectedConflicts.ContainsKey(robotId))
            {
                detectedConflicts[robotId] = new List<PathConflictInfo>();
            }
            
            var conflictInfo = new PathConflictInfo
            {
                RobotId = robotId,
                PlanId = planId,
                DetectionTime = Time.time,
                CellSpaceConflicts = new Dictionary<string, List<CellSpaceConflictInfo>>()
            };
            
            // 获取路径对应的CellSpace占用
            var cellOccupancies = cellSpaceConflictDetector.MapConnectionPointPathToCellSpaceOccupancies(
                indoorSpace, path, times);
            
            // 创建CellId到占用时间的映射
            Dictionary<string, CellSpaceOccupancy> cellOccupancyMap = new Dictionary<string, CellSpaceOccupancy>();
            foreach (var occ in cellOccupancies)
            {
                cellOccupancyMap[occ.CellId] = occ;
            }
            
            foreach (var entry in conflicts)
            {
                string cellId = entry.Key;
                var cellConflicts = new List<CellSpaceConflictInfo>();
                
                // 获取当前CellSpace的占用时间
                if (!cellOccupancyMap.TryGetValue(cellId, out var myCellOccupancy))
                {
                    Debug.LogWarning($"无法找到CellSpace {cellId}的占用信息");
                    continue;
                }
                
                float myEntryTime = myCellOccupancy.EntryTime;
                float myExitTime = myCellOccupancy.ExitTime;
                bool isStartCell = myCellOccupancy.IsStartCell;
                bool isEndCell = myCellOccupancy.IsEndCell;
                
                foreach (var occupancy in entry.Value)
                {
                    var conflict = new CellSpaceConflictInfo
                    {
                        CellId = cellId,
                        ConflictRobotId = occupancy.Occupier.RobotId,
                        ConflictPlanId = occupancy.Occupier.PlanId,
                        // ConflictType = DetermineConflictType(myEntryTime, myExitTime, occupancy.StartTime, occupancy.EndTime),
                        MyEntryTime = myEntryTime,
                        MyExitTime = myExitTime,
                        TheirEntryTime = occupancy.StartTime,
                        TheirExitTime = occupancy.EndTime,
                        OverlapDuration = CalculateOverlapDuration(myEntryTime, myExitTime, occupancy.StartTime, occupancy.EndTime),
                        IsStartCell = isStartCell,
                        IsEndCell = isEndCell,
                        TheirIsStartCell = occupancy.IsStartCell,
                        TheirIsEndCell = occupancy.IsEndCell
                    };
                    
                    cellConflicts.Add(conflict);
                }
                
                conflictInfo.CellSpaceConflicts[cellId] = cellConflicts;
            }
            
            detectedConflicts[robotId].Add(conflictInfo);
            hasNewConflicts = true;
            
            // 日志输出冲突信息
            LogCellSpaceConflictInfo(conflictInfo);
        }
        
        // 计算重叠持续时间
        private float CalculateOverlapDuration(float t1Start, float t1End, float t2Start, float t2End)
        {
            float overlapStart = Mathf.Max(t1Start, t2Start);
            float overlapEnd = Mathf.Min(t1End, t2End);
            return Mathf.Max(0, overlapEnd - overlapStart);
        }
        
        // // 确定冲突类型
        // private ConflictType DetermineConflictType(float myEntryTime, float myExitTime, float theirEntryTime, float theirExitTime)
        // {
        //     // 简单分类：基于进入/离开时间判断
        //     // 更复杂的分类可以考虑机器人移动方向
            
        //     // 同时进入 (±0.1秒容差)
        //     if (Mathf.Abs(myEntryTime - theirEntryTime) < 0.1f)
        //     {
        //         return ConflictType.HeadOn;
        //     }
            
        //     // 我先进入，对方后进入
        //     if (myEntryTime < theirEntryTime)
        //     {
        //         // 我离开前对方就进入 -> 追尾冲突
        //         if (myExitTime > theirEntryTime)
        //         {
        //             return ConflictType.BeingOvertaken;
        //         }
        //     }
        //     // 对方先进入，我后进入
        //     else
        //     {
        //         // 对方离开前我就进入 -> 追尾冲突
        //         if (theirExitTime > myEntryTime)
        //         {
        //             return ConflictType.Overtaking;
        //         }
        //     }
            
        //     // 其他类型冲突
        //     return ConflictType.General;
        // }
        
        // 日志输出冲突信息
        private void LogCellSpaceConflictInfo(PathConflictInfo conflictInfo)
        {
            Debug.LogWarning($"===== 检测到路径冲突(中央检测) =====");
            Debug.LogWarning($"机器人: {conflictInfo.RobotId}, 计划: {conflictInfo.PlanId}, 时间: {conflictInfo.DetectionTime:F2}");
            
            foreach (var entry in conflictInfo.CellSpaceConflicts)
            {
                string cellId = entry.Key;
                var conflicts = entry.Value;
                
                Debug.LogWarning($"CellSpace {cellId} 存在 {conflicts.Count} 个冲突:");
                
                foreach (var conflict in conflicts)
                {
                    string cellTypeInfo = "";
                    if (conflict.IsStartCell) cellTypeInfo += "[起点]";
                    if (conflict.IsEndCell) cellTypeInfo += "[终点]";
                    
                    string theirCellTypeInfo = "";
                    if (conflict.TheirIsStartCell) theirCellTypeInfo += "[起点]";
                    if (conflict.TheirIsEndCell) theirCellTypeInfo += "[终点]";
                    
                    Debug.LogWarning($"  我方{cellTypeInfo}时间: {conflict.MyEntryTime:F2}-{conflict.MyExitTime:F2}, " +
                                $"对方{theirCellTypeInfo}时间: {conflict.TheirEntryTime:F2}-{conflict.TheirExitTime:F2}, " +
                                $"重叠: {conflict.OverlapDuration:F2}秒");
                }
            }
            
            Debug.LogWarning($"========================");
        }
        
        // 处理机器人路径规划完成事件
        private void HandleRobotPathPlanned(object sender, RobotPathEventArgs e)
        {
            if (e.Path == null || e.Path.Count < 2 || e.Times == null || e.Times.Count < e.Path.Count)
            {
                Debug.LogWarning($"收到无效的路径规划事件: 机器人 {e.RobotId}");
                return;
            }
            
            // 检测路径冲突
            bool hasConflict = DetectPathConflicts(e.RobotId, e.Path, e.Times, e.PlanId);
            
            if (hasConflict)
            {
                Debug.LogWarning($"The path of robot {e.RobotId} has conflicts!");
                // 在此可以触发事件通知机器人重新规划
                // 或者执行其他冲突处理策略
            }
            else
            {
                Debug.Log($"The path of robot {e.RobotId} has been accepted.");
            }
        }
        
        // 预留路径占用空间
        private void ReserveCellSpacePath(string robotId, string planId, List<ConnectionPoint> path, List<float> times)
        {
            if (string.IsNullOrEmpty(planId))
            {
                planId = $"Plan_{robotId}_{DateTime.Now.Ticks}";
            }
            
            // Use the new method to record path occupancy
            globalOccupancyLayer.RecordPathOccupancy(indoorSpace, path, times, robotId, planId);
        }

        // 定期更新并处理冲突
        private void UpdateConflictDetection()
        {
            // 更新占用层时间
            globalOccupancyLayer.UpdateTime(Time.time);
            
            // 清理过期的冲突记录
            CleanupConflicts();
            
            // 如果有新的冲突，可以触发相应事件
            if (hasNewConflicts)
            {
                // 触发冲突处理事件
                // OnConflictsDetected?.Invoke(detectedConflicts);
                hasNewConflicts = false;
            }
        }
        
        /// 清理过期的冲突记录
        private void CleanupConflicts()
        {
            float expiryTime = Time.time - 300f; // 5分钟过期
            
            foreach (var robotId in new List<string>(detectedConflicts.Keys))
            {
                detectedConflicts[robotId].RemoveAll(c => c.DetectionTime < expiryTime);
                
                if (detectedConflicts[robotId].Count == 0)
                {
                    detectedConflicts.Remove(robotId);
                }
            }
        }
        
        /// 获取指定机器人的冲突记录
        public List<PathConflictInfo> GetRobotConflicts(string robotId)
        {
            if (detectedConflicts.TryGetValue(robotId, out var conflicts))
            {
                return new List<PathConflictInfo>(conflicts);
            }
            return new List<PathConflictInfo>();
        }
        
        #endregion

        #region Order Management
        private void GenerateInitialOrders()
        {
            Debug.Log("Generating initial orders...");
            
            int initialOrderCount = Mathf.Max(5, robotManager.TotalRobots * 2);
            GenerateRandomOrders(initialOrderCount);
            
            DispatchOrders();
        }
        
        public void GenerateRandomOrders(int count)
        {
            Debug.Log($"Start generating {count} orders...");
            StartCoroutine(GenerateOrders(count));
        }

        private IEnumerator GenerateOrders(int count)
        {
            if (mapLoader == null || !mapLoader.IsMapLoaded || mapLoader.indoorSpace == null)
            {
                Debug.LogError("Map not loaded, cannot generate orders");
                yield break;
            }
            
            var pickingPoints = mapLoader.indoorSpace.PickingPoints;
            if (pickingPoints == null || pickingPoints.Count == 0)
            {
                Debug.LogError(" No picking points found, cannot generate orders");
                yield break;
            }
            
            Debug.Log($"Find {pickingPoints.Count} picking points");
            
            // Temp list to store new orders
            List<Order> newOrders = new List<Order>();
            
            // Generate random orders
            for (int i = 0; i < count; i++)
            {
                // Randomly pick a picking point
                int randomIndex = UnityEngine.Random.Range(0, pickingPoints.Count);
                PickingPoint pickingPoint = pickingPoints[randomIndex];
                
                if (pickingPoint == null)
                {
                    Debug.LogWarning($"Picking point at index {randomIndex} is null, skipping order {i+1}");
                    continue;
                }
                
                // Get the corresponding CellSpace
                CellSpace cellSpace = mapLoader.indoorSpace.GetCellSpaceFromId(pickingPoint.Id);
                if (cellSpace == null)
                {
                    Debug.LogWarning($"CellSpace for picking point {pickingPoint.Id} is null, skipping order {i+1}");
                    continue;
                }
                
                // Set the destination
                Vector3 destination = new Vector3(
                    (float)cellSpace.Space.Centroid.X,
                    0.05f,
                    (float)cellSpace.Space.Centroid.Y
                );
                
                // Generate a unique order ID
                string orderId = $"Order_{System.DateTime.Now.Ticks}_{i}";
                
                // Generate a random execution time
                // float executionTime = UnityEngine.Random.Range(10f, 30f);
                float executionTime = 5.0f;
                Order order = new Order(orderId, destination, pickingPoint, executionTime);
                
                // Add the order to the list
                newOrders.Add(order);
                
                // Wait for a frame to avoid blocking the main thread
                if (i % 5 == 0) yield return null;
            }
            
            // Add the new orders to the pending list
            lock (dispatchLock)
            {
                pendingOrders.AddRange(newOrders);
            }
            
            Debug.Log($"Generated {newOrders.Count} new orders");

            // Dispatch the new orders
            DispatchOrders();
        }

        private IEnumerator StartGeneratingOrders()
        {
            generatingOrders = true;
            Debug.Log("Order generation started...");
            
            while (generatingOrders)
            {
                // Generate random orders
                int ordersToGenerate = UnityEngine.Random.Range(1, robotManager.TotalRobots);
                Debug.Log($"Generating {ordersToGenerate} new orders...");
                GenerateRandomOrders(ordersToGenerate);
                
                // Wait for the interval
                yield return new WaitForSeconds(interval);
            }
        }

        private void DispatchOrders()
        {
            lock (dispatchLock)
            {
                try
                {
                    Debug.Log("Dispatching orders...");
                    
                    // Combine pending and unassigned orders
                    List<Order> ordersToAssign = new List<Order>(pendingOrders);
                    ordersToAssign.AddRange(unassignedOrders);
                    
                    if (ordersToAssign.Count == 0)
                    {
                        Debug.Log("No orders to dispatch");
                        return;
                    }
                    
                    // Get available robots
                    List<RobotController> robotsToAssign = robotManager.GetAvailableRobots();
                    
                    if (robotsToAssign.Count == 0)
                    {
                        Debug.Log("No available robots to dispatch orders");
                        unassignedOrders = ordersToAssign;
                        pendingOrders.Clear();
                        return;
                    }
                    
                    Debug.Log($"Ready to dispatch {ordersToAssign.Count} orders to {robotsToAssign.Count} robots");
                    
                    // Perform order assignment
                    if (!isInitialDispatchDone)
                    {
                        Debug.Log("Performing initial order distribution...");
                        orderAssignmentSelector.InitialOrderDistribution(ordersToAssign, robotsToAssign);
                        isInitialDispatchDone = true;
                    }
                    else
                    {
                        Debug.Log("Performing normal order distribution...");
                        orderAssignmentSelector.AssignOrders(ordersToAssign, robotsToAssign);
                    }
                    
                    // Update order lists
                    unassignedOrders = ordersToAssign.Where(o => o.AssignedRobotId == null).ToList();
                    pendingOrders.Clear();

                    int assignedCount = ordersToAssign.Count - unassignedOrders.Count;
                    Debug.Log($"Order dispatch completed: {assignedCount} orders assigned, {unassignedOrders.Count} orders unassigned");
                }
                catch (Exception ex)
                {
                    Debug.LogError($"Error dispatching orders: {ex.Message}\n{ex.StackTrace}");
                }
            }
        }

        private void DispatchOrdersToFreedRobot(RobotController freeRobot)
        {
            if (freeRobot == null || !freeRobot.IsAvailable) return;
            
            lock (dispatchLock)
            {
                if (unassignedOrders.Count > 0)
                {
                    Debug.Log($"Dispatching orders to freed robot {freeRobot.Id}");
                    
                    // Get the nearest order
                    Order nearestOrder = unassignedOrders
                        .OrderBy(o => Vector3.Distance(freeRobot.transform.position, o.Destination))
                        .FirstOrDefault();
                    
                    if (nearestOrder != null)
                    {
                        try
                        {
                            freeRobot.ReceiveOrder(nearestOrder);
                            nearestOrder.AssignedRobotId = freeRobot.Id;
                            unassignedOrders.Remove(nearestOrder);
                            
                            Debug.Log($"Order {nearestOrder.Id} dispatched to robot {freeRobot.Id}");
                        }
                        catch (Exception ex)
                        {
                            Debug.LogError($"Error dispatching order to robot {freeRobot.Id}: {ex.Message}");
                        }
                    }
                }
            }
        }

        private void HandleRobotBecameFree(object sender, RobotStatusEventArgs e)
        {
            if (e.Robot != null && e.Robot.IsFree)
            {
                Debug.Log($"Robot {e.Robot.Id} became free, dispatching orders...");
                DispatchOrdersToFreedRobot(e.Robot);
            }
        }
        #endregion

        #region System Check
        private IEnumerator PeriodicSystemCheck()
        {
            while (true)
            {
                yield return new WaitForSeconds(30f);
                PerformSystemCheck();
            }
        }

        private void PerformSystemCheck()
        {
            Debug.Log("Performing system check...");
            
            // Check order status
            Debug.Log($"订单状态: 未分配订单: {unassignedOrders.Count}, 待处理订单: {pendingOrders.Count}");
            
            // 检查机器人状态
            var robots = robotManager.GetAllRobots();
            int freeRobots = robots.Count(r => r.IsFree);
            int stuckRobots = robots.Count(r => r.IsStuck);
            
            Debug.Log($"机器人状态: 总数: {robots.Count}, 空闲: {freeRobots}, 卡住: {stuckRobots}");
            
            // 检查NavMesh状态
            if (navMeshInitialized)
            {
                CheckEntitiesOnNavMesh();
            }
            
            // 检查是否需要生成更多订单
            if (unassignedOrders.Count == 0 && pendingOrders.Count == 0 && freeRobots > 0)
            {
                Debug.Log("系统空闲，生成新订单");
                GenerateRandomOrders(freeRobots);
                DispatchOrders();
            }
            
            // 重置卡住的机器人
            if (stuckRobots > 0)
            {
                Debug.Log($"重置 {stuckRobots} 个卡住的机器人");
                robotManager.ResetStuckRobots();
            }
        }
        
        private void CheckEntitiesOnNavMesh()
        {
            int robotsOnNavMesh = 0;
            int pickersOnNavMesh = 0;
            
            // 检查机器人
            var robots = robotManager.GetAllRobots();
            foreach (var robot in robots)
            {
                NavMeshHit hit;
                if (NavMesh.SamplePosition(robot.transform.position, out hit, 0.5f, NavMesh.AllAreas))
                {
                    robotsOnNavMesh++;
                }
            }
            
            // 检查拣货员
            var pickers = pickerManager.GetAllPickers();
            foreach (var picker in pickers)
            {
                NavMeshHit hit;
                if (NavMesh.SamplePosition(picker.transform.position, out hit, 0.5f, NavMesh.AllAreas))
                {
                    pickersOnNavMesh++;
                }
            }
            
            Debug.Log($"NavMesh实体检查: 机器人 {robotsOnNavMesh}/{robots.Count}, 拣货员 {pickersOnNavMesh}/{pickers.Count} 在NavMesh上");
            
            // 如果有实体不在NavMesh上，尝试将它们重新放置到NavMesh上
            if (robotsOnNavMesh < robots.Count || pickersOnNavMesh < pickers.Count)
            {
                Debug.LogWarning("检测到实体不在NavMesh上，尝试重新定位...");
                RepositionEntitiesOnNavMesh();
            }
        }
        
        private void RepositionEntitiesOnNavMesh()
        {
            // 重新定位机器人
            var robots = robotManager.GetAllRobots();
            foreach (var robot in robots)
            {
                NavMeshHit hit;
                if (!NavMesh.SamplePosition(robot.transform.position, out hit, 0.5f, NavMesh.AllAreas))
                {
                    // 尝试找到附近的NavMesh点
                    if (NavMesh.SamplePosition(robot.transform.position, out hit, 5.0f, NavMesh.AllAreas))
                    {
                        Debug.Log($"重新定位机器人 {robot.Id} 到NavMesh上: {hit.position}");
                        robot.transform.position = hit.position;
                    }
                }
            }
            
            // 重新定位拣货员
            var pickers = pickerManager.GetAllPickers();
            foreach (var picker in pickers)
            {
                NavMeshHit hit;
                if (!NavMesh.SamplePosition(picker.transform.position, out hit, 0.5f, NavMesh.AllAreas))
                {
                    // 尝试找到附近的NavMesh点
                    if (NavMesh.SamplePosition(picker.transform.position, out hit, 5.0f, NavMesh.AllAreas))
                    {
                        Debug.Log($"重新定位拣货员 {picker.Id} 到NavMesh上: {hit.position}");
                        picker.transform.position = hit.position;
                    }
                }
            }
        }
        #endregion

        #region Notification System
        public static void NotifyRobotFree(RobotController robot)
        {
            OnRobotBecameFree?.Invoke(null, new RobotStatusEventArgs(robot));
        }

        private void NotifyRobotPathModified(string robotId, string planId, List<ConnectionPoint> path, 
                                            List<float> newTimes, string modificationType, float modificationValue)
        {
            // 找到对应的机器人
            RobotController robot = robotManager.GetRobotById(robotId);
            if (robot == null)
            {
                Debug.LogWarning($"无法找到机器人 {robotId}，无法通知路径修改");
                return;
            }
            
            // 创建路径修改事件参数
            var modificationArgs = new RobotPathModificationEventArgs(
                robotId, planId, path, newTimes, modificationType, modificationValue);
            
            // 触发事件通知机器人
            EventManager.Instance.TriggerRobotPathModified(modificationArgs);
        }
        #endregion

        #region Debugging
        private void HandleDebugKeys()
        {
            // F5键：系统诊断
            if (Input.GetKeyDown(KeyCode.F5))
            {
                DiagnoseSystemState();
            }
            
            // F6键：强制生成并分配订单
            if (Input.GetKeyDown(KeyCode.F6))
            {
                ForceGenerateAndAssignOrders();
            }
            
            // F7键：重建NavMesh
            if (Input.GetKeyDown(KeyCode.F7))
            {
                RebuildNavMesh();
            }
        }

        public void DiagnoseSystemState()
        {
            Debug.Log("===== 系统诊断开始 =====");
            Debug.Log($"当前时间: {Time.time}");
            
            // 检查初始化状态
            Debug.Log($"系统初始化状态: {(isSystemInitialized ? "已初始化" : "未初始化")}");
            Debug.Log($"NavMesh状态: {(navMeshInitialized ? "已初始化" : "未初始化")}");
            
            // 检查订单系统
            Debug.Log($"订单系统: 未分配订单: {unassignedOrders.Count}, 等待订单: {pendingOrders.Count}");
            Debug.Log($"订单生成状态: {(generatingOrders ? "正在生成" : "未生成")}");
            
            // 检查机器人状态
            var robots = robotManager.GetAllRobots();
            Debug.Log($"机器人总数: {robots.Count}");
            int freeRobots = 0;
            int availableRobots = 0;
            int stuckRobots = 0;
            
            foreach (var robot in robots)
            {
                if (robot.IsFree) freeRobots++;
                if (robot.IsAvailable) availableRobots++;
                if (robot.IsStuck) stuckRobots++;
                
                Debug.Log($"机器人 {robot.Id}: 空闲={robot.IsFree}, 可用={robot.IsAvailable}, " +
                        $"移动中={robot.IsMoving}, 队列大小={robot.GetRobotOrdersQueueCount}");
            }
            
            Debug.Log($"机器人状态汇总: 空闲={freeRobots}, 可用={availableRobots}, 卡住={stuckRobots}");
            
            // 检查拣货员状态
            var pickers = pickerManager.GetAllPickers();
            int idlePickers = pickers.Count(p => p.IsIdle);
            Debug.Log($"拣货员: 总数={pickers.Count}, 空闲={idlePickers}");
            
            // 检查导航状态
            if (navMeshInitialized)
            {
                NavMeshTriangulation triangulation = NavMesh.CalculateTriangulation();
                Debug.Log($"NavMesh: 顶点数={triangulation.vertices.Length}, 三角形数={triangulation.indices.Length/3}");
            }
            
            Debug.Log("===== 系统诊断完成 =====");
        }

        public void ForceGenerateAndAssignOrders()
        {
            Debug.Log("强制生成并分配订单");
            
            // 根据机器人数量确定订单数量
            int robotCount = robotManager.GetAllRobots().Count;
            int orderCount = Math.Max(5, robotCount);
            
            // 清除已有未分配订单
            pendingOrders.Clear();
            
            // 生成新订单并等待完成
            StartCoroutine(ForceGenerateOrdersCoroutine(orderCount));
        }

        private IEnumerator ForceGenerateOrdersCoroutine(int count)
        {
            Debug.Log($"强制生成{count}个订单");
            
            // 生成订单
            yield return StartCoroutine(GenerateOrders(count));
            
            // 确保订单生成后再分配
            if (pendingOrders.Count > 0)
            {
                Debug.Log($"强制分配订单，共有{pendingOrders.Count}个等待订单");
                DispatchOrders();
                
                // 检查分配结果
                yield return new WaitForSeconds(0.5f);
                
                if (pendingOrders.Count == 0 && unassignedOrders.Count == 0)
                {
                    Debug.Log("所有订单已分配完毕");
                }
                else
                {
                    Debug.LogWarning($"还有{pendingOrders.Count + unassignedOrders.Count}个订单未分配，尝试直接分配");
                    
                    // 尝试简单的直接分配
                    var remainingOrders = new List<Order>(pendingOrders);
                    remainingOrders.AddRange(unassignedOrders);
                    var availableRobots = robotManager.GetAvailableRobots();
                    
                    for (int i = 0; i < Math.Min(remainingOrders.Count, availableRobots.Count); i++)
                    {
                        try
                        {
                            availableRobots[i].ReceiveOrder(remainingOrders[i]);
                            remainingOrders[i].AssignedRobotId = availableRobots[i].Id;
                            
                            Debug.Log($"直接分配订单 {remainingOrders[i].Id} 给机器人 {availableRobots[i].Id}");
                        }
                        catch (Exception ex)
                        {
                            Debug.LogError($"直接分配订单失败: {ex.Message}");
                        }
                    }
                    
                    // 更新未分配订单列表
                    unassignedOrders = remainingOrders.Where(o => o.AssignedRobotId == null).ToList();
                    pendingOrders.Clear();
                }
            }
            else
            {
                Debug.LogWarning("没有生成订单，无法进行分配");
            }
        }

        public void RebuildNavMesh()
        {
            Debug.Log("手动重建NavMesh...");
            
            if (navMeshSurface != null)
            {
                navMeshSurface.BuildNavMesh();
                ValidateNavMesh();
                Debug.Log("NavMesh重建完成");
            }
            else
            {
                Debug.LogError("NavMeshSurface为空，无法重建NavMesh");
            }
        }
        #endregion

        #region Event Handlers
        private void OnMapLoaded()
        {
            // 此方法保留以处理地图加载事件
            // 实际初始化逻辑已移至InitializeSystemSequence
            Debug.Log("OnMapLoaded事件触发 - 使用新的初始化流程");
        }

        private void OnRobotsInitialized()
        {
            // 此方法保留以处理机器人初始化事件
            // 实际初始化逻辑已移至InitializeSystemSequence
            Debug.Log("OnRobotsInitialized事件触发 - 使用新的初始化流程");
        }

        private void OnEnable()
        {
            // 订阅EventManager中的事件
            if (EventManager.Instance != null)
            {
                EventManager.Instance.OnRobotPathPlanned += HandleRobotPathPlanned;
            }
        }

        private void OnDisable()
        {
            // 取消订阅事件
            if (EventManager.Instance != null)
            {
                EventManager.Instance.OnRobotPathPlanned -= HandleRobotPathPlanned;
            }
        }
        #endregion
    }

    // // 冲突类型
    // public enum ConflictType
    // {
    //     // 头对头冲突 - 两机器人相向而行
    //     HeadOn,
    //     // 超车冲突 - 我方想超越前车
    //     Overtaking,
    //     // 被超车冲突 - 后车想超越我方
    //     BeingOvertaken,
    //     // 一般冲突 - 其他类型冲突
    //     General
    // }
    
    // 边界冲突信息
    public class CellSpaceConflictInfo
    {
        public string CellId { get; set; }
        public string ConflictRobotId { get; set; }
        public string ConflictPlanId { get; set; }
        // public ConflictType ConflictType { get; set; }
        public float MyEntryTime { get; set; }
        public float MyExitTime { get; set; }
        public float TheirEntryTime { get; set; }
        public float TheirExitTime { get; set; }
        public float OverlapDuration { get; set; }
        public bool IsStartCell { get; set; }
        public bool IsEndCell { get; set; }
        public bool TheirIsStartCell { get; set; }
        public bool TheirIsEndCell { get; set; }
    }
    
    // 路径冲突信息
    public class PathConflictInfo
    {
        public string RobotId { get; set; }
        public string PlanId { get; set; }
        public float DetectionTime { get; set; }
        public Dictionary<string, List<CellSpaceConflictInfo>> CellSpaceConflicts { get; set; } = 
            new Dictionary<string, List<CellSpaceConflictInfo>>();
    }
    
    // 机器人路径事件参数
    public class RobotPathEventArgs : EventArgs
    {
        public string RobotId { get; set; }
        public string PlanId { get; set; }
        public List<ConnectionPoint> Path { get; set; }
        public List<float> Times { get; set; }
        
        public RobotPathEventArgs(string robotId, string planId, List<ConnectionPoint> path, List<float> times)
        {
            RobotId = robotId;
            PlanId = planId;
            Path = path;
            Times = times;
        }
    }
}
