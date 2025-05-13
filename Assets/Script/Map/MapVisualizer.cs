using UnityEngine;
using System;
using System.Collections.Generic;
using NetTopologySuite.Geometries;
using NetTopologySuite.IO;
using Map;

public class MapVisualizer : MonoBehaviour
{
    # region Materials
    public Material navigablePolygonMaterial;
    public Material nonNavigablePolygonMaterial;
    public Material pickingPointPolygonMaterial;
    public Material stairMaterial;
    public Material elevatorMaterial;
    public Material verticalConnectionMaterial;
    # endregion
    
    # region Prefabs
    public GameObject shelfPrefab;
    public GameObject planePrefab;
    # endregion
    
    # region Floor Containers
    private Dictionary<string, GameObject> floorContainers = new Dictionary<string, GameObject>();
    public float floorHeight = 3.0f;
    public float floorSeparation = 3.0f;
    # endregion

    public void VisualizeMap(IndoorSpace indoorSpace)
    {
        ClearVisualization();
        
        List<string> floorLevels = indoorSpace.GetAllFloorLevels();
        
        Debug.Log($"Found {floorLevels.Count} floor levels: {string.Join(", ", floorLevels)}");

        // 创建一个专门用于垂直连接元素的容器
        GameObject verticalElementsContainer = new GameObject("VerticalElements");
        verticalElementsContainer.transform.SetParent(transform);
        
        // 为每个楼层创建一个容器
        foreach (var floorLevel in floorLevels)
        {
            GameObject floorContainer = new GameObject($"Floor_{floorLevel}");
            floorContainer.transform.SetParent(transform);
            
            // 计算楼层高度
            int floorIndex = ParseFloorLevel(floorLevel);
            float yPosition = (floorIndex - 1) * floorSeparation;
            
            // 明确设置容器位置
            floorContainer.transform.position = new Vector3(0, yPosition, 0);
            Debug.Log($"Created floor container for {floorLevel} at height {yPosition}");
            
            floorContainers[floorLevel] = floorContainer;
        }
        
        // 首先处理所有电梯，因为它们是特殊的
        foreach (var cellSpace in indoorSpace.CellSpaces)
        {
            if (cellSpace.IsElevator())
            {
                Debug.Log($"Processing elevator: {cellSpace.Id}");
                
                // 获取电梯连接的所有楼层
                List<string> connectedFloors = cellSpace.GetConnectedFloors();
                
                if (connectedFloors.Count == 0)
                {
                    Debug.LogWarning($"Elevator {cellSpace.Id} doesn't connect to any floors. Skipping visualization.");
                    continue;
                }
                
                int lowestFloor = int.MaxValue;
                int highestFloor = int.MinValue;

                foreach (var floor in connectedFloors)
                {
                    int floorIndex = ParseFloorLevel(floor);
                    lowestFloor = Mathf.Min(lowestFloor, floorIndex);
                    highestFloor = Mathf.Max(highestFloor, floorIndex);
                }

                // 计算最低楼层的高度作为电梯的起点
                float startHeight = (lowestFloor - 1) * floorSeparation;
                // 计算电梯总高度
                float elevatorTotalHeight = ((highestFloor - lowestFloor) + 1) * floorSeparation;

                // 创建电梯可视化 - 放在垂直元素容器中
                CreateElevatorVisualization(cellSpace, verticalElementsContainer, connectedFloors, startHeight, elevatorTotalHeight);
                
                // 在每个连接的楼层上添加电梯标记
                foreach (var floor in connectedFloors)
                {
                    if (floorContainers.TryGetValue(floor, out GameObject floorContainer))
                    {
                        CreateElevatorMarker(cellSpace, floorContainer, floor);
                    }
                }
            }
        }
        
        // 然后处理所有非电梯CellSpace
        foreach (var cellSpace in indoorSpace.CellSpaces)
        {
            // 跳过电梯，因为我们已经处理过它们
            if (cellSpace.IsElevator())
                continue;
                
            string floorLevel = cellSpace.GetLocatedFloor();
            if (string.IsNullOrEmpty(floorLevel) || !floorContainers.ContainsKey(floorLevel))
            {
                Debug.LogWarning($"CellSpace {cellSpace.Id} has no floor level or floor container not found");
                continue;
            }
            
            GameObject floorContainer = floorContainers[floorLevel];
            
            if (cellSpace.IsStair())
            {
                Debug.Log($"Creating stair for {cellSpace.Id} on floor {floorLevel}");
                CreateStairVisualization(cellSpace, floorContainer);
            }
            else
            {
                CreateSpaceVisualization(cellSpace, floorContainer);
            }
        }
        
        // 可视化垂直连接
        VisualizeVerticalConnections(indoorSpace);
    }
    
    // 清除现有可视化
    private void ClearVisualization()
    {
        foreach (var container in floorContainers.Values)
        {
            if (container != null)
            {
                Destroy(container);
            }
        }
        floorContainers.Clear();
        
        // 清除垂直连接可视化
        GameObject oldConnectionsContainer = transform.Find("VerticalConnections")?.gameObject;
        if (oldConnectionsContainer != null)
        {
            Destroy(oldConnectionsContainer);
        }
    }
    
    // 可视化楼层间的垂直连接
    private void VisualizeVerticalConnections(IndoorSpace indoorSpace)
    {
        // 创建一个容器用于垂直连接
        GameObject connectionsContainer = new GameObject("VerticalConnections");
        connectionsContainer.transform.SetParent(transform);
        
        int connectionCount = 0;
        
        // 处理所有垂直CellBoundaries
        foreach (var cellBoundary in indoorSpace.CellBoundaries)
        {
            if (cellBoundary.IsVertical())
            {
                CellSpace sourceCell = indoorSpace.GetCellSpaceFromId(cellBoundary.Source);
                CellSpace targetCell = indoorSpace.GetCellSpaceFromId(cellBoundary.Target);
                
                if (sourceCell != null && targetCell != null)
                {
                    string sourceFloor = sourceCell.GetLocatedFloor();
                    string targetFloor = targetCell.GetLocatedFloor();
                    
                    if (!string.IsNullOrEmpty(sourceFloor) && !string.IsNullOrEmpty(targetFloor) && 
                        floorContainers.ContainsKey(sourceFloor) && floorContainers.ContainsKey(targetFloor))
                    {
                        // 创建连接线
                        CreateVerticalConnectionLine(sourceCell, targetCell, connectionsContainer);
                        connectionCount++;
                    }
                }
            }
        }
        
        Debug.Log($"Created {connectionCount} vertical connections");
    }
    
    // 创建垂直连接线
    private void CreateVerticalConnectionLine(CellSpace source, CellSpace target, GameObject parent)
    {
        string sourceFloor = source.GetLocatedFloor();
        string targetFloor = target.GetLocatedFloor();
        
        // 获取源和目标的中心点
        Vector3 sourcePos = new Vector3(
            (float)source.Space.Centroid.X,
            (ParseFloorLevel(sourceFloor) - 1) * floorSeparation,
            (float)source.Space.Centroid.Y
        );
        
        Vector3 targetPos = new Vector3(
            (float)target.Space.Centroid.X,
            (ParseFloorLevel(targetFloor) - 1) * floorSeparation,
            (float)target.Space.Centroid.Y
        );
        
        // 创建线条游戏对象
        GameObject lineObj = new GameObject($"Connection_{source.Id}_{target.Id}");
        lineObj.transform.SetParent(parent.transform);
        
        // 添加LineRenderer组件
        LineRenderer line = lineObj.AddComponent<LineRenderer>();
        line.positionCount = 2;
        line.SetPosition(0, sourcePos);
        line.SetPosition(1, targetPos);
        
        // 设置线条外观
        line.startWidth = 0.2f;
        line.endWidth = 0.2f;
        
        // 如果有指定材质则使用，否则创建默认材质
        if (verticalConnectionMaterial != null)
        {
            line.material = verticalConnectionMaterial;
        }
        else
        {
            Material defaultMaterial = new Material(Shader.Find("Standard"));
            defaultMaterial.color = Color.yellow;
            line.material = defaultMaterial;
        }
    }

    // 创建普通空间可视化
    public void CreateSpaceVisualization(CellSpace cellSpace, GameObject parent)
    {
        Polygon polygon = cellSpace.Space as Polygon;
        if (polygon == null)
        {
            Debug.LogError($"CellSpace {cellSpace.Id} has no valid polygon geometry");
            return;
        }
        
        string id = cellSpace.Id;
        bool isNavigable = cellSpace.IsNavigable();
        bool isPickingPoint = cellSpace.IsPickingPoint();
        
        // 创建地面对象
        GameObject polygonObject = Instantiate(planePrefab, Vector3.zero, Quaternion.identity, parent.transform);
        polygonObject.name = $"Ground_{id}";
        polygonObject.layer = LayerMask.NameToLayer("GroundLayer");
        
        // 使用局部坐标（相对于父楼层容器）
        Vector3 localPosition = new Vector3((float)polygon.Centroid.X, 0, (float)polygon.Centroid.Y);
        polygonObject.transform.localPosition = localPosition;
        
        // 获取多边形尺寸
        float width = (float)(polygon.EnvelopeInternal.MaxX - polygon.EnvelopeInternal.MinX);
        float length = (float)(polygon.EnvelopeInternal.MaxY - polygon.EnvelopeInternal.MinY);
        
        // 适当调整缩放
        polygonObject.transform.localScale = new Vector3(width * 0.1f, 0.1f, length * 0.1f);

        MeshRenderer renderer = polygonObject.GetComponent<MeshRenderer>();
        if (renderer != null)
        {
            if (isPickingPoint && pickingPointPolygonMaterial != null)
            {
                renderer.material = pickingPointPolygonMaterial;
            }
            else if (isNavigable && navigablePolygonMaterial != null)
            {
                renderer.material = navigablePolygonMaterial;
            }
            else if (!isNavigable && nonNavigablePolygonMaterial != null)
            {
                renderer.material = nonNavigablePolygonMaterial;
            }
            else
            {
                Debug.LogWarning($"Missing material for CellSpace {id}");
                // 创建一个默认材质
                Material defaultMaterial = new Material(Shader.Find("Standard"));
                defaultMaterial.color = isNavigable ? Color.gray : Color.blue;
                renderer.material = defaultMaterial;
            }
        }
        
        // 为不可导航区域添加货架
        if (!isNavigable && shelfPrefab != null)
        {
            GameObject shelfObject = Instantiate(shelfPrefab, Vector3.zero, Quaternion.identity, parent.transform);
            shelfObject.name = $"Shelf_{id}";
            shelfObject.layer = LayerMask.NameToLayer("CollisionLayer");
            
            // 使用局部坐标（相对于父楼层容器）
            shelfObject.transform.localPosition = localPosition;
            shelfObject.transform.localScale = new Vector3(1f, 1f, 1f);
        }
    }

    // 创建楼梯可视化
    private void CreateStairVisualization(CellSpace cellSpace, GameObject parent)
    {
        Polygon polygon = cellSpace.Space as Polygon;
        if (polygon == null)
        {
            Debug.LogError($"Stair CellSpace {cellSpace.Id} has no valid polygon geometry");
            return;
        }
        
        // 创建地面平面
        GameObject groundObject = Instantiate(planePrefab, Vector3.zero, Quaternion.identity, parent.transform);
        groundObject.name = $"Stair_Ground_{cellSpace.Id}";
        groundObject.layer = LayerMask.NameToLayer("GroundLayer");
        
        // 使用局部坐标（相对于父楼层容器）
        Vector3 localPosition = new Vector3((float)polygon.Centroid.X, 0, (float)polygon.Centroid.Y);
        groundObject.transform.localPosition = localPosition;
        
        // 获取多边形尺寸
        float width = (float)(polygon.EnvelopeInternal.MaxX - polygon.EnvelopeInternal.MinX);
        float length = (float)(polygon.EnvelopeInternal.MaxY - polygon.EnvelopeInternal.MinY);
        
        // 适当调整缩放
        groundObject.transform.localScale = new Vector3(width * 0.1f, 0.1f, length * 0.1f);
        
        // 设置材质
        MeshRenderer groundRenderer = groundObject.GetComponent<MeshRenderer>();
        if (groundRenderer != null)
        {
            if (navigablePolygonMaterial != null)
            {
                groundRenderer.material = navigablePolygonMaterial;
            }
            else
            {
                // 创建默认材质
                Material defaultMaterial = new Material(Shader.Find("Standard"));
                defaultMaterial.color = Color.gray;
                groundRenderer.material = defaultMaterial;
            }
        }
        
        // 创建半透明立方体表示楼梯
        GameObject stairObject = GameObject.CreatePrimitive(PrimitiveType.Cube);
        stairObject.transform.SetParent(parent.transform);
        stairObject.transform.localPosition = Vector3.zero;
        stairObject.name = $"Stair_Box_{cellSpace.Id}";
        stairObject.layer = LayerMask.NameToLayer("GroundLayer");

        BoxCollider stairCollider = stairObject.GetComponent<BoxCollider>();
        if (stairCollider != null)
        {
            stairCollider.enabled = false;
        }
        
        // 使用局部坐标，将立方体上移半个高度
        Vector3 boxLocalPosition = new Vector3(localPosition.x, floorHeight / 2, localPosition.z);
        stairObject.transform.localPosition = boxLocalPosition;
        stairObject.transform.localScale = new Vector3(width, floorHeight, length);
        
        // 设置楼梯材质
        MeshRenderer stairRenderer = stairObject.GetComponent<MeshRenderer>();
        if (stairRenderer != null)
        {
            if (stairMaterial != null)
            {
                stairRenderer.material = stairMaterial;
            }
            else
            {
                Debug.LogWarning("stairMaterial is null! Using default material.");
                // 使用默认材质
                Material defaultMaterial = new Material(Shader.Find("Standard"));
                defaultMaterial.color = new Color(0.5f, 0.5f, 1f, 0.5f); // 半透明蓝色
                
                // 设置为透明
                defaultMaterial.SetFloat("_Mode", 3); // 完全透明模式
                defaultMaterial.SetInt("_SrcBlend", (int)UnityEngine.Rendering.BlendMode.SrcAlpha);
                defaultMaterial.SetInt("_DstBlend", (int)UnityEngine.Rendering.BlendMode.OneMinusSrcAlpha);
                defaultMaterial.SetInt("_ZWrite", 0);
                defaultMaterial.DisableKeyword("_ALPHATEST_ON");
                defaultMaterial.EnableKeyword("_ALPHABLEND_ON");
                defaultMaterial.DisableKeyword("_ALPHAPREMULTIPLY_ON");
                defaultMaterial.renderQueue = 3000;
                
                stairRenderer.material = defaultMaterial;
            }
        }
        else
        {
            Debug.LogError($"No MeshRenderer found on stair object for {cellSpace.Id}");
        }
        
        // 添加标签组件
        SpaceIdentifier identifier = stairObject.AddComponent<SpaceIdentifier>();
        identifier.cellSpaceId = cellSpace.Id;
        identifier.spaceType = "Stair";
    }

    // 创建电梯可视化
    private void CreateElevatorVisualization(CellSpace cellSpace, GameObject parent, List<string> connectedFloors, float startHeight, float totalHeight)
    {
        Debug.Log($"Starting elevator visualization for {cellSpace.Id} at height {startHeight} with total height {totalHeight}");
        
        Polygon polygon = cellSpace.Space as Polygon;
        if (polygon == null)
        {
            Debug.LogError($"Elevator CellSpace {cellSpace.Id} has no valid polygon geometry");
            return;
        }
        
        // 获取多边形尺寸
        float width = (float)(polygon.EnvelopeInternal.MaxX - polygon.EnvelopeInternal.MinX);
        float length = (float)(polygon.EnvelopeInternal.MaxY - polygon.EnvelopeInternal.MinY);
        
        // 电梯中心坐标 - 水平位置是多边形中心，垂直位置是起始高度加上高度的一半
        Vector3 position = new Vector3(
            (float)polygon.Centroid.X, 
            startHeight + totalHeight / 2, // 垂直居中
            (float)polygon.Centroid.Y
        );
        
        Debug.Log($"Elevator dimensions: width={width}, length={length}, position={position}");
        
        // 创建电梯容器
        GameObject elevatorObject = new GameObject($"Elevator_{cellSpace.Id}");
        elevatorObject.transform.SetParent(parent.transform);
        elevatorObject.transform.position = position;
        
        // 创建电梯轴 - 一个连接所有楼层的长立方体
        GameObject shaftObject = GameObject.CreatePrimitive(PrimitiveType.Cube);
        shaftObject.name = "ElevatorShaft";
        shaftObject.transform.SetParent(elevatorObject.transform);

        BoxCollider shaftCollider = shaftObject.GetComponent<BoxCollider>();
        if (shaftCollider != null)
        {
            shaftCollider.enabled = false;
        }
        
        // 设置轴位置 - 相对于电梯容器，已经垂直居中了
        shaftObject.transform.localPosition = Vector3.zero;
        shaftObject.transform.localScale = new Vector3(width, totalHeight, length);
        
        // 设置电梯轴材质
        MeshRenderer shaftRenderer = shaftObject.GetComponent<MeshRenderer>();
        if (shaftRenderer != null)
        {
            if (elevatorMaterial != null)
            {
                shaftRenderer.material = elevatorMaterial;
            }
            else
            {
                // 创建默认电梯材质
                Material defaultMaterial = new Material(Shader.Find("Standard"));
                defaultMaterial.color = new Color(1f, 0.3f, 0.3f, 0.5f); // 半透明红色
                
                // 设置为透明
                defaultMaterial.SetFloat("_Mode", 3);
                defaultMaterial.SetInt("_SrcBlend", (int)UnityEngine.Rendering.BlendMode.SrcAlpha);
                defaultMaterial.SetInt("_DstBlend", (int)UnityEngine.Rendering.BlendMode.OneMinusSrcAlpha);
                defaultMaterial.SetInt("_ZWrite", 0);
                defaultMaterial.DisableKeyword("_ALPHATEST_ON");
                defaultMaterial.EnableKeyword("_ALPHABLEND_ON");
                defaultMaterial.DisableKeyword("_ALPHAPREMULTIPLY_ON");
                defaultMaterial.renderQueue = 3000;
                
                shaftRenderer.material = defaultMaterial;
            }
        }
        
        // 添加标识组件
        SpaceIdentifier identifier = elevatorObject.AddComponent<SpaceIdentifier>();
        identifier.cellSpaceId = cellSpace.Id;
        identifier.spaceType = "Elevator";
        identifier.connectedFloors = new List<string>(connectedFloors);
        
        Debug.Log($"Completed elevator visualization for {cellSpace.Id}");
    }

    // 创建电梯楼层标记
    private void CreateElevatorMarker(CellSpace elevatorSpace, GameObject floorContainer, string floorLevel)
    {
        Debug.Log($"Creating elevator marker for {elevatorSpace.Id} on floor {floorLevel}");
        
        Polygon polygon = elevatorSpace.Space as Polygon;
        if (polygon == null)
        {
            Debug.LogError($"Elevator CellSpace {elevatorSpace.Id} has no valid polygon geometry");
            return;
        }
        
        // 使用局部坐标（相对于父楼层容器）
        Vector3 localPosition = new Vector3((float)polygon.Centroid.X, 0, (float)polygon.Centroid.Y);
        
        // 获取多边形尺寸
        float width = (float)(polygon.EnvelopeInternal.MaxX - polygon.EnvelopeInternal.MinX);
        float length = (float)(polygon.EnvelopeInternal.MaxY - polygon.EnvelopeInternal.MinY);
        
        // 创建电梯标记对象
        GameObject markerObject = new GameObject($"ElevatorMarker_{elevatorSpace.Id}_{floorLevel}");
        markerObject.transform.SetParent(floorContainer.transform);
        markerObject.transform.localPosition = localPosition;
        
        // 创建地面标记
        GameObject groundObject = Instantiate(planePrefab, Vector3.zero, Quaternion.identity, markerObject.transform);
        groundObject.name = "ElevatorGround";
        groundObject.layer = LayerMask.NameToLayer("GroundLayer");
        groundObject.transform.localPosition = Vector3.zero;
        groundObject.transform.localScale = new Vector3(width * 0.1f, 0.1f, length * 0.1f);
        
        MeshRenderer groundRenderer = groundObject.GetComponent<MeshRenderer>();
        if (groundRenderer != null)
        {
            // 使用特殊材质标记电梯区域
            Material specialMaterial = new Material(Shader.Find("Standard"));
            specialMaterial.color = new Color(1f, 0.7f, 0.7f); // 淡红色
            groundRenderer.material = specialMaterial;
        }
        
        // 添加文本标签 - 使用3D TextMesh
        GameObject textObject = new GameObject("ElevatorLabel");
        textObject.transform.SetParent(markerObject.transform);
        textObject.transform.localPosition = new Vector3(0, 0.2f, 0);
        textObject.transform.localRotation = Quaternion.Euler(90, 0, 0);
        
        // 尝试添加TextMesh组件（不使用TMPro以避免可能的依赖问题）
        TextMesh textMesh = textObject.AddComponent<TextMesh>();
        textMesh.text = $"Elevator {elevatorSpace.Id}";
        textMesh.fontSize = 30;
        textMesh.alignment = TextAlignment.Center;
        textMesh.anchor = TextAnchor.MiddleCenter;
        textMesh.color = Color.black;
        textMesh.characterSize = 0.1f;
        
        // 添加标识组件
        SpaceIdentifier identifier = markerObject.AddComponent<SpaceIdentifier>();
        identifier.cellSpaceId = elevatorSpace.Id;
        identifier.spaceType = "ElevatorMarker";
        identifier.connectedFloors = new List<string>(elevatorSpace.GetConnectedFloors());
    }
    
    // 解析楼层级别为数字(例如:"1F" -> 1, "B1" -> -1)
    private int ParseFloorLevel(string floorLevel)
    {
        if (string.IsNullOrEmpty(floorLevel))
            return 0;
                
        if (floorLevel.StartsWith("B") || floorLevel.StartsWith("b"))
        {
            // 地下楼层
            if (int.TryParse(floorLevel.Substring(1).Replace("F", "").Replace("f", ""), out int level))
            {
                return -level;
            }
        }
        else
        {
            // 地上楼层
            if (int.TryParse(floorLevel.Replace("F", "").Replace("f", ""), out int level))
            {
                return level;
            }
        }
            
        return 0; // 默认
    }
}

// 用于标识空间类型的组件
public class SpaceIdentifier : MonoBehaviour
{
    public string cellSpaceId;
    public string spaceType; // "Elevator", "Stair", "Regular", etc.
    public List<string> connectedFloors = new List<string>(); // 连接的楼层
}