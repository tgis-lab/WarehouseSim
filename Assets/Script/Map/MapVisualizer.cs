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

        // Create a container specifically for vertical connection elements
        GameObject verticalElementsContainer = new GameObject("VerticalElements");
        verticalElementsContainer.transform.SetParent(transform);
        
        // Create a container for each floor
        foreach (var floorLevel in floorLevels)
        {
            GameObject floorContainer = new GameObject($"Floor_{floorLevel}");
            floorContainer.transform.SetParent(transform);
            
            // Calculate floor height
            int floorIndex = ParseFloorLevel(floorLevel);
            float yPosition = (floorIndex - 1) * floorSeparation;
            
            // Set container position explicitly
            floorContainer.transform.position = new Vector3(0, yPosition, 0);
            Debug.Log($"Created floor container for {floorLevel} at height {yPosition}");
            
            floorContainers[floorLevel] = floorContainer;
        }
        
        // Process all elevators first because they are special
        foreach (var cellSpace in indoorSpace.CellSpaces)
        {
            if (cellSpace.IsElevator())
            {
                Debug.Log($"Processing elevator: {cellSpace.Id}");
                
                // Get all floors connected by the elevator
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

                // Calculate height of the lowest floor as the elevator's starting point
                float startHeight = (lowestFloor - 1) * floorSeparation;
                // Calculate total elevator height
                float elevatorTotalHeight = ((highestFloor - lowestFloor) + 1) * floorSeparation;

                // Create elevator visualization - place in vertical elements container
                CreateElevatorVisualization(cellSpace, verticalElementsContainer, connectedFloors, startHeight, elevatorTotalHeight);
                
                // Add elevator markers on each connected floor
                foreach (var floor in connectedFloors)
                {
                    if (floorContainers.TryGetValue(floor, out GameObject floorContainer))
                    {
                        CreateElevatorMarker(cellSpace, floorContainer, floor);
                    }
                }
            }
        }
        
        // Then process all non-elevator CellSpaces
        foreach (var cellSpace in indoorSpace.CellSpaces)
        {
            // Skip elevators because they have already been processed
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
        
        // Visualize vertical connections
        VisualizeVerticalConnections(indoorSpace);
    }
    
    // Clear existing visualization
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
        
        // Clear vertical connection visualization
        GameObject oldConnectionsContainer = transform.Find("VerticalConnections")?.gameObject;
        if (oldConnectionsContainer != null)
        {
            Destroy(oldConnectionsContainer);
        }
    }
    
    // Visualize vertical connections between floors
    private void VisualizeVerticalConnections(IndoorSpace indoorSpace)
    {
        // Create a container for vertical connections
        GameObject connectionsContainer = new GameObject("VerticalConnections");
        connectionsContainer.transform.SetParent(transform);
        
        int connectionCount = 0;
        
        // Process all vertical CellBoundaries
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
                        // Create connection line
                        CreateVerticalConnectionLine(sourceCell, targetCell, connectionsContainer);
                        connectionCount++;
                    }
                }
            }
        }
        
        Debug.Log($"Created {connectionCount} vertical connections");
    }
    
    // Create vertical connection line
    private void CreateVerticalConnectionLine(CellSpace source, CellSpace target, GameObject parent)
    {
        string sourceFloor = source.GetLocatedFloor();
        string targetFloor = target.GetLocatedFloor();
        
        // Get center points of source and target
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
        
        // Create line game object
        GameObject lineObj = new GameObject($"Connection_{source.Id}_{target.Id}");
        lineObj.transform.SetParent(parent.transform);
        
        // Add LineRenderer component
        LineRenderer line = lineObj.AddComponent<LineRenderer>();
        line.positionCount = 2;
        line.SetPosition(0, sourcePos);
        line.SetPosition(1, targetPos);
        
        // Set line appearance
        line.startWidth = 0.2f;
        line.endWidth = 0.2f;
        
        // If specified material exists use it, otherwise create default material
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

    // Create visualization for regular spaces
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
        
        // Create ground object
        GameObject polygonObject = Instantiate(planePrefab, Vector3.zero, Quaternion.identity, parent.transform);
        polygonObject.name = $"Ground_{id}";
        polygonObject.layer = LayerMask.NameToLayer("GroundLayer");
        
        // Use local coordinates (relative to parent floor container)
        Vector3 localPosition = new Vector3((float)polygon.Centroid.X, 0, (float)polygon.Centroid.Y);
        polygonObject.transform.localPosition = localPosition;
        
        // Get polygon dimensions
        float width = (float)(polygon.EnvelopeInternal.MaxX - polygon.EnvelopeInternal.MinX);
        float length = (float)(polygon.EnvelopeInternal.MaxY - polygon.EnvelopeInternal.MinY);
        
        // Adjust scale appropriately
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
                // Create a default material
                Material defaultMaterial = new Material(Shader.Find("Standard"));
                defaultMaterial.color = isNavigable ? Color.gray : Color.blue;
                renderer.material = defaultMaterial;
            }
        }
        
        // Add shelves for non-navigable areas
        if (!isNavigable && shelfPrefab != null)
        {
            GameObject shelfObject = Instantiate(shelfPrefab, Vector3.zero, Quaternion.identity, parent.transform);
            shelfObject.name = $"Shelf_{id}";
            shelfObject.layer = LayerMask.NameToLayer("CollisionLayer");
            
            // Use local coordinates (relative to parent floor container)
            shelfObject.transform.localPosition = localPosition;
            shelfObject.transform.localScale = new Vector3(1f, 1f, 1f);
        }
    }

    // Create stair visualization
    private void CreateStairVisualization(CellSpace cellSpace, GameObject parent)
    {
        Polygon polygon = cellSpace.Space as Polygon;
        if (polygon == null)
        {
            Debug.LogError($"Stair CellSpace {cellSpace.Id} has no valid polygon geometry");
            return;
        }
        
        // Create ground plane
        GameObject groundObject = Instantiate(planePrefab, Vector3.zero, Quaternion.identity, parent.transform);
        groundObject.name = $"Stair_Ground_{cellSpace.Id}";
        groundObject.layer = LayerMask.NameToLayer("GroundLayer");
        
        // Use local coordinates (relative to parent floor container)
        Vector3 localPosition = new Vector3((float)polygon.Centroid.X, 0, (float)polygon.Centroid.Y);
        groundObject.transform.localPosition = localPosition;
        
        // Get polygon dimensions
        float width = (float)(polygon.EnvelopeInternal.MaxX - polygon.EnvelopeInternal.MinX);
        float length = (float)(polygon.EnvelopeInternal.MaxY - polygon.EnvelopeInternal.MinY);
        
        // Adjust scale appropriately
        groundObject.transform.localScale = new Vector3(width * 0.1f, 0.1f, length * 0.1f);
        
        // Set material
        MeshRenderer groundRenderer = groundObject.GetComponent<MeshRenderer>();
        if (groundRenderer != null)
        {
            if (navigablePolygonMaterial != null)
            {
                groundRenderer.material = navigablePolygonMaterial;
            }
            else
            {
                // Create default material
                Material defaultMaterial = new Material(Shader.Find("Standard"));
                defaultMaterial.color = Color.gray;
                groundRenderer.material = defaultMaterial;
            }
        }
        
        // Create semi-transparent cube to represent stairs
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
        
        // Use local coordinates, shift cube up by half height
        Vector3 boxLocalPosition = new Vector3(localPosition.x, floorHeight / 2, localPosition.z);
        stairObject.transform.localPosition = boxLocalPosition;
        stairObject.transform.localScale = new Vector3(width, floorHeight, length);
        
        // Set stair material
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
                // Use default material
                Material defaultMaterial = new Material(Shader.Find("Standard"));
                defaultMaterial.color = new Color(0.5f, 0.5f, 1f, 0.5f); // Semi-transparent blue
                
                // Set to transparent
                defaultMaterial.SetFloat("_Mode", 3); // Fully transparent mode
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
        
        // Add label component
        SpaceIdentifier identifier = stairObject.AddComponent<SpaceIdentifier>();
        identifier.cellSpaceId = cellSpace.Id;
        identifier.spaceType = "Stair";
    }

    // Create elevator visualization
    private void CreateElevatorVisualization(CellSpace cellSpace, GameObject parent, List<string> connectedFloors, float startHeight, float totalHeight)
    {
        Debug.Log($"Starting elevator visualization for {cellSpace.Id} at height {startHeight} with total height {totalHeight}");
        
        Polygon polygon = cellSpace.Space as Polygon;
        if (polygon == null)
        {
            Debug.LogError($"Elevator CellSpace {cellSpace.Id} has no valid polygon geometry");
            return;
        }
        
        // Get polygon dimensions
        float width = (float)(polygon.EnvelopeInternal.MaxX - polygon.EnvelopeInternal.MinX);
        float length = (float)(polygon.EnvelopeInternal.MaxY - polygon.EnvelopeInternal.MinY);
        
        // Elevator center coordinates - horizontal position is polygon center, vertical position is start height plus half height
        Vector3 position = new Vector3(
            (float)polygon.Centroid.X, 
            startHeight + totalHeight / 2, // Vertically centered
            (float)polygon.Centroid.Y
        );
        
        Debug.Log($"Elevator dimensions: width={width}, length={length}, position={position}");
        
        // Create elevator container
        GameObject elevatorObject = new GameObject($"Elevator_{cellSpace.Id}");
        elevatorObject.transform.SetParent(parent.transform);
        elevatorObject.transform.position = position;
        
        // Create elevator shaft - a long cube connecting all floors
        GameObject shaftObject = GameObject.CreatePrimitive(PrimitiveType.Cube);
        shaftObject.name = "ElevatorShaft";
        shaftObject.transform.SetParent(elevatorObject.transform);

        BoxCollider shaftCollider = shaftObject.GetComponent<BoxCollider>();
        if (shaftCollider != null)
        {
            shaftCollider.enabled = false;
        }
        
        // Set shaft position - relative to elevator container, already vertically centered
        shaftObject.transform.localPosition = Vector3.zero;
        shaftObject.transform.localScale = new Vector3(width, totalHeight, length);
        
        // Set elevator shaft material
        MeshRenderer shaftRenderer = shaftObject.GetComponent<MeshRenderer>();
        if (shaftRenderer != null)
        {
            if (elevatorMaterial != null)
            {
                shaftRenderer.material = elevatorMaterial;
            }
            else
            {
                // Create default elevator material
                Material defaultMaterial = new Material(Shader.Find("Standard"));
                defaultMaterial.color = new Color(1f, 0.3f, 0.3f, 0.5f); // Semi-transparent red
                
                // Set to transparent
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
        
        // Add identifier component
        SpaceIdentifier identifier = elevatorObject.AddComponent<SpaceIdentifier>();
        identifier.cellSpaceId = cellSpace.Id;
        identifier.spaceType = "Elevator";
        identifier.connectedFloors = new List<string>(connectedFloors);
        
        Debug.Log($"Completed elevator visualization for {cellSpace.Id}");
    }

    // Create elevator floor marker
    private void CreateElevatorMarker(CellSpace elevatorSpace, GameObject floorContainer, string floorLevel)
    {
        Debug.Log($"Creating elevator marker for {elevatorSpace.Id} on floor {floorLevel}");
        
        Polygon polygon = elevatorSpace.Space as Polygon;
        if (polygon == null)
        {
            Debug.LogError($"Elevator CellSpace {elevatorSpace.Id} has no valid polygon geometry");
            return;
        }
        
        // Use local coordinates (relative to parent floor container)
        Vector3 localPosition = new Vector3((float)polygon.Centroid.X, 0, (float)polygon.Centroid.Y);
        
        // Get polygon dimensions
        float width = (float)(polygon.EnvelopeInternal.MaxX - polygon.EnvelopeInternal.MinX);
        float length = (float)(polygon.EnvelopeInternal.MaxY - polygon.EnvelopeInternal.MinY);
        
        // Create elevator marker object
        GameObject markerObject = new GameObject($"ElevatorMarker_{elevatorSpace.Id}_{floorLevel}");
        markerObject.transform.SetParent(floorContainer.transform);
        markerObject.transform.localPosition = localPosition;
        
        // Create ground marker
        GameObject groundObject = Instantiate(planePrefab, Vector3.zero, Quaternion.identity, markerObject.transform);
        groundObject.name = "ElevatorGround";
        groundObject.layer = LayerMask.NameToLayer("GroundLayer");
        groundObject.transform.localPosition = Vector3.zero;
        groundObject.transform.localScale = new Vector3(width * 0.1f, 0.1f, length * 0.1f);
        
        MeshRenderer groundRenderer = groundObject.GetComponent<MeshRenderer>();
        if (groundRenderer != null)
        {
            // Mark elevator area with special material
            Material specialMaterial = new Material(Shader.Find("Standard"));
            specialMaterial.color = new Color(1f, 0.7f, 0.7f); // Pale red
            groundRenderer.material = specialMaterial;
        }
        
        // Add text label - using 3D TextMesh
        GameObject textObject = new GameObject("ElevatorLabel");
        textObject.transform.SetParent(markerObject.transform);
        textObject.transform.localPosition = new Vector3(0, 0.2f, 0);
        textObject.transform.localRotation = Quaternion.Euler(90, 0, 0);
        
        // Try to add TextMesh component (avoiding potential dependency issues with TMPro)
        TextMesh textMesh = textObject.AddComponent<TextMesh>();
        textMesh.text = $"Elevator {elevatorSpace.Id}";
        textMesh.fontSize = 30;
        textMesh.alignment = TextAlignment.Center;
        textMesh.anchor = TextAnchor.MiddleCenter;
        textMesh.color = Color.black;
        textMesh.characterSize = 0.1f;
        
        // Add identifier component
        SpaceIdentifier identifier = markerObject.AddComponent<SpaceIdentifier>();
        identifier.cellSpaceId = elevatorSpace.Id;
        identifier.spaceType = "ElevatorMarker";
        identifier.connectedFloors = new List<string>(elevatorSpace.GetConnectedFloors());
    }
    
    // Parse floor level as number (e.g., "1F" -> 1, "B1" -> -1)
    private int ParseFloorLevel(string floorLevel)
    {
        if (string.IsNullOrEmpty(floorLevel))
            return 0;
                
        if (floorLevel.StartsWith("B") || floorLevel.StartsWith("b"))
        {
            // Basement floors
            if (int.TryParse(floorLevel.Substring(1).Replace("F", "").Replace("f", ""), out int level))
            {
                return -level;
            }
        }
        else
        {
            // Regular floors
            if (int.TryParse(floorLevel.Replace("F", "").Replace("f", ""), out int level))
            {
                return level;
            }
        }
            
        return 0; // Default
    }
}

// Component to identify space type
public class SpaceIdentifier : MonoBehaviour
{
    public string cellSpaceId;
    public string spaceType; // "Elevator", "Stair", "Regular", etc.
    public List<string> connectedFloors = new List<string>(); // Connected floors
}