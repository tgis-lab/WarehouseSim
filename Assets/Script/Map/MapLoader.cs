using UnityEngine;
using UnityEngine.UI;
using UnityEngine.AI;
using Unity.AI.Navigation;
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
            Debug.Log("Starting map load...");
            indoorSpace = await LoadJsonAsync();

            if (indoorSpace == null)
            {
                throw new ArgumentNullException(nameof(indoorSpace), "IndoorSpace cannot be null.");
            }
            
            if (indoorSpace.CellSpaces == null)
            {
                throw new ArgumentNullException(nameof(indoorSpace.CellSpaces), "CellSpaces cannot be null.");
            }
            
            if (mapVisualizer != null)
            {
                mapVisualizer.floorHeight = floorHeight;
                mapVisualizer.floorSeparation = floorSeparation;
                
                mapVisualizer.VisualizeMap(indoorSpace);
            }
            else
            {
                Debug.LogError("MapVisualizer not assigned!");
            }
            
            this.graph = GenerateRouteGraph(indoorSpace);
            
            this.mapGrid = SetMapGrid(width, floorHeight, length);

            CreateMapBoundary();

            // Setup and build NavMesh
            SetupNavMesh();
            
            // NavMesh build complete, adding vertical links
            if (IsNavMeshReady)
            {
                CreateVerticalNavMeshLinks();
            }
            else
            {
                Debug.LogError("NavMesh not ready, cannot create vertical links");
            }

            // Check NavMesh availability
            CheckNavMeshAvailability();
            Debug.Log("NavMesh availability check complete");

            IsMapLoaded = true;
            
            if (EventManager.Instance != null)
            {
                EventManager.Instance.TriggerMapLoaded();
            }
            Debug.Log("Map load complete");
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
            
            var logMessages = new List<string>();
            var errorMessages = new List<string>();

            var result = await Task.Run(() =>
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
                            errorMessages.Add($"JSON Error: {args.ErrorContext.Error.Message}, Path: {args.ErrorContext.Path}");
                            args.ErrorContext.Handled = true;
                        }
                    };

                    var indoorSpace = JsonConvert.DeserializeObject<IndoorSpace>(jsonText, settings);
                    if (indoorSpace != null)
                    {
                        indoorSpace.LoadProperties();
                        logMessages.Add($"JSON Deserialized successfully: {indoorSpace.CellSpaces.Count} cell spaces, {indoorSpace.CellBoundaries.Count} cell boundaries");
                    }
                    else
                    {
                        errorMessages.Add("Deserialized IndoorSpace is null");
                    }
                    return indoorSpace;
                }
                catch (Exception ex)
                {
                    errorMessages.Add($"Error deserializing JSON: {ex.Message}\n{ex.StackTrace}");
                    throw;
                }
            });

            // Flush collected log messages on main thread (Unity Debug API is not thread-safe)
            foreach (var msg in errorMessages)
            {
                Debug.LogError(msg);
            }
            foreach (var msg in logMessages)
            {
                Debug.Log(msg);
            }

            return result;
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
            
            // Create RoutePoint for all connection points
            Debug.Log($"Creating RoutePoints for {hypergraph.HyperNodes.Count} connection points");
            foreach(var cpNode in hypergraph.HyperNodes)
            {
                RoutePoint rpNode = new RoutePoint(cpNode);
                graph.AddNode(rpNode);
            }
            
            // Create edges for all RLineGroups
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
                    
                    // Find ConnectionPoint
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
                    // Rebuild NavMesh for each floor
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
                    // Rebuild unified NavMesh
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
                Debug.Log("Starting NavMesh setup...");
                
                // Get all floor levels
                List<string> floorLevels = indoorSpace.GetAllFloorLevels();
                Debug.Log($"Setting up NavMesh for {floorLevels.Count} floors: {string.Join(", ", floorLevels)}");
                
                if (separateNavMeshPerFloor)
                {
                    // Clear existing NavMeshSurfaces if any
                    foreach (var surface in navMeshSurfaces.Values)
                    {
                        if (surface != null)
                        {
                            Destroy(surface.gameObject);
                        }
                    }
                    navMeshSurfaces.Clear();
                    
                    // Create separate NavMeshSurface for each floor
                    foreach (var floorLevel in floorLevels)
                    {
                        // Create floor navigation mesh object (independent of Floor_X container)
                        GameObject navMeshObj = new GameObject($"NavMesh_{floorLevel}");
                        navMeshObj.transform.SetParent(transform);
                        
                        // Get floor height
                        int floorIndex = ParseFloorLevel(floorLevel);
                        float floorY = floorIndex * floorSeparation;
                        navMeshObj.transform.position = new Vector3(0, floorY, 0);
                        
                        // Add NavMeshSurface component
                        NavMeshSurface navMeshSurface = navMeshObj.AddComponent<NavMeshSurface>();
                        
                        // Configure NavMeshSurface, use ALL to ensure more objects are captured
                        navMeshSurface.collectObjects = CollectObjects.All;
                        navMeshSurface.layerMask = LayerMask.GetMask("Default", "GroundLayer");
                        navMeshSurface.useGeometry = NavMeshCollectGeometry.PhysicsColliders;
                        navMeshSurface.defaultArea = 0; // Walkable
                        
                        // Store reference
                        navMeshSurfaces[floorLevel] = navMeshSurface;
                        
                        // Bake NavMesh
                        navMeshSurface.BuildNavMesh();
                        Debug.Log($"NavMesh built for floor {floorLevel}");
                    }
                }
                else
                {
                    // Use single NavMeshSurface
                    GameObject navMeshObj = new GameObject("NavMesh");
                    navMeshObj.transform.SetParent(transform);
                    
                    NavMeshSurface navMeshSurface = navMeshObj.AddComponent<NavMeshSurface>();
                    navMeshSurface.collectObjects = CollectObjects.All;
                    navMeshSurface.layerMask = LayerMask.GetMask("Default", "GroundLayer");
                    navMeshSurface.useGeometry = NavMeshCollectGeometry.PhysicsColliders;
                    
                    // Store reference
                    navMeshSurfaces["unified"] = navMeshSurface;
                    
                    navMeshSurface.BuildNavMesh();
                    Debug.Log("Unified NavMesh built for all floors");
                }
                
                // Set flag indicating NavMesh is ready
                IsNavMeshReady = true;
                
                Debug.Log("NavMesh setup complete");
            }
            catch (Exception ex)
            {
                Debug.LogError($"Error setting up NavMesh: {ex.Message}\n{ex.StackTrace}");
                IsNavMeshReady = false;
            }
        }

        private void CreateVerticalNavMeshLinks()
        {
            Debug.Log("Creating vertical NavMesh links...");
            
            if (indoorSpace == null || !IsNavMeshReady) 
            {
                Debug.LogWarning("Cannot create vertical connections: IndoorSpace is null or NavMesh is not ready");
                return;
            }
            
            // Get all vertical connections
            var elevators = indoorSpace.CellSpaces.Where(cs => cs.IsElevator()).ToList();
            var verticalBoundaries = indoorSpace.CellBoundaries.Where(cb => cb.IsVertical()).ToList();
            
            // Create NavMesh links for elevators
            foreach (var elevator in elevators)
            {
                var connectedFloors = elevator.GetConnectedFloors();
                if (connectedFloors.Count >= 2)
                {
                    CreateNavMeshLinksForVerticalSpace(elevator, connectedFloors, "elevator");
                }
            }
            
            // Create NavMesh links for stairs
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
                            
                            Debug.Log($"Created NavMesh links for stair layer {stairLayer.Id} connecting floors: {string.Join(", ", floors)}");
                        }
                    }
                }
            }
            
            // Create NavMesh links for vertical boundaries (escalators, etc.)
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
                        // Create connection points from boundary
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
                            
                            // Create NavMesh links between floors
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
            
            Debug.Log($"Created NavMesh links for {elevators.Count} elevators, {stairCount} stairs, and {verticalBoundaries.Count} vertical boundaries");
        }

        private void CreateNavMeshLinksForVerticalSpace(CellSpace space, List<string> connectedFloors, string linkType)
        {
            if (connectedFloors.Count < 2) return;
            
            // Get center position
            Point centerPos = (Point)space.Node;
            Vector3 position = new Vector3((float)centerPos.X, 0, (float)centerPos.Y);
            
            // Create links between all connected floors
            for (int i = 0; i < connectedFloors.Count; i++)
            {
                for (int j = i + 1; j < connectedFloors.Count; j++)
                {
                    string floor1 = connectedFloors[i];
                    string floor2 = connectedFloors[j];
                    
                    // Calculate floor heights
                    float height1 = ParseFloorLevel(floor1) * floorHeight + 0.05f;
                    float height2 = ParseFloorLevel(floor2) * floorHeight + 0.05f;
                    
                    // Create positions with appropriate Y coordinates
                    Vector3 pos1 = new Vector3(position.x, height1, position.z);
                    Vector3 pos2 = new Vector3(position.x, height2, position.z);
                    
                    // Create bidirectional links
                    CreateNavMeshLink(pos1, pos2, floor1, floor2, linkType);
                    CreateNavMeshLink(pos2, pos1, floor2, floor1, linkType);
                }
            }
        }

        private void CreateNavMeshLink(Vector3 start, Vector3 end, string startFloor, string endFloor, string linkType)
        {
            // Create new GameObject for link
            GameObject linkObject = new GameObject($"NavMeshLink_{linkType}_{startFloor}_to_{endFloor}");
            linkObject.transform.SetParent(transform);
            
            // Add NavMeshLink component
            NavMeshLink link = linkObject.AddComponent<NavMeshLink>();
            link.startPoint = start;
            link.endPoint = end;
            link.width = 1.0f;
            link.bidirectional = false; // We create separate links for each direction
            link.costModifier = linkType == "elevator" ? 1 : 2; // Elevators are cheaper than stairs/escalators
            link.agentTypeID = NavMesh.GetSettingsByIndex(0).agentTypeID;
        }

        public void CheckNavMeshAvailability()
        {
            Debug.Log("Checking NavMesh availability...");
            
            if (indoorSpace == null)
            {
                Debug.LogError("IndoorSpace is null, cannot check NavMesh");
                return;
            }
            
            List<string> floorLevels = indoorSpace.GetAllFloorLevels();
            Debug.Log($"Found {floorLevels.Count} floor levels: {string.Join(", ", floorLevels)}");
            
            foreach (string floor in floorLevels)
            {
                // Get a CellSpace on this floor
                var cellSpacesOnFloor = indoorSpace.CellSpaces
                    .Where(cs => cs.GetLocatedFloor() == floor && !cs.IsVerticalConnection())
                    .ToList();
                
                if (cellSpacesOnFloor.Count > 0)
                {
                    var testCell = cellSpacesOnFloor[0];
                    Point center = (Point)testCell.Node;
                    
                    // Calculate floor height
                    float floorY = ParseFloorLevel(floor) * floorHeight + 0.05f;
                    Vector3 testPos = new Vector3((float)center.X, floorY, (float)center.Y);
                    
                    // Test NavMesh availability
                    NavMeshHit hit;
                    bool hasNavMesh = NavMesh.SamplePosition(testPos, out hit, 5.0f, NavMesh.AllAreas);
                    
                    string status = hasNavMesh ? "Valid" : "Invalid";
                    Debug.Log($"NavMesh status on floor {floor}: {status}");
                    
                    if (hasNavMesh)
                    {
                        Debug.Log($"   Closest point: {hit.position}, distance: {hit.distance}");
                    }
                }
                else
                {
                    Debug.LogWarning($"No navigable CellSpace found on floor {floor}");
                }
            }
        }
        # endregion

        # region Helper Methods
        private int ParseFloorLevel(string floorLevel)
        {
            return FloorLevelUtils.Parse(floorLevel);
        }
        # endregion
    }
}