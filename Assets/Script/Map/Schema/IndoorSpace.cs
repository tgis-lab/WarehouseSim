using System.Collections;
using UnityEngine;
using Newtonsoft.Json;
using NetTopologySuite.Geometries;
using System.Collections.Generic;
using System.Linq;

namespace Map {
    public class IndoorSpace
    {
        private Dictionary<string, object> _properties = new Dictionary<string, object>();
        private List<CellSpace> _cellSpaces = new List<CellSpace>();
        private List<CellBoundary> _cellBoundaries = new List<CellBoundary>();
        private List<Layer> _layers = new List<Layer>();
        private List<Container> _containers = new List<Container>();

        private List<PickingPoint> _pickingPoints = new List<PickingPoint>();
        private List<PackingPoint> _packingPoints = new List<PackingPoint>();
        private List<QueueArea> _queueAreas = new List<QueueArea>();
        private List<CellSpace> _businessPoints = new List<CellSpace>();

        [JsonProperty("properties")]
        public Dictionary<string, object> Properties { get => _properties; set => _properties = value; }
        [JsonProperty("CellSpaces")]
        public List<CellSpace> CellSpaces { get => _cellSpaces; }
        [JsonProperty("CellBoundaries")]
        public List<CellBoundary> CellBoundaries { get => _cellBoundaries; }
        [JsonProperty("Layers")]
        public List<Layer> Layers { get => _layers; }
        [JsonProperty("Containers")]
        public List<Container> Containers { get => _containers; }

        public List<PickingPoint> PickingPoints { get => _pickingPoints; }
        public List<CellSpace> BusinessPoints { get => _businessPoints; }
        public List<PackingPoint> PackingPoints { get => _packingPoints; }
        public List<QueueArea> QueueAreas { get => _queueAreas; }
        
        public List<string> GetAllFloorLevels()
        {
            HashSet<string> floors = new HashSet<string>();
            foreach (var cellSpace in _cellSpaces)
            {
                string floorLevel = cellSpace.GetLocatedFloor();
                if (!string.IsNullOrEmpty(floorLevel))
                {
                    floors.Add(floorLevel);
                }
            }
            return new List<string>(floors).OrderBy(f => ParseFloorLevel(f)).ToList();
        }
        
        // Parses floor level string into an integer (e.g., "1F" -> 1, "B1" -> -1)
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
        
        public List<CellSpace> GetCellSpacesOnFloor(string floorLevel)
        {
            return _cellSpaces.Where(cs => cs.IsLocatedOnFloor(floorLevel)).ToList();
        }

        public List<PickingPoint> GetPickingPointsOnFloor(string floorLevel)
        {
            return _pickingPoints.Where(pp => pp.FloorLevel == floorLevel).ToList();
        }
        
        public List<CellSpace> GetNavigableCellSpacesOnFloor(string floorLevel)
        {
            return _cellSpaces.Where(cs => cs.IsLocatedOnFloor(floorLevel) && cs.IsNavigable()).ToList();
        }
        
        public List<CellSpace> GetVerticalConnectionsOnFloor(string floorLevel)
        {
            return _cellSpaces.Where(cs => cs.IsLocatedOnFloor(floorLevel) && cs.IsVerticalConnection()).ToList();
        }
        
        public List<CellSpace> GetElevatorsOnFloor(string floorLevel)
        {
            return _cellSpaces.Where(cs => cs.IsLocatedOnFloor(floorLevel) && cs.IsElevator()).ToList();
        }
        
        public List<CellSpace> GetStairsOnFloor(string floorLevel)
        {
            return _cellSpaces.Where(cs => cs.IsLocatedOnFloor(floorLevel) && cs.IsStair()).ToList();
        }
        
        public List<CellBoundary> GetVerticalBoundariesBetweenFloors(string floorLevel1, string floorLevel2)
        {
            List<CellBoundary> result = new List<CellBoundary>();
            
            foreach (var cellBoundary in _cellBoundaries)
            {
                if (cellBoundary.IsVertical())
                {
                    CellSpace sourceCell = GetCellSpaceFromId(cellBoundary.Source);
                    CellSpace targetCell = GetCellSpaceFromId(cellBoundary.Target);
                    
                    if (sourceCell != null && targetCell != null)
                    {
                        string sourceFloor = sourceCell.GetLocatedFloor();
                        string targetFloor = targetCell.GetLocatedFloor();
                        
                        if ((sourceFloor == floorLevel1 && targetFloor == floorLevel2) ||
                            (sourceFloor == floorLevel2 && targetFloor == floorLevel1))
                        {
                            result.Add(cellBoundary);
                        }
                    }
                }
            }
            
            return result;
        }
        
        public List<string> GetDirectlyReachableFloors(string fromFloorLevel)
        {
            HashSet<string> reachableFloors = new HashSet<string>();
            
            var verticalConnections = GetVerticalConnectionsOnFloor(fromFloorLevel);
            foreach (var verticalConnection in verticalConnections)
            {
                var connectedFloors = verticalConnection.GetConnectedFloors();
                foreach (var floor in connectedFloors)
                {
                    reachableFloors.Add(floor);
                }
            }
            
            foreach (var cellBoundary in _cellBoundaries)
            {
                if (cellBoundary.IsVertical())
                {
                    CellSpace sourceCell = GetCellSpaceFromId(cellBoundary.Source);
                    CellSpace targetCell = GetCellSpaceFromId(cellBoundary.Target);
                    
                    if (sourceCell != null && targetCell != null)
                    {
                        string sourceFloor = sourceCell.GetLocatedFloor();
                        string targetFloor = targetCell.GetLocatedFloor();
                        
                        if (sourceFloor == fromFloorLevel)
                        {
                            reachableFloors.Add(targetFloor);
                        }
                        else if (targetFloor == fromFloorLevel)
                        {
                            reachableFloors.Add(sourceFloor);
                        }
                    }
                }
            }
            
            return new List<string>(reachableFloors);
        }
        
        public List<Layer> GetLayersOnFloor(string floorLevel)
        {
            HashSet<Layer> layersOnFloor = new HashSet<Layer>();
            
            var cellSpacesOnFloor = GetCellSpacesOnFloor(floorLevel);
            
            foreach (var cellSpace in cellSpacesOnFloor)
            {
                foreach (var layer in _layers)
                {
                    if (layer.CellSpaces.Contains(cellSpace.Id))
                    {
                        layersOnFloor.Add(layer);
                    }
                }
            }
            
            return new List<Layer>(layersOnFloor);
        }

        public string GetElevatorIdFromCellBoundary(CellBoundary cellBoundary)
        {
            foreach (var cellSpace in _cellSpaces)
            {
                if (cellSpace.Id == cellBoundary.Source || cellSpace.Id == cellBoundary.Target)
                {
                    if (cellSpace.IsElevator())
                    {
                        return cellSpace.Id;
                    }
                }
            }
            Debug.Log("Elevator not found");
            return null;
        }

        public List<Layer> GetStairLayers()
        {
            List<Layer> stairLayers = new List<Layer>();
            foreach (var layer in _layers)
            {
                if (layer.Properties.ContainsKey("type") && layer.Properties["type"].Equals("stair"))
                {
                    stairLayers.Add(layer);
                }
            }
            
            if (stairLayers.Count == 0)
            {
                Debug.Log("Stair layers not found");
            }
            
            return stairLayers;
        }

        public List<CellSpace> GetStairGroupFromLayerId(string layerId)
        {
            List<CellSpace> cellSpaces = new List<CellSpace>();
            List<Layer> layers = GetAllLayers();
            foreach (var layer in layers)
            {
                if (layer.Id == layerId)
                {
                    foreach (var cellSpaceId in layer.CellSpaces)
                    {
                        cellSpaces.Add(GetCellSpaceFromId(cellSpaceId));
                    }
                }
            }
            return cellSpaces;
        }

        public string GetStairIdFromCellBoundary(CellBoundary cellBoundary)
        {
            foreach (var cellSpace in _cellSpaces)
            {
                if (cellSpace.Id == cellBoundary.Source || cellSpace.Id == cellBoundary.Target)
                {
                    if (cellSpace.IsStair())
                    {
                        return cellSpace.Id;
                    }
                }
            }
            Debug.Log("Stair not found");
            return null;
        }

        public CellSpace GetCellSpaceFromId(string id)
        {
            foreach (var cellSpace in _cellSpaces)
            {
                if (cellSpace.Id == id)
                {
                    return cellSpace;
                }
            }
            Debug.Log("CellSpace not found");
            return null;
        }

        public CellSpace GetCellSpaceFromConnectionPoint(ConnectionPoint connectionPoint, bool sequence = true)
        {
            // true: source, false: target
            if (sequence)
            {
                foreach (var cellBoundary in _cellBoundaries)
                {
                    if (cellBoundary.Id == connectionPoint.Id)
                    {
                        return GetCellSpaceFromId(cellBoundary.Source);
                    }
                }
            }
            else
            {
                foreach (var cellBoundary in _cellBoundaries)
                {
                    if (cellBoundary.Id == connectionPoint.Id)
                    {
                        return GetCellSpaceFromId(cellBoundary.Target);
                    }
                }
            }
            Debug.Log("CellSpace not found");
            return null;
        }

        public CellSpace GetCellSpaceFromCoordinates(Vector3 position, string floorLevel)
        {
            List<CellSpace> cellSpacesOnFloor = GetCellSpacesOnFloor(floorLevel);
            foreach (var cellSpace in cellSpacesOnFloor)
            {
                Polygon polygon = (Polygon)cellSpace.Space;
                if (polygon.Contains(new Point(position.x, position.z)))
                {
                    return cellSpace;
                }
            }
            Debug.Log("CellSpace not found");
            return null;
        }

        public List<CellSpace> GetCellSpaceFromLayerId(string layerId)
        {
            List<CellSpace> cellSpaces = new List<CellSpace>();
            List<Layer> layers = GetAllLayers();
            foreach (var layer in layers)
            {
                if (layer.Id == layerId)
                {
                    foreach (var cellSpaceId in layer.CellSpaces)
                    {
                        cellSpaces.Add(GetCellSpaceFromId(cellSpaceId));
                    }
                }
            }
            return cellSpaces;
        }

        public List<CellSpace> GetCellSpaceFromLayerType(string type)
        {
            List<CellSpace> cellSpaces = new List<CellSpace>();
            List<Layer> layers = GetLayerFromType(type);
            foreach (var layer in layers)
            {
                foreach (var cellSpaceId in layer.CellSpaces)
                {
                    cellSpaces.Add(GetCellSpaceFromId(cellSpaceId));
                }
            }
            return cellSpaces;
        }

        public List<CellSpace> GetCellSpaceFromLayerTypeAndFloor(string type, string floorLevel)
        {
            List<CellSpace> cellSpaces = new List<CellSpace>();
            List<Layer> layers = GetLayerFromType(type);
            foreach (var layer in layers)
            {
                foreach (var cellSpaceId in layer.CellSpaces)
                {
                    CellSpace cellSpace = GetCellSpaceFromId(cellSpaceId);
                    if (cellSpace.IsLocatedOnFloor(floorLevel))
                    {
                        cellSpaces.Add(cellSpace);
                    }
                }
            }
            return cellSpaces;
        }

        public CellBoundary GetCellBoundaryFromId(string id)
        {
            foreach (var cellBoundary in _cellBoundaries)
            {
                if (cellBoundary.Id == id)
                {
                    Debug.Log(cellBoundary.ToJson());
                    return cellBoundary;
                }
            }
            Debug.Log("CellBoundary not found");
            return null;
        }

        public List<Layer> GetAllLayers()
        {
            return _layers;
        }

        public List<Layer> GetLayerFromType(string type)
        {
            List<Layer> layers = new List<Layer>();
            foreach (var layer in _layers)
            {
                if (layer.Properties["type"].Equals(type))
                {
                    layers.Add(layer);
                }
            }
            return layers;
        }

        public Layer GetLayerFromCellSpaceId(string cellSpaceId)
        {
            foreach (var layer in _layers)
            {
                if (layer.CellSpaces.Contains(cellSpaceId))
                {
                    return layer;
                }
            }

            Debug.Log(cellSpaceId + "Layer not found");
            return null;
        }

        public List<ConnectionPoint> GetAllConnectionPoints()
        {
            List<ConnectionPoint> connectionPoints = new List<ConnectionPoint>();
            foreach (var cellBoundary in _cellBoundaries)
            {
                connectionPoints.Add(GetConnectionPointFromCellBoundary(cellBoundary.Id));
            }
            return connectionPoints;
        }

        public ConnectionPoint GetConnectionPointFromId(string id)
        {
            foreach (var cellBoundary in _cellBoundaries)
            {
                if (cellBoundary.Id == id)
                {
                    return GetConnectionPointFromCellBoundary(cellBoundary.Id);
                }
            }
            Debug.Log(id + "ConnectionPoint not found");
            return null;
        }

        public ConnectionPoint GetConnectionPointFromCellBoundary(string cellBoundaryId)
        {
            foreach (var cellBoundary in _cellBoundaries)
            {
                if (cellBoundary.Id == cellBoundaryId)
                {
                    return cellBoundary.ConvertToConnectionPoint(GetCellBoundaryFloorLevel(cellBoundary), IsStair(cellBoundary), IsElevator(cellBoundary));
                }
            }
            Debug.Log(cellBoundaryId + "ConnectionPoint not found");
            return null;
        }

        public ConnectionPoint GetConnectionPointFromCellSpace(string cellSpaceId, bool sequence = true)
        {
            // First pass: try the requested direction
            foreach (var cellBoundary in _cellBoundaries)
            {
                if (sequence)
                {
                    if (cellBoundary.Source == cellSpaceId)
                    {
                        return GetConnectionPointFromCellBoundary(cellBoundary.Id);
                    }
                }
                else
                {
                    if (cellBoundary.Target == cellSpaceId)
                    {
                        return GetConnectionPointFromCellBoundary(cellBoundary.Id);
                    }
                }
            }

            // Second pass: fallback to opposite direction if not found
            // This handles cases where CellSpace only has one-directional connections
            foreach (var cellBoundary in _cellBoundaries)
            {
                if (sequence)
                {
                    // Fallback: try as Target
                    if (cellBoundary.Target == cellSpaceId)
                    {
                        Debug.Log($"CellSpace {cellSpaceId}: Using fallback direction (Target instead of Source)");
                        return GetConnectionPointFromCellBoundary(cellBoundary.Id);
                    }
                }
                else
                {
                    // Fallback: try as Source
                    if (cellBoundary.Source == cellSpaceId)
                    {
                        Debug.Log($"CellSpace {cellSpaceId}: Using fallback direction (Source instead of Target)");
                        return GetConnectionPointFromCellBoundary(cellBoundary.Id);
                    }
                }
            }

            Debug.LogWarning($"CellSpace {cellSpaceId}: ConnectionPoint not found in any CellBoundary");
            return null;
        }

        public string GetCellBoundaryFloorLevel(CellBoundary cellBoundary)
        {
            if (cellBoundary.IsHorizontal())
            {
                CellSpace sourceCellSpace = GetCellSpaceFromId(cellBoundary.Source);
                CellSpace targetCellSpace = GetCellSpaceFromId(cellBoundary.Target);
                if (!sourceCellSpace.IsElevator() && !targetCellSpace.IsElevator())
                {
                    return sourceCellSpace.GetLocatedFloor();
                }
                else if (sourceCellSpace.IsElevator() && !targetCellSpace.IsElevator())
                {
                    return targetCellSpace.GetLocatedFloor();
                }
                else if (!sourceCellSpace.IsElevator() && targetCellSpace.IsElevator())
                {
                    return sourceCellSpace.GetLocatedFloor();
                }
            }
            return null;
        }

        public Hypergraph GetHypergraph()
        {
            Hypergraph hypergraph = new Hypergraph();

            foreach (var cellBoundary in _cellBoundaries)
            {
                ConnectionPoint connectionPoint = cellBoundary.ConvertToConnectionPoint(GetCellBoundaryFloorLevel(cellBoundary), IsStair(cellBoundary), IsElevator(cellBoundary));
                hypergraph.AddHyperNode(connectionPoint);
            }
            
            foreach (var container in _containers)
            {
                RLineGroup rLineGroup = container.ConvertToRLineGroup();
                hypergraph.AddHyperEdge(rLineGroup);
            }
            
            return hypergraph;
        }

        public void AddPickingPoint(CellSpace cellSpace)
        {
            if (cellSpace.IsPickingPoint())
            {
                List<string> boundings = cellSpace.GetBoundings();
                if (boundings != null)
                {
                    PickingPoint pickingPoint = new PickingPoint(cellSpace.Id, boundings, cellSpace.GetLocatedFloor());
                    _pickingPoints.Add(pickingPoint);
                }
            }
        }

        public void AddPackingPoint(CellSpace cellSpace)
        {
            if (cellSpace.IsPackingPoint())
            {
                List<string> boundings = cellSpace.GetBoundings();
                if (boundings != null)
                {
                    PackingPoint packingPoint = new PackingPoint(cellSpace.Id, boundings, cellSpace.GetLocatedFloor());
                    _packingPoints.Add(packingPoint);
                }
            }
        }

        public void AddBusinessPoint(CellSpace cellSpace)
        {
            if (cellSpace.IsBusinesspoint())
            {
                cellSpace.SetStatus(CellSpaceStatus.OccupiedByFacilities);
                _businessPoints.Add(cellSpace);
            }
        }

        public void AddQueueArea(CellSpace cellSpace)
        {
            if (cellSpace.IsQueue())
            {
                List<string> boundings = cellSpace.GetBoundings();
                if (boundings != null)
                {
                    QueueArea queueArea = new QueueArea(cellSpace.Id, boundings, cellSpace.GetLocatedFloor());
                    _queueAreas.Add(queueArea);
                }
            }
        }

        public Dictionary<string, Vector3> GetCellSpaceCentroids()
        {
            Dictionary<string, Vector3> cellSpaceCentroids = new Dictionary<string, Vector3>();
            foreach (var cellSpace in _cellSpaces)
            {
                float y = 0;
                string floorLevel = cellSpace.GetLocatedFloor();
                if (!string.IsNullOrEmpty(floorLevel))
                {
                    int level = ParseFloorLevel(floorLevel);
                    // Convert floor level to y coordinate
                    if (level > 0)
                    {
                        y = (level - 1) * 3.0f;
                    }
                    y = level * 3.0f;
                }
                
                cellSpaceCentroids.Add(
                    cellSpace.Id, 
                    new Vector3((float)cellSpace.Space.Centroid.X, y, (float)cellSpace.Space.Centroid.Y)
                );
            }
            return cellSpaceCentroids;
        }

        // Check if the CellBoundary is stair
        public bool IsStair(CellBoundary cellBoundary)
        {
            foreach (var cellSpace in _cellSpaces)
            {
                if (cellSpace.Id == cellBoundary.Source || cellSpace.Id == cellBoundary.Target)
                {
                    if (cellSpace.IsStair())
                    {
                        return true;
                    }
                }
            }
            return false;
        }

        public bool IsStair(string cellBoundaryId)
        {
            return IsStair(GetCellBoundaryFromId(cellBoundaryId));
        }

        // Check if the CellBoundary is elevator
        public bool IsElevator(CellBoundary cellBoundary)
        {
            foreach (var cellSpace in _cellSpaces)
            {
                if (cellSpace.Id == cellBoundary.Source || cellSpace.Id == cellBoundary.Target)
                {
                    if (cellSpace.IsElevator())
                    {
                        return true;
                    }
                }
            }
            return false;
        }

        public bool IsElevator(string cellBoundaryId)
        {
            return IsElevator(GetCellBoundaryFromId(cellBoundaryId));
        }
        
        public void LoadProperties()
        {
            foreach (var cellSpace in _cellSpaces)
            {
                AddPickingPoint(cellSpace);
                AddPackingPoint(cellSpace);
                AddBusinessPoint(cellSpace);
            }
        }

        public string ToJson()
        {
            var settings = new JsonSerializerSettings
            {
                Formatting = Formatting.Indented,
                Converters = new List<JsonConverter> { new GeometryConverter() }
            };
            return JsonConvert.SerializeObject(this, settings);
        }

        public static IndoorSpace FromJson(string json)
        {
            var settings = new JsonSerializerSettings
            {
                Converters = new List<JsonConverter> { new GeometryConverter() }
            };
            var indoorSpace = JsonConvert.DeserializeObject<IndoorSpace>(json, settings);
            return indoorSpace;
        }
    }
}