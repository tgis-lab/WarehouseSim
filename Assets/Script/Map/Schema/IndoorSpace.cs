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

        // --- O(1) lookup indexes (built lazily on first access) ---
        private bool _indexesBuilt = false;
        private Dictionary<string, CellSpace> _cellSpaceIndex;
        private Dictionary<string, CellBoundary> _cellBoundaryIndex;
        private Dictionary<string, List<CellBoundary>> _cellToBoundariesAsSource;
        private Dictionary<string, List<CellBoundary>> _cellToBoundariesAsTarget;
        private Dictionary<string, List<CellSpace>> _floorIndex;
        private Dictionary<string, ConnectionPoint> _connectionPointIndex;

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

        /// <summary>
        /// Builds all dictionary indexes from the current CellSpaces and CellBoundaries lists.
        /// Called lazily on first access via EnsureIndexes().
        /// </summary>
        private void BuildIndexes()
        {
            _cellSpaceIndex = new Dictionary<string, CellSpace>(_cellSpaces.Count);
            foreach (var cs in _cellSpaces)
            {
                _cellSpaceIndex[cs.Id] = cs;
            }

            _cellBoundaryIndex = new Dictionary<string, CellBoundary>(_cellBoundaries.Count);
            _cellToBoundariesAsSource = new Dictionary<string, List<CellBoundary>>();
            _cellToBoundariesAsTarget = new Dictionary<string, List<CellBoundary>>();
            foreach (var cb in _cellBoundaries)
            {
                _cellBoundaryIndex[cb.Id] = cb;

                if (!_cellToBoundariesAsSource.TryGetValue(cb.Source, out var sourceList))
                {
                    sourceList = new List<CellBoundary>();
                    _cellToBoundariesAsSource[cb.Source] = sourceList;
                }
                sourceList.Add(cb);

                if (!_cellToBoundariesAsTarget.TryGetValue(cb.Target, out var targetList))
                {
                    targetList = new List<CellBoundary>();
                    _cellToBoundariesAsTarget[cb.Target] = targetList;
                }
                targetList.Add(cb);
            }

            _floorIndex = new Dictionary<string, List<CellSpace>>();
            foreach (var cs in _cellSpaces)
            {
                string floor = cs.GetLocatedFloor();
                if (!string.IsNullOrEmpty(floor))
                {
                    if (!_floorIndex.TryGetValue(floor, out var floorList))
                    {
                        floorList = new List<CellSpace>();
                        _floorIndex[floor] = floorList;
                    }
                    floorList.Add(cs);
                }
            }

            _connectionPointIndex = new Dictionary<string, ConnectionPoint>(_cellBoundaries.Count);

            _indexesBuilt = true;
        }

        /// <summary>
        /// Ensures indexes are built before any lookup. Safe for lazy init.
        /// </summary>
        private void EnsureIndexes()
        {
            if (!_indexesBuilt)
            {
                BuildIndexes();
            }
        }
        
        public List<string> GetAllFloorLevels()
        {
            EnsureIndexes();
            return _floorIndex.Keys.OrderBy(f => ParseFloorLevel(f)).ToList();
        }
        
        private int ParseFloorLevel(string floorLevel)
        {
            return FloorLevelUtils.Parse(floorLevel);
        }
        
        public List<CellSpace> GetCellSpacesOnFloor(string floorLevel)
        {
            EnsureIndexes();
            if (_floorIndex.TryGetValue(floorLevel, out var cells))
            {
                return new List<CellSpace>(cells);
            }
            return new List<CellSpace>();
        }

        public List<PickingPoint> GetPickingPointsOnFloor(string floorLevel)
        {
            return _pickingPoints.Where(pp => pp.FloorLevel == floorLevel).ToList();
        }
        
        public List<CellSpace> GetNavigableCellSpacesOnFloor(string floorLevel)
        {
            EnsureIndexes();
            if (_floorIndex.TryGetValue(floorLevel, out var cells))
            {
                return cells.Where(cs => cs.IsNavigable()).ToList();
            }
            return new List<CellSpace>();
        }

        public List<CellSpace> GetVerticalConnectionsOnFloor(string floorLevel)
        {
            EnsureIndexes();
            if (_floorIndex.TryGetValue(floorLevel, out var cells))
            {
                return cells.Where(cs => cs.IsVerticalConnection()).ToList();
            }
            return new List<CellSpace>();
        }

        public List<CellSpace> GetElevatorsOnFloor(string floorLevel)
        {
            EnsureIndexes();
            if (_floorIndex.TryGetValue(floorLevel, out var cells))
            {
                return cells.Where(cs => cs.IsElevator()).ToList();
            }
            return new List<CellSpace>();
        }

        public List<CellSpace> GetStairsOnFloor(string floorLevel)
        {
            EnsureIndexes();
            if (_floorIndex.TryGetValue(floorLevel, out var cells))
            {
                return cells.Where(cs => cs.IsStair()).ToList();
            }
            return new List<CellSpace>();
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
            EnsureIndexes();
            if (_cellSpaceIndex.TryGetValue(cellBoundary.Source, out var sourceCell) && sourceCell.IsElevator())
            {
                return sourceCell.Id;
            }
            if (_cellSpaceIndex.TryGetValue(cellBoundary.Target, out var targetCell) && targetCell.IsElevator())
            {
                return targetCell.Id;
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
            EnsureIndexes();
            if (_cellSpaceIndex.TryGetValue(cellBoundary.Source, out var sourceCell) && sourceCell.IsStair())
            {
                return sourceCell.Id;
            }
            if (_cellSpaceIndex.TryGetValue(cellBoundary.Target, out var targetCell) && targetCell.IsStair())
            {
                return targetCell.Id;
            }
            Debug.Log("Stair not found");
            return null;
        }

        public CellSpace GetCellSpaceFromId(string id)
        {
            EnsureIndexes();
            if (_cellSpaceIndex.TryGetValue(id, out var cellSpace))
            {
                return cellSpace;
            }
            Debug.Log("CellSpace not found");
            return null;
        }

        public CellSpace GetCellSpaceFromConnectionPoint(ConnectionPoint connectionPoint, bool sequence = true)
        {
            EnsureIndexes();
            // true: source, false: target
            if (_cellBoundaryIndex.TryGetValue(connectionPoint.Id, out var cellBoundary))
            {
                return GetCellSpaceFromId(sequence ? cellBoundary.Source : cellBoundary.Target);
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
            EnsureIndexes();
            if (_cellBoundaryIndex.TryGetValue(id, out var cellBoundary))
            {
                return cellBoundary;
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
            EnsureIndexes();
            if (_cellBoundaryIndex.ContainsKey(id))
            {
                return GetConnectionPointFromCellBoundary(id);
            }
            Debug.Log(id + "ConnectionPoint not found");
            return null;
        }

        public ConnectionPoint GetConnectionPointFromCellBoundary(string cellBoundaryId)
        {
            EnsureIndexes();
            if (_connectionPointIndex.TryGetValue(cellBoundaryId, out var cached))
            {
                return cached;
            }
            if (_cellBoundaryIndex.TryGetValue(cellBoundaryId, out var cellBoundary))
            {
                var cp = cellBoundary.ConvertToConnectionPoint(GetCellBoundaryFloorLevel(cellBoundary), IsStair(cellBoundary), IsElevator(cellBoundary));
                _connectionPointIndex[cellBoundaryId] = cp;
                return cp;
            }
            Debug.Log(cellBoundaryId + "ConnectionPoint not found");
            return null;
        }

        public ConnectionPoint GetConnectionPointFromCellSpace(string cellSpaceId, bool sequence = true)
        {
            EnsureIndexes();

            // First pass: try the requested direction
            if (sequence)
            {
                if (_cellToBoundariesAsSource.TryGetValue(cellSpaceId, out var sourceList) && sourceList.Count > 0)
                {
                    return GetConnectionPointFromCellBoundary(sourceList[0].Id);
                }
            }
            else
            {
                if (_cellToBoundariesAsTarget.TryGetValue(cellSpaceId, out var targetList) && targetList.Count > 0)
                {
                    return GetConnectionPointFromCellBoundary(targetList[0].Id);
                }
            }

            // Second pass: fallback to opposite direction if not found
            // This handles cases where CellSpace only has one-directional connections
            if (sequence)
            {
                // Fallback: try as Target
                if (_cellToBoundariesAsTarget.TryGetValue(cellSpaceId, out var targetList) && targetList.Count > 0)
                {
                    Debug.Log($"CellSpace {cellSpaceId}: Using fallback direction (Target instead of Source)");
                    return GetConnectionPointFromCellBoundary(targetList[0].Id);
                }
            }
            else
            {
                // Fallback: try as Source
                if (_cellToBoundariesAsSource.TryGetValue(cellSpaceId, out var sourceList) && sourceList.Count > 0)
                {
                    Debug.Log($"CellSpace {cellSpaceId}: Using fallback direction (Source instead of Target)");
                    return GetConnectionPointFromCellBoundary(sourceList[0].Id);
                }
            }

            Debug.LogWarning($"CellSpace {cellSpaceId}: ConnectionPoint not found in any CellBoundary");
            return null;
        }

        public string GetCellBoundaryFloorLevel(CellBoundary cellBoundary)
        {
            if (cellBoundary == null) return null;

            CellSpace sourceCellSpace = GetCellSpaceFromId(cellBoundary.Source);
            CellSpace targetCellSpace = GetCellSpaceFromId(cellBoundary.Target);

            if (sourceCellSpace == null && targetCellSpace == null) return null;
            if (sourceCellSpace == null) return targetCellSpace?.GetLocatedFloor();
            if (targetCellSpace == null) return sourceCellSpace.GetLocatedFloor();

            if (cellBoundary.IsHorizontal())
            {
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

            // Vertical boundary or both-elevator: fallback to source cell floor
            return sourceCellSpace.GetLocatedFloor();
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
            EnsureIndexes();
            if (_cellSpaceIndex.TryGetValue(cellBoundary.Source, out var sourceCell) && sourceCell.IsStair())
            {
                return true;
            }
            if (_cellSpaceIndex.TryGetValue(cellBoundary.Target, out var targetCell) && targetCell.IsStair())
            {
                return true;
            }
            return false;
        }

        public bool IsStair(string cellBoundaryId)
        {
            CellBoundary cb = GetCellBoundaryFromId(cellBoundaryId);
            if (cb == null) return false;
            return IsStair(cb);
        }

        // Check if the CellBoundary is elevator
        public bool IsElevator(CellBoundary cellBoundary)
        {
            EnsureIndexes();
            if (_cellSpaceIndex.TryGetValue(cellBoundary.Source, out var sourceCell) && sourceCell.IsElevator())
            {
                return true;
            }
            if (_cellSpaceIndex.TryGetValue(cellBoundary.Target, out var targetCell) && targetCell.IsElevator())
            {
                return true;
            }
            return false;
        }

        public bool IsElevator(string cellBoundaryId)
        {
            CellBoundary cb = GetCellBoundaryFromId(cellBoundaryId);
            if (cb == null) return false;
            return IsElevator(cb);
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