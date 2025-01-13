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

        public void AddCellSpace(CellSpace cellSpace)
        {
            if (_cellSpaces.Any(c => c.Id == cellSpace.Id))
                throw new System.Exception("CellSpace id already exists");
            _cellSpaces.Add(cellSpace);
        }

        public void AddCellBoundary(CellBoundary cellBoundary)
        {
            if (_cellBoundaries.Any(c => c.Id == cellBoundary.Id))
                throw new System.Exception("CellBoundary id already exists");
            if (!_cellSpaces.Any(c => c.Id == cellBoundary.Source) || !_cellSpaces.Any(c => c.Id == cellBoundary.Target))
                throw new System.Exception("Source and/or target cell do not exist");
            _cellBoundaries.Add(cellBoundary);
        }

        public void AddLayer(Layer layer)
        {
            _layers.Add(layer);
        }

        public void AddContainer(Container container)
        {
            _containers.Add(container);
        }

        public void AddPickingPoint(CellSpace cellSpace)
        {
            if (cellSpace.IsPickingPoint())
            {
                List<string> boundings = cellSpace.GetBoundings();
                if (boundings != null)
                {
                    PickingPoint pickingPoint = new PickingPoint(cellSpace.Id, boundings);
                    _pickingPoints.Add(pickingPoint);
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

        public void LoadProperties()
        {
            foreach (var cellSpace in _cellSpaces)
            {
                AddPickingPoint(cellSpace);
                AddBusinessPoint(cellSpace);
            }
            Debug.Log($"Picking Points: {PickingPoints.Count}, Business Points: {BusinessPoints.Count}");
        }

        public PickingPoint GetPickingPointFromBusinessPoint(CellSpace cellSpace)
        {
            if (cellSpace.IsBusinesspoint())
            {
                foreach (var pickingPoint in _pickingPoints)
                {
                    if (pickingPoint.Boundings.Contains(cellSpace.Id))
                    {
                        return pickingPoint;
                    }
                }
            }
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

        public CellSpace GetCellSpaceFromCoordinates(Vector3 position)
        {
            foreach (var cellSpace in _cellSpaces)
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

        public List<ConnectionPoint> GetAllConnectionPoints()
        {
            List<ConnectionPoint> connectionPoints = new List<ConnectionPoint>();
            foreach (var cellBoundary in _cellBoundaries)
            {
                connectionPoints.Add(cellBoundary.ConvertToConnectionPoint());
            }
            return connectionPoints;
        }

        public ConnectionPoint GetConnectionPointFromId(string id)
        {
            foreach (var cellBoundary in _cellBoundaries)
            {
                if (cellBoundary.Id == id)
                {
                    return cellBoundary.ConvertToConnectionPoint();
                }
            }
            Debug.Log(id + "ConnectionPoint not found");
            return null;
        }

        public ConnectionPoint GetConnectionPointFromCellSpace(string cellSpaceId, bool sequence = true)
        {
            foreach (var cellBoundary in _cellBoundaries)
            {
                if (sequence)
                {
                    if (cellBoundary.Source == cellSpaceId)
                    {
                        return cellBoundary.ConvertToConnectionPoint();
                    }
                }
                else
                {
                    if (cellBoundary.Target == cellSpaceId)
                    {
                        return cellBoundary.ConvertToConnectionPoint();
                    }
                }
            }
            Debug.Log(cellSpaceId);
            Debug.Log("ConnectionPoint not found");
            return null;
        }

        public RLineGroup GetRlineGroupFromCellSpaceId(string id)
        {
            foreach (var container in _containers)
            {
                if (container.CellSpaceId == id)
                {
                    return container.ConvertToRLineGroup();
                }
            }
            Debug.Log("RLineGroup not found");
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

        public Layer GetLayerFromCellSpaceId(string id)
        {
            foreach (var layer in _layers)
            {
                if (layer.CellSpaces.Contains(id))
                {
                    return layer;
                }
            }
            Debug.Log("Layer not found");
            return null;
        }

        public Hypergraph GetHypergraph()
        {
            Hypergraph hypergraph = new Hypergraph();
            foreach (var cellBoundary in _cellBoundaries)
            {
                ConnectionPoint connectionPoint = cellBoundary.ConvertToConnectionPoint();
                hypergraph.AddHyperNode(connectionPoint);
            }
            foreach (var container in _containers)
            {
                RLineGroup rLineGroup = container.ConvertToRLineGroup();
                hypergraph.AddHyperEdge(rLineGroup);
            }
            return hypergraph;
        }

        public Dictionary<string, Vector3> GetCellSpaceCentroids()
        {
            Dictionary<string, Vector3> cellSpaceCentroids = new Dictionary<string, Vector3>();
            foreach (var cellSpace in _cellSpaces)
            {
                cellSpaceCentroids.Add(cellSpace.Id, new Vector3((float)cellSpace.Space.Centroid.X, 0, (float)cellSpace.Space.Centroid.Y));
            }
            return cellSpaceCentroids;
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
