using System.Collections;
using UnityEngine;
using Newtonsoft.Json;
using NetTopologySuite.Geometries;
using System.Collections.Generic;

namespace Map {
    public class Hypergraph
    {
        public List<ConnectionPoint> HyperNodes { get; private set; }
        public List<RLineGroup> HyperEdges { get; private set; }

        public Hypergraph()
        {
            this.HyperNodes = new List<ConnectionPoint>();
            this.HyperEdges = new List<RLineGroup>();
        }

        public Hypergraph(List<ConnectionPoint> hyperNodes, List<RLineGroup> hyperEdges)
        {
            this.HyperNodes = hyperNodes;
            this.HyperEdges = hyperEdges;
        }

        public void AddHyperNode(ConnectionPoint connectionPoint)
        {
            this.HyperNodes.Add(connectionPoint);
        }

        public void AddHyperEdge(RLineGroup rLineGroup)
        {
            this.HyperEdges.Add(rLineGroup);
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
    }
}