using System.Collections;
using UnityEngine;
using Newtonsoft.Json;
using NetTopologySuite.Geometries;
using System.Collections.Generic;

namespace Map {
    public class RLineGroup
    {
        public string Id { get; private set; }
        public List<string> Sources { get; private set; }
        public List<string> Targets { get; private set; }
        public List<string> ConnectionPoints { get; private set; }
        public List<List<string>> InnerEdges { get; private set; }

        public RLineGroup()
        {
            this.Id = "";
            this.Sources = new List<string>();
            this.Targets = new List<string>();
            this.ConnectionPoints = new List<string>();
            this.InnerEdges = new List<List<string>>();
        }

        public RLineGroup(string id, List<string> sources, List<string> targets, List<string> connectionPoints, List<List<string>> innerEdges)
        {
            this.Id = id;
            this.Sources = sources;
            this.Targets = targets;
            this.ConnectionPoints = connectionPoints;
            this.InnerEdges = innerEdges;
        }

        public void AddId(string id)
        {
            this.Id = id;
        }

        public void AddSource(string source)
        {
            this.Sources.Add(source);
        }

        public void AddTarget(string target)
        {
            this.Targets.Add(target);
        }

        public void AddConnectionPoint(string connectionPointId)
        {
            this.ConnectionPoints.Add(connectionPointId);
        }

        public void AddInnerEdge(List<string> innerEdge)
        {
            this.InnerEdges.Add(innerEdge);
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