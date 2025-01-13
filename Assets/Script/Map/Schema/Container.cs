using System.Collections;
using UnityEngine;
using Newtonsoft.Json;
using NetTopologySuite.Geometries;
using System.Collections.Generic;
using System.Linq;

namespace Map {
    public class Container
    {
        [JsonProperty("id")]
        public string Id { get; private set; }
        [JsonProperty("cell")]
        public string CellSpaceId { get; private set; }
        [JsonProperty("source")]
        public List<string> Ins { get; private set; }
        [JsonProperty("target")]
        public List<string> Outs { get; private set; }
        [JsonProperty("limited")]
        public List<List<string>> Limited { get; private set; }

        public Container(string rlineId, string cellSpaceId, List<string> ins, List<string> outs, List<List<string>> limited)
        {
            this.Id = rlineId;
            this.CellSpaceId = cellSpaceId;
            this.Ins = ins;
            this.Outs = outs;
            this.Limited = limited;
        }

        public RLineGroup ConvertToRLineGroup()
        {
            List<string> connectionPoints = new List<string>();
            if (this.Ins != null)
            {
                foreach (var source in this.Ins)
                {
                    connectionPoints.Add(source);
                }
            }
            if (this.Outs != null)
            {
                foreach (var target in this.Outs)
                {
                    connectionPoints.Add(target);
                }
            }

            List<List<string>> InnerEdges = new List<List<string>>();
            if (this.Ins != null && this.Outs != null)
            foreach (var source in this.Ins)
            {
                foreach (var target in this.Outs)
                {
                    List<string> innerEdge = new List<string>();
                    innerEdge.Add(source);
                    innerEdge.Add(target);
                    if (this.Limited == null || !this.Limited.Any(limited => limited.SequenceEqual(innerEdge)))
                    {
                        InnerEdges.Add(innerEdge);
                    }
                }
            }

            RLineGroup rLineGroup = new RLineGroup(this.Id, this.Ins, this.Outs, connectionPoints, InnerEdges);

            return rLineGroup;
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

        public static Container FromJson(string json)
        {
            var settings = new JsonSerializerSettings
            {
                Converters = new List<JsonConverter> { new GeometryConverter() }
            };
            var container = JsonConvert.DeserializeObject<Container>(json, settings);
            return container;
        }
    }
}