using System.Collections;
using UnityEngine;
using Newtonsoft.Json;
using NetTopologySuite.Geometries;
using NetTopologySuite.IO;
using System.Collections.Generic;

namespace Map {
    public class CellBoundary
    {
        [JsonProperty("id")]
        public string Id { get; private set; }
        [JsonProperty("properties")]
        public Dictionary<string, object> Properties { get; private set; }
        [JsonProperty("source")]
        public string Source { get; private set; }
        [JsonProperty("target")]
        public string Target { get; private set; }
        [JsonProperty("boundary")]
        public Geometry Boundary { get; private set; }
        [JsonProperty("edge")]
        public Geometry Edge { get; private set; }

        public CellBoundary(string cellBoundaryId, Dictionary<string, object> properties, string from, string to, Geometry boundary, Geometry edge)
        {
            this.Id = cellBoundaryId;
            this.Properties = properties;
            this.Source = from;
            this.Target = to;
            this.Boundary = boundary;
            this.Edge = edge;
        }
        
        public ConnectionPoint ConvertToConnectionPoint()
        {
            return new ConnectionPoint(this.Id, this, this.Boundary.Centroid);
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

        public static CellBoundary FromJson(string json)
        {
            var settings = new JsonSerializerSettings
            {
                Converters = new List<JsonConverter> { new GeometryConverter() }
            };
            var cellBoundary = JsonConvert.DeserializeObject<CellBoundary>(json, settings);
            return cellBoundary;
        }
    }
}
