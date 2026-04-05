using System.Collections;
using System;
using UnityEngine;
using Newtonsoft.Json;
using NetTopologySuite.Geometries;
using System.Collections.Generic;

namespace Map {
    public class Layer
    {
        [JsonProperty("id")]
        public string Id { get; private set; }
        [JsonProperty("cells")]
        public List<string> CellSpaces { get; private set; }
        [JsonProperty("properties")]
        public Dictionary<string, object> Properties { get; private set; }

        public Layer(string layerId, List<string> cellSpaces, Dictionary<string, object> properties)
        {
            this.Id = layerId;
            this.CellSpaces = cellSpaces;
            this.Properties = properties;
        }

        public float SpeedLimit()
        {
            if (this.Properties.TryGetValue("speed", out var speed) && speed != null)
            {
                return Convert.ToSingle(speed);
            }
            return 1.0f;
        }

        public float DeaccelerationFactor()
        {
            if (this.Properties.ContainsKey("deaccelerationFactor"))
            {
                return Convert.ToSingle(this.Properties["deaccelerationFactor"]);
            }
            return 1f;
        }


        public bool IsIntersection()
        {
            return this.Properties.TryGetValue("type", out var t) && "intersection".Equals(t);
        }

        public bool IsAisle()
        {
            return this.Properties.TryGetValue("type", out var t) && "aisle".Equals(t);
        }

        public bool IsQueue()
        {
            return this.Properties.TryGetValue("type", out var t) && "queue".Equals(t);
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

        public static Layer FromJson(string json)
        {
            var settings = new JsonSerializerSettings
            {
                Converters = new List<JsonConverter> { new GeometryConverter() }
            };
            var layer = JsonConvert.DeserializeObject<Layer>(json, settings);
            return layer;
        }
    }
}