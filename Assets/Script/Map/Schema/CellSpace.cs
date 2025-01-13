using System;
using System.Collections;
using System.Linq;
using UnityEngine;
using Newtonsoft.Json;
using NetTopologySuite.Geometries;
using System.Collections.Generic;
using Newtonsoft.Json.Linq;

namespace Map {
    public enum CellSpaceStatus
    {
        Free, 
        OccupiedByFacilities
    }

    public class CellSpace
    {
        [JsonProperty("id")]
        public string Id { get; private set; }
        public Dictionary<string, object> Properties { get; private set; }
        public Geometry Space { get; private set; }
        public Geometry Node { get; private set; }
        public CellSpaceStatus Status;


        public CellSpace(string cellId, Dictionary<string, object> properties, Geometry space, Geometry node)
        {
            this.Id = cellId;
            this.Properties = properties;
            this.Space = space;
            this.Node = node;
            this.Status = CellSpaceStatus.Free;
        }

        private bool HasFunctionOfType(string type) {
            if (Properties.TryGetValue("functions", out object functionsObj)) {

                if (functionsObj is JArray functionsJArray) {
                    var functions = functionsJArray.ToObject<List<Dictionary<string, object>>>();
                    return functions.Any(function => function.TryGetValue("type", out object typeObj) &&
                                                    typeObj is string typeString &&
                                                    string.Equals(typeString, type, StringComparison.OrdinalIgnoreCase));
                }
            }
            return false;
        }

        public bool IsBusinesspoint() {
            return HasFunctionOfType("shelf");
        }

        public bool IsPickingPoint() {
            return HasFunctionOfType("picking point");
        }

        public bool IsNavigable()
        {
            if (Properties.TryGetValue("navigable", out object propertiesObj) && propertiesObj is bool navigable && navigable)
            {
                return true;
            }
            return false;
        }

        public CellSpaceStatus GetStatus()
        {
            return this.Status;
        }

        public void SetStatus(CellSpaceStatus status)
        {
            this.Status = status;
        }

        public List<string> GetBoundings()
        {
            if (Properties.TryGetValue("functions", out object functionsObj) && functionsObj is JArray functionsJArray)
            {
                var functions = functionsJArray.ToObject<List<Dictionary<string, object>>>();
                foreach (var function in functions)
                {
                    if (function.TryGetValue("type", out object typeObj) && typeObj as string == "picking point")
                    {
                        if (function.TryGetValue("features", out object featuresObj) && featuresObj is JObject featuresJObject)
                        {
                            if (featuresJObject.TryGetValue("boundings", out JToken boundingsJToken) && boundingsJToken is JArray boundingsJArray)
                            {
                                return boundingsJArray.ToObject<List<string>>();
                            }
                        }
                    }
                }
            }
            return null;
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

        public static CellSpace FromJson(string json)
        {
            var settings = new JsonSerializerSettings
            {
                Converters = new List<JsonConverter> { new GeometryConverter() }
            };
            var cellSpace = JsonConvert.DeserializeObject<CellSpace>(json, settings);
            return cellSpace;
        }
    }
}

