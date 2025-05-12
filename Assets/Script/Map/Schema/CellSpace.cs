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
    
    public class VerticalFeatures
    {
        [JsonProperty("verticalType")]
        public string VerticalType { get; set; } // null, "elevator", "stair"
        
        [JsonProperty("floorAssociation")]
        public List<FloorAssociation> FloorAssociations { get; set; }
        
        public VerticalFeatures()
        {
            FloorAssociations = new List<FloorAssociation>();
        }
    }
    
    public class FloorAssociation
    {
        [JsonProperty("floorLevel")]
        public string FloorLevel { get; set; }
        
        [JsonProperty("associationType")]
        public string AssociationType { get; set; }
    }

    public class CellSpace
    {
        [JsonProperty("id")]
        public string Id { get; private set; }
        
        [JsonProperty("properties")]
        public Dictionary<string, object> Properties { get; private set; }
        
        [JsonProperty("space")]
        public Geometry Space { get; private set; }
        
        [JsonProperty("node")]
        public Geometry Node { get; private set; }
        
        public CellSpaceStatus Status { get; set; }

        public CellSpace(string cellId, Dictionary<string, object> properties, Geometry space, Geometry node)
        {
            this.Id = cellId;
            this.Properties = properties;
            this.Space = space;
            this.Node = node;
            this.Status = CellSpaceStatus.Free;
        }
        
        // Get vertical features
        public VerticalFeatures GetVerticalFeatures()
        {
            if (Properties.TryGetValue("verticalFeatures", out object verticalFeaturesObj))
            {
                if (verticalFeaturesObj is JObject verticalFeaturesJObject)
                {
                    return verticalFeaturesJObject.ToObject<VerticalFeatures>();
                }
            }
            return null;
        }
        
        // Get the floor where this cell is located
        public string GetLocatedFloor()
        {
            var verticalFeatures = GetVerticalFeatures();
            if (verticalFeatures != null && verticalFeatures.FloorAssociations != null)
            {
                var locatedFloor = verticalFeatures.FloorAssociations
                    .FirstOrDefault(fa => fa.AssociationType == "located");
                    
                if (locatedFloor != null)
                    return locatedFloor.FloorLevel;
            }
            return null;
        }
        
        // Check if this is a vertical connection cell
        public bool IsVerticalConnection()
        {
            if (Properties.TryGetValue("isVerticalConnection", out object isVerticalConnectionObj) &&
                isVerticalConnectionObj is bool isVerticalConnection)
            {
                return isVerticalConnection;
            }
            return false;
        }
        
        // Check if this cell is an elevator
        public bool IsElevator()
        {
            var verticalFeatures = GetVerticalFeatures();
            return verticalFeatures != null && 
                   verticalFeatures.VerticalType == "elevator";
        }
        
        // Check if this cell is a staircase
        public bool IsStair()
        {
            var verticalFeatures = GetVerticalFeatures();
            return verticalFeatures != null && 
                   verticalFeatures.VerticalType == "stair";
        }
        
        // Get all floors this cell is connected to
        public List<string> GetConnectedFloors()
        {
            var verticalFeatures = GetVerticalFeatures();
            if (verticalFeatures == null || verticalFeatures.FloorAssociations == null)
                return new List<string>();
                
            return verticalFeatures.FloorAssociations
                .Where(fa => fa.AssociationType == "connected")
                .Select(fa => fa.FloorLevel)
                .ToList();
        }
        
        // Check if this cell is located on a specific floor
        public bool IsLocatedOnFloor(string floorLevel)
        {
            return GetLocatedFloor() == floorLevel;
        }
        
        // Check if this cell connects to a specific floor
        public bool ConnectsToFloor(string floorLevel)
        {
            return GetConnectedFloors().Contains(floorLevel);
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

        public bool IsQueue()
        {
            return HasFunctionOfType("queue area");
        }

        public bool IsPackingPoint()
        {
            return HasFunctionOfType("packing point");
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