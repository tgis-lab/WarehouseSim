using System.Collections;
using UnityEngine;
using Newtonsoft.Json;
using NetTopologySuite.Geometries;
using NetTopologySuite.IO;
using System.Collections.Generic;
using System;

namespace Map {
    public enum TransitionType
    {
        Horizontal,
        Vertical
    }

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
        
        [JsonProperty("transitionType")]
        public string TransitionTypeString { get; private set; }
        
        [JsonIgnore]
        public TransitionType TransitionType 
        { 
            get 
            {
                if (string.IsNullOrEmpty(TransitionTypeString))
                    return TransitionType.Horizontal; // Default
                
                return TransitionTypeString.ToLower() == "vertical" ? 
                    TransitionType.Vertical : TransitionType.Horizontal;
            }
        }

        public CellBoundary(string cellBoundaryId, Dictionary<string, object> properties, 
                           string from, string to, Geometry boundary, Geometry edge, 
                           string transitionType = "horizontal")
        {
            this.Id = cellBoundaryId;
            this.Properties = properties;
            this.Source = from;
            this.Target = to;
            this.Boundary = boundary;
            this.Edge = edge;
            this.TransitionTypeString = transitionType;
        }
        
        public ConnectionPoint ConvertToConnectionPoint(string FloorLevel, bool isStair, bool isElevator,
            CellSpace sourceCellSpace = null, CellSpace targetCellSpace = null)
        {
            // For vertical transitions, we create a synthetic centroid since there's no geometry
            if (TransitionType == TransitionType.Vertical)
            {
                var geometryFactory = new GeometryFactory();
                Point syntheticPoint;

                if (sourceCellSpace?.Node != null && targetCellSpace?.Node != null)
                {
                    double midX = (sourceCellSpace.Node.Centroid.X + targetCellSpace.Node.Centroid.X) / 2.0;
                    double midY = (sourceCellSpace.Node.Centroid.Y + targetCellSpace.Node.Centroid.Y) / 2.0;
                    syntheticPoint = geometryFactory.CreatePoint(new Coordinate(midX, midY));
                }
                else if (sourceCellSpace?.Node != null)
                {
                    syntheticPoint = geometryFactory.CreatePoint(new Coordinate(
                        sourceCellSpace.Node.Centroid.X, sourceCellSpace.Node.Centroid.Y));
                }
                else if (targetCellSpace?.Node != null)
                {
                    syntheticPoint = geometryFactory.CreatePoint(new Coordinate(
                        targetCellSpace.Node.Centroid.X, targetCellSpace.Node.Centroid.Y));
                }
                else
                {
                    syntheticPoint = geometryFactory.CreatePoint(new Coordinate(0, 0));
                }

                return new ConnectionPoint(this.Id, this, syntheticPoint, FloorLevel, isStair, isElevator);
            }

            // For horizontal transitions, use the existing boundary centroid
            return new ConnectionPoint(this.Id, this, this.Boundary != null ? this.Boundary.Centroid : null, FloorLevel, isStair, isElevator);
        }
        
        // Check if this is a vertical transition
        public bool IsVertical()
        {
            return TransitionType == TransitionType.Vertical;
        }
        
        // Check if this is a horizontal transition
        public bool IsHorizontal()
        {
            return TransitionType == TransitionType.Horizontal;
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