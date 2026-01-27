using System.Collections;
using UnityEngine;
using Newtonsoft.Json;
using NetTopologySuite.Geometries;
using System.Collections.Generic;

namespace Map {
    public class ConnectionPoint
    {
        public string Id { get; private set; }
        public CellBoundary CellBoundary { get; private set; }
        public Geometry Point { get; private set; }
        public string FloorLevel { get; private set; }
        public bool IsStair { get; private set; }
        public bool IsElevator { get; private set; }
        
        [JsonIgnore]
        private Vector3? cachedPosition;
        
        [JsonIgnore]
        public Vector3 Position
        {
            get
            {
                if (!cachedPosition.HasValue)
                {
                    if (Point is Point p)
                    {
                        cachedPosition = new Vector3((float)p.X, 0, (float)p.Y);
                    }
                    else
                    {
                        cachedPosition = Vector3.zero;
                    }
                }
                return cachedPosition.Value;
            }
        }

        public ConnectionPoint(string connectionPointId, CellBoundary cellBoundary, Geometry point, string floorLevel, bool isStair, bool isElevator)
        {
            this.Id = connectionPointId;
            this.CellBoundary = cellBoundary;
            this.Point = point;
            this.FloorLevel = floorLevel;
            this.IsStair = isStair;
            this.IsElevator = isElevator;
        }
        
        // Check if this connection point represents a vertical transition
        public bool IsVertical()
        {
            return CellBoundary != null && CellBoundary.IsVertical();
        }
        
        // Check if this connection point represents a horizontal transition
        public bool IsHorizontal()
        {
            return CellBoundary != null && CellBoundary.IsHorizontal();
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
