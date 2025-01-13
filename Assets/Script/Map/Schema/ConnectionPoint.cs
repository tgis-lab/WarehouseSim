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

        public ConnectionPoint(string connectionPointId, CellBoundary cellBoundary, Geometry point)
        {
            this.Id = connectionPointId;
            this.CellBoundary = cellBoundary;
            this.Point = point;
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