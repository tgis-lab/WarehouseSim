using System.Collections;
using UnityEngine;
using Newtonsoft.Json;
using NetTopologySuite.Geometries;
using System.Collections.Generic;

namespace Map {
    public class QueueArea
    {
        public string Id { get; private set; }
        public List<string> Boundings { get; private set; }
        public string FloorLevel { get; private set; }

        public QueueArea(string id, List<string> boundings, string floorLevel)
        {
            Id = id;
            Boundings = boundings;
            FloorLevel = floorLevel;
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

