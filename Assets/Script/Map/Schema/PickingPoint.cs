using System.Collections;
using UnityEngine;
using Newtonsoft.Json;
using NetTopologySuite.Geometries;
using System.Collections.Generic;

namespace Map {
    public class PickingPoint
    {
        public string Id { get; private set; }
        public List<string> Boundings { get; private set; }

        public PickingPoint(string id, List<string> boundings)
        {
            Id = id;
            Boundings = boundings;
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

