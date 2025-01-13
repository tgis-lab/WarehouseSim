using Newtonsoft.Json;
using NetTopologySuite.Geometries;
using NetTopologySuite.IO;
using System;

namespace Map {
    public class GeometryConverter : JsonConverter
    {
        public override bool CanConvert(Type objectType)
        {
            return typeof(Geometry).IsAssignableFrom(objectType);
        }

        public override void WriteJson(JsonWriter writer, object value, JsonSerializer serializer)
        {
            Geometry geometry = value as Geometry;
            if (geometry != null)
            {
                WKTWriter wktWriter = new WKTWriter();
                writer.WriteValue(wktWriter.Write(geometry));
            }
            else
            {
                writer.WriteNull();
            }
        }

        public override object ReadJson(JsonReader reader, Type objectType, object existingValue, JsonSerializer serializer)
        {
            if (reader.TokenType == JsonToken.String)
            {
                string wkt = (string)reader.Value;
                WKTReader wktReader = new WKTReader();
                return wktReader.Read(wkt);
            }
            return null;
        }
    }
}
