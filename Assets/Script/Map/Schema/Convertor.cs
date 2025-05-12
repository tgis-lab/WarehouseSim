using Newtonsoft.Json;
using NetTopologySuite.Geometries;
using NetTopologySuite.IO;
using System;
using UnityEngine;

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
            if (reader.TokenType == JsonToken.Null)
            {
                return null;
            }
            
            if (reader.TokenType == JsonToken.String)
            {
                try
                {
                    string wkt = (string)reader.Value;
                    if (string.IsNullOrEmpty(wkt))
                    {
                        return null;
                    }
                    
                    WKTReader wktReader = new WKTReader();
                    return wktReader.Read(wkt);
                }
                catch (Exception ex)
                {
                    Debug.LogError($"Error parsing WKT: {ex.Message}");
                    return null;
                }
            }
            
            Debug.LogWarning($"Unexpected token type when deserializing geometry: {reader.TokenType}");
            return null;
        }
    }
}