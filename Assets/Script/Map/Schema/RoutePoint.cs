using UnityEngine;
using UnityEngine.UI;
using System;
using System.Collections.Generic;
using System.IO;
using Newtonsoft.Json;
using NetTopologySuite.Geometries;
using NetTopologySuite.IO;

namespace Map {
    public class RoutePoint
    {
        public ConnectionPoint ConnectionPoint { get; private set; }
        public List<ConnectionPoint> Parents { get; private set; }
        public List<ConnectionPoint> Children { get; private set; }
        public float FScore { get; private set; }

        public RoutePoint()
        {
            this.ConnectionPoint = null;
            this.Parents = new List<ConnectionPoint>();
            this.Children = new List<ConnectionPoint>();
            this.FScore = Mathf.Infinity;
        }

        public RoutePoint(ConnectionPoint connectionPoint)
        {
            this.ConnectionPoint = connectionPoint;
            this.Parents = new List<ConnectionPoint>();
            this.Children = new List<ConnectionPoint>();
            this.FScore = Mathf.Infinity;
        }

        public void AddConnectionPoint(ConnectionPoint connectionPoint)
        {
            this.ConnectionPoint = connectionPoint;
        }

        public void AddParent(ConnectionPoint parent)
        {
            this.Parents.Add(parent);
        }

        public void AddChild(ConnectionPoint child)
        {
            this.Children.Add(child);
        }

        public void RemoveParent(ConnectionPoint parent)
        {
            this.Parents.Remove(parent);
        }

        public void RemoveChild(ConnectionPoint child)
        {
            this.Children.Remove(child);
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