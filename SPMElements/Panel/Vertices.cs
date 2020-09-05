using System;
using Autodesk.AutoCAD.Geometry;

namespace SPMElements.PanelGeometry
{
    /// <summary>
    /// Panel vertices struct.
    /// </summary>
    public struct Vertices : IEquatable<Vertices>
    {
        /// <summary>
        /// Get vertex 1 (base left vertex).
        /// </summary>
        public Point3d Vertex1 { get; }

        /// <summary>
        /// Get vertex 2 (base right vertex).
        /// </summary>
        public Point3d Vertex2 { get; }

        /// <summary>
        /// Get vertex 3 (top right vertex).
        /// </summary>
        public Point3d Vertex3 { get; }

        /// <summary>
        /// Get vertex 4 (top left vertex).
        /// </summary>
        public Point3d Vertex4 { get; }

        /// <summary>
        /// Panel vertices object.
        /// </summary>
        /// <param name="vertex1">The base left vertex.</param>
        /// <param name="vertex2">The base right vertex.</param>
        /// <param name="vertex3">The upper right vertex.</param>
        /// <param name="vertex4">The upper left vertex.</param>
        public Vertices(Point3d vertex1, Point3d vertex2, Point3d vertex3, Point3d vertex4)
        {
	        Vertex1 = vertex1;
	        Vertex2 = vertex2;
	        Vertex3 = vertex3;
	        Vertex4 = vertex4;
        }

        /// <summary>
        /// Get vertices as an array.
        /// </summary>
        public Point3d[] AsArray() => new [] {Vertex1, Vertex2, Vertex3, Vertex4};

        /// <summary>
        /// Returns true if all vertices are equal.
        /// </summary>
        /// <param name="other">The other <see cref="Vertices"/> to compare.</param>
        public bool Equals(Vertices other) => Vertex1 == other.Vertex1 && Vertex2 == other.Vertex2 && Vertex3 == other.Vertex3 && Vertex4 == other.Vertex4;

        public override bool Equals(object obj) => obj is Vertices other && Equals(other);

        public override int GetHashCode()
        {
	        var array = AsArray();
	        double result = 0;

	        foreach (var point in array)
		        result += point.X * point.Y;

	        return (int) result;
        }

        public override string ToString()
        {
	        return
		        $"Vertex 1: ({Vertex1.X:0.00}, {Vertex1.Y:0.00})\n" +
		        $"Vertex 2: ({Vertex2.X:0.00}, {Vertex2.Y:0.00})\n" +
		        $"Vertex 3: ({Vertex3.X:0.00}, {Vertex3.Y:0.00})\n" +
		        $"Vertex 4: ({Vertex4.X:0.00}, {Vertex4.Y:0.00})";
        }

        /// <summary>
        /// Returns true if arguments are equal.
        /// </summary>
        public static bool operator == (Vertices left, Vertices right) => left.Equals(right);

        /// <summary>
        /// Returns true if arguments are different.
        /// </summary>
        public static bool operator != (Vertices left, Vertices right) => !left.Equals(right);
    }
}
