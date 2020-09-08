using System;
using Autodesk.AutoCAD.Geometry;
using Extensions;
using Extensions.AutoCAD;
using UnitsNet.Units;

namespace SPMElements.PanelProperties
{
    /// <summary>
    /// Panel vertices struct.
    /// </summary>
    public struct Vertices : IEquatable<Vertices>
    {
		// Auxiliary
		private Point3d _vertex1, _vertex2, _vertex3, _vertex4;
		private Point3d? _centerPoint;
		private double[] _xCoordinates, _yCoordinates;

		/// <summary>
		/// Get the <see cref="LengthUnit"/> of vertices' coordinates.
		/// </summary>
		public LengthUnit Unit { get ; private set; }

		/// <summary>
		/// Get vertex 1 (base left vertex).
		/// </summary>
		public Point3d Vertex1 => _vertex1;

		/// <summary>
		/// Get vertex 2 (base right vertex).
		/// </summary>
		public Point3d Vertex2 => _vertex2;

		/// <summary>
		/// Get vertex 3 (top right vertex).
		/// </summary>
		public Point3d Vertex3 => _vertex3;

		/// <summary>
		/// Get vertex 4 (top left vertex).
		/// </summary>
		public Point3d Vertex4 => _vertex4;

		/// <summary>
		/// Get <see cref="Vertices"/> approximated center point.
		/// </summary>
		public Point3d CenterPoint => _centerPoint ?? CalculateCenterPoint();

		/// <summary>
		/// Get X coordinates (in mm) of vertices as an array.
		/// </summary>
		public double[] XCoordinates => _xCoordinates ?? VertexCoordinates().x;

		/// <summary>
		/// Get Y coordinates (in mm) of vertices as an array.
		/// </summary>
		public double[] YCoordinates => _yCoordinates ?? VertexCoordinates().y;

        /// <summary>
        /// Panel vertices object.
        /// </summary>
        /// <param name="vertex1">The base left vertex.</param>
        /// <param name="vertex2">The base right vertex.</param>
        /// <param name="vertex3">The upper right vertex.</param>
        /// <param name="vertex4">The upper left vertex.</param>
        /// <param name="geometryUnit">The <see cref="LengthUnit"/> of vertices' coordinates.</param>
        public Vertices(Point3d vertex1, Point3d vertex2, Point3d vertex3, Point3d vertex4, LengthUnit geometryUnit = LengthUnit.Millimeter)
        {
	        Unit = geometryUnit;

	        _vertex1 = vertex1;
	        _vertex2 = vertex2;
	        _vertex3 = vertex3;
	        _vertex4 = vertex4;

	        _centerPoint  = null;
	        _xCoordinates = _yCoordinates = null;
        }

        /// <summary>
        /// Panel vertices object.
        /// </summary>
        /// <param name="vertices">The array of vertices, in any order.</param>
        /// <param name="geometryUnit">The <see cref="LengthUnit"/> of <paramref name="vertices"/>' coordinates.</param>
        public Vertices(Point3d[] vertices, LengthUnit geometryUnit = LengthUnit.Millimeter)
        {
			if (vertices.Length != 4)
				throw new NotImplementedException();

			Unit = geometryUnit;

            // Order points
            vertices = vertices.Order();

            // Set in necessary order (invert 3 and 4)
            _vertex1 = vertices[0];
            _vertex2 = vertices[1];
            _vertex3 = vertices[3];
            _vertex4 = vertices[2];

            _centerPoint  = null;
            _xCoordinates = _yCoordinates = null;
        }

        /// <summary>
        /// Get vertices as an array.
        /// </summary>
        public Point3d[] AsArray() => new [] {Vertex1, Vertex2, Vertex3, Vertex4};

        /// <summary>
        /// Change the <see cref="LengthUnit"/> of vertices' coordinates.
        /// </summary>
        /// <param name="unit">The <see cref="LengthUnit"/> to convert.</param>
        public void ChangeUnit(LengthUnit unit)
        {
	        if (Unit == unit)
		        return;

	        _vertex1 = _vertex1.Convert(Unit, unit);
	        _vertex2 = _vertex2.Convert(Unit, unit);
	        _vertex3 = _vertex3.Convert(Unit, unit);
	        _vertex4 = _vertex4.Convert(Unit, unit);

	        Unit = unit;
        }

        /// <summary>
        /// Calculate <see cref="Vertices"/> approximated center point.
        /// </summary>
        private Point3d CalculateCenterPoint()
        {
	        // Calculate the approximated center point
	        var Pt1 = Vertex1.MidPoint(Vertex3);
	        var Pt2 = Vertex2.MidPoint(Vertex4);

	        _centerPoint = Pt1.MidPoint(Pt2);

	        return _centerPoint.Value;
        }

		/// <summary>
        /// Get X and Y vertices coordinates;
        /// </summary>
        /// <returns></returns>
        private (double[] x, double[] y) VertexCoordinates()
        {
	        var vertices = AsArray();
	        double[] 
		        x = new double[4],
		        y = new double[4];

	        for (int i = 0; i < 4; i++)
	        {
		        x[i] = vertices[i].Convert(Unit).X;
		        y[i] = vertices[i].Convert(Unit).Y;
	        }

	        _xCoordinates = x;
	        _yCoordinates = y;

	        return (x, y);
        }

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
