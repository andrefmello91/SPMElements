using System;
using Autodesk.AutoCAD.Geometry;
using Extensions;
using MathNet.Numerics;
using UnitsNet;
using UnitsNet.Units;

namespace SPMElements.PanelProperties
{
	/// <summary>
    /// Panel geometry struct.
    /// </summary>
    public struct PanelGeometry : IEquatable<PanelGeometry>
    {
		// Auxiliary fields
		private Length _width;
		private double? _a, _b, _c, _d;

		/// <summary>
        /// Get the <see cref="LengthUnit"/> that this was constructed with.
        /// </summary>
        public LengthUnit Unit => _width.Unit;

        /// <summary>
        /// Get vertices of panel.
        /// </summary>
        public Vertices Vertices { get; }

        /// <summary>
        /// Get <see cref="Edge"/> 1 (base edge).
        /// </summary>
        public Edge Edge1 { get; }

        /// <summary>
        /// Get <see cref="Edge"/> 2 (right edge).
        /// </summary>
        public Edge Edge2 { get; }

        /// <summary>
        /// Get <see cref="Edge"/> 3 (top edge).
        /// </summary>
        public Edge Edge3 { get; }

        /// <summary>
        /// Get <see cref="Edge"/> 4 (left edge).
        /// </summary>
        public Edge Edge4 { get; }

		/// <summary>
        /// Get panel width, in mm.
        /// </summary>
        public double Width => _width.Millimeters;

		/// <summary>
        /// Returns true if this geometry is rectangular.
        /// </summary>
		public bool Rectangular
		{
			get
			{
				// Calculate the angles between the edges
				var ang2 = Edge2.Angle - Edge1.Angle;
				var ang4 = Edge4.Angle - Edge3.Angle;

				if (ang2 == Constants.PiOver2 && ang4 == Constants.PiOver2)
					return true;

				return false;
			}
		}

		/// <summary>
		/// Get panel dimensions (a, b, c, d), in mm.
		/// </summary>
		public (double a, double b, double c, double d) Dimensions
		{
			get
			{
				if (!_a.HasValue)
					CalculateDimensions();

				return
					(_a.Value, _b.Value, _c.Value, _d.Value);
			}
		}

        /// <summary>
        /// Panel geometry constructor.
        /// </summary>
        /// <param name="vertex1">The base left vertex.</param>
        /// <param name="vertex2">The base right vertex.</param>
        /// <param name="vertex3">The upper right vertex.</param>
        /// <param name="vertex4">The upper left vertex.</param>
        /// <param name="width">Panel width, in <paramref name="geometryUnit"/>.</param>
        /// <param name="geometryUnit">The <see cref="LengthUnit"/> of <paramref name="width"/> and vertices' coordinates.</param>
        public PanelGeometry(Point3d vertex1, Point3d vertex2, Point3d vertex3, Point3d vertex4, double width, LengthUnit geometryUnit = LengthUnit.Millimeter) 
			: this (new Vertices(vertex1, vertex2, vertex3, vertex4), width, geometryUnit)
		{
		}

        /// <summary>
        /// Panel geometry constructor.
        /// </summary>
        /// <param name="vertices">The array of vertices, in any order.</param>
        /// <param name="width">Panel width, in <paramref name="geometryUnit"/>.</param>
        /// <param name="geometryUnit">The <see cref="LengthUnit"/> of <paramref name="width"/> and <paramref name="vertices"/>' coordinates.</param>
        public PanelGeometry(Point3d[] vertices, double width, LengthUnit geometryUnit = LengthUnit.Millimeter) 
			: this (new Vertices(vertices), width, geometryUnit)
		{
		}

        /// <summary>
        /// Panel geometry constructor.
        /// </summary>
        /// <param name="vertices">Panel <see cref="Vertices"/> object.</param>
        /// <param name="width">Panel width, in <paramref name="geometryUnit"/>.</param>
        /// <param name="geometryUnit">The <see cref="LengthUnit"/> of <paramref name="width"/> and <paramref name="vertices"/>' coordinates.</param>
        public PanelGeometry(Vertices vertices, double width, LengthUnit geometryUnit = LengthUnit.Millimeter)
		{
			Vertices = vertices;
			_width   = Length.From(width, geometryUnit);

			// Get edges
			Edge1 = new Edge(vertices.Vertex1, vertices.Vertex2, geometryUnit);
			Edge2 = new Edge(vertices.Vertex2, vertices.Vertex3, geometryUnit);
			Edge3 = new Edge(vertices.Vertex3, vertices.Vertex4, geometryUnit);
			Edge4 = new Edge(vertices.Vertex4, vertices.Vertex1, geometryUnit);

			_a = _b = _c = _d = null;
		}

		/// <summary>
		/// Change the <see cref="LengthUnit"/> of this.
		/// </summary>
		/// <param name="unit">The <see cref="LengthUnit"/> to convert.</param>
		public void ChangeUnit(LengthUnit unit)
		{
			if (Unit != unit)
				_width.ToUnit(unit);
		}

		/// <summary>
        /// Calculate panel dimensions (a, b, c, d).
        /// </summary>
		private void CalculateDimensions()
		{
			var x = Vertices.XCoordinates;
			var y = Vertices.YCoordinates;

			// Calculate the necessary dimensions of the panel
			_a = 0.5 * (x[1] + x[2] - x[0] - x[3]);
			_b = 0.5 * (y[2] + y[3] - y[0] - y[1]);
			_c = 0.5 * (x[2] + x[3] - x[0] - x[1]);
			_d = 0.5 * (y[1] + y[2] - y[0] - y[3]);
		}

        /// <summary>
        /// Returns true if all <see cref="Vertices"/> are equal.
        /// </summary>
        /// <param name="other">The other <see cref="PanelGeometry"/> to compare.</param>
        public bool Equals(PanelGeometry other) => Vertices == other.Vertices;

        public override bool Equals(object obj) => obj is PanelGeometry other && Equals(other);

        public override int GetHashCode() => Vertices.GetHashCode();

        public override string ToString()
        {
	        return
		        Vertices + "\n" +
		        $"Width = {_width}";
        }

        /// <summary>
        /// Returns true if arguments are equal.
        /// </summary>
        public static bool operator == (PanelGeometry left, PanelGeometry right) => left.Equals(right);

        /// <summary>
        /// Returns true if arguments are different.
        /// </summary>
        public static bool operator != (PanelGeometry left, PanelGeometry right) => !left.Equals(right);

    }
}
