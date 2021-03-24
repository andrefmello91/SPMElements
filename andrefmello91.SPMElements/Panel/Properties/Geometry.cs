using System;
using System.Collections.Generic;
using System.Linq;
using Extensions;
using UnitsNet;
using UnitsNet.Units;

#nullable disable

namespace andrefmello91.SPMElements.PanelProperties
{
	/// <summary>
	///     Panel geometry struct.
	/// </summary>
	public struct PanelGeometry : IUnitConvertible<PanelGeometry, LengthUnit>, IApproachable<PanelGeometry, Length>, IEquatable<PanelGeometry>, IComparable<PanelGeometry>, ICloneable<PanelGeometry>
	{
		#region Fields

		private Length _width;

		#endregion

		#region Properties

		/// <summary>
		///     Get the <see cref="LengthUnit" /> that this was constructed with.
		/// </summary>
		public LengthUnit Unit
		{
			get => Vertices.Unit;
			set => ChangeUnit(value);
		}

		/// <summary>
		///     Get panel dimensions (a, b, c, d).
		/// </summary>
		public (Length a, Length b, Length c, Length d) Dimensions { get; private set; }

		/// <summary>
		///     Get <see cref="Edge" /> 1 (base edge).
		/// </summary>
		public Edge Edge1 { get; }

		/// <summary>
		///     Get <see cref="Edge" /> 2 (right edge).
		/// </summary>
		public Edge Edge2 { get; }

		/// <summary>
		///     Get <see cref="Edge" /> 3 (top edge).
		/// </summary>
		public Edge Edge3 { get; }

		/// <summary>
		///     Get <see cref="Edge" /> 4 (left edge).
		/// </summary>
		public Edge Edge4 { get; }

		/// <summary>
		///     Get edges' lengths as an array.
		/// </summary>
		public Length[] EdgeLengths => Edges.Select(e => e.Length).ToArray();

		/// <summary>
		///     Get edges as an array.
		/// </summary>
		public Edge[] Edges => new [] { Edge1, Edge2, Edge3, Edge4 };

		/// <summary>
		///     Get grip positions as an array.
		/// </summary>
		public Point[] GripPositions => Edges.Select(e => e.CenterPoint).ToArray();

		/// <summary>
		///     Returns true if this geometry is rectangular.
		/// </summary>
		public bool IsRectangular => Vertices.IsRectangular;

		/// <summary>
		///     Get edges' stringer dimensions as an array.
		///     <para>See: <see cref="Edge.SetStringerDimension(Length)" /></para>
		/// </summary>
		public Length[] StringerDimensions => Edges.Select(e => e.StringerDimension).ToArray();

		/// <summary>
		///     Get vertices of panel.
		/// </summary>
		public Vertices Vertices { get; }

		/// <summary>
		///     Get panel width.
		/// </summary>
		public Length Width
		{
			get => _width;
			set => _width = value.ToUnit(Unit);
		}

		#endregion

		#region Constructors

		/// <summary>
		///     Panel geometry constructor.
		/// </summary>
		/// <param name="vertices">The collection of vertices, in any order.</param>
		/// <param name="width">Panel width.</param>
		/// <param name="unit">
		///     The <see cref="LengthUnit" /> of <paramref name="width" /> and <paramref name="vertices" />'
		///     coordinates.
		/// </param>
		public PanelGeometry(IEnumerable<Point> vertices, double width, LengthUnit unit = LengthUnit.Millimeter)
			: this (new Vertices(vertices), width, unit)
		{
		}

		/// <inheritdoc cref="PanelGeometry(IEnumerable{Point}, double, LengthUnit)" />
		public PanelGeometry(IEnumerable<Point> vertices, Length width)
			: this (new Vertices(vertices), width)
		{
		}

		/// <param name="vertices">Panel <see cref="PanelProperties.Vertices" /> object.</param>
		/// <inheritdoc cref="PanelGeometry(IEnumerable{Point}, double, LengthUnit)" />
		public PanelGeometry(Vertices vertices, double width, LengthUnit unit = LengthUnit.Millimeter)
			: this (vertices, Length.From(width, unit))
		{
		}

		/// <inheritdoc cref="PanelGeometry" />
		public PanelGeometry(Vertices vertices, Length width)
		{
			Vertices = vertices;
			_width   = width.ToUnit(vertices.Unit);

			// Get edges
			Edge1 = new Edge(vertices.Vertex1, vertices.Vertex2);
			Edge2 = new Edge(vertices.Vertex2, vertices.Vertex3);
			Edge3 = new Edge(vertices.Vertex3, vertices.Vertex4);
			Edge4 = new Edge(vertices.Vertex4, vertices.Vertex1);

			Dimensions = CalculateDimensions(Vertices);
		}

		#endregion

		#region  Methods

		/// <summary>
		///     Calculate panel dimensions (a, b, c, d).
		/// </summary>
		public static (Length a, Length b, Length c, Length d) CalculateDimensions(Vertices vertices)
		{
			var x = vertices.XCoordinates;
			var y = vertices.YCoordinates;

			// Calculate the necessary dimensions of the panel
			Length
				a = 0.5 * (x[1] + x[2] - x[0] - x[3]),
				b = 0.5 * (y[2] + y[3] - y[0] - y[1]),
				c = 0.5 * (x[2] + x[3] - x[0] - x[1]),
				d = 0.5 * (y[1] + y[2] - y[0] - y[3]);

			return (a, b, c, d);
		}

		/// <summary>
		///     Divide a <see cref="PanelGeometry" /> object into new ones.
		/// </summary>
		/// <param name="geometry">The <see cref="PanelGeometry" /> object to divide.</param>
		/// <inheritdoc cref="Vertices.Divide(Vertices, int, int)" />
		public static IEnumerable<PanelGeometry> Divide(PanelGeometry geometry, int rows, int columns) => geometry.Vertices.Divide(rows, columns).Select(v => new PanelGeometry(v, geometry.Width));

		/// <inheritdoc cref="Vertices.Divide(int, int)" />
		public IEnumerable<PanelGeometry> Divide(int rows, int columns) => Divide(this, rows, columns);

		/// <summary>
		///     Change the <see cref="LengthUnit" /> of this.
		/// </summary>
		/// <param name="unit">The <see cref="LengthUnit" /> to convert.</param>
		public void ChangeUnit(LengthUnit unit)
		{
			if (Unit == unit)
				return;

			_width = _width.ToUnit(unit);

			Vertices.ChangeUnit(unit);

			Edge1.ChangeUnit(unit);
			Edge2.ChangeUnit(unit);
			Edge3.ChangeUnit(unit);
			Edge4.ChangeUnit(unit);

			Dimensions = CalculateDimensions(Vertices);
		}

		/// <summary>
		///     Return <see cref="Dimensions" /> in <see cref="LengthUnit.Millimeter" />.
		/// </summary>
		public (double a, double b, double c, double d) DimensionsInMillimeters() => (Dimensions.a.Millimeters, Dimensions.b.Millimeters, Dimensions.c.Millimeters, Dimensions.d.Millimeters);

		public PanelGeometry Convert(LengthUnit unit) => new PanelGeometry(Vertices.Convert(unit), Width.ToUnit(unit));

		public bool Approaches(PanelGeometry other, Length tolerance) => Vertices.Approaches(other.Vertices, tolerance);

		public PanelGeometry Clone() => new PanelGeometry(Vertices.Clone(), Width);

		public int CompareTo(PanelGeometry other) => Vertices.CompareTo(other.Vertices);

		/// <summary>
		///     Returns true if all <see cref="Vertices" /> are equal.
		/// </summary>
		/// <param name="other">The other <see cref="PanelGeometry" /> to compare.</param>
		public bool Equals(PanelGeometry other) => Approaches(other, Tolerance);

		public override string ToString() =>
			$"{Vertices}\n" +
			$"Width = {Width}";

		public override bool Equals(object obj) => obj is PanelGeometry other && Equals(other);

		public override int GetHashCode() => Vertices.GetHashCode();

		#endregion

		#region Operators

		/// <summary>
		///     Returns true if arguments are equal.
		/// </summary>
		public static bool operator == (PanelGeometry left, PanelGeometry right) => left.Equals(right);

		/// <summary>
		///     Returns true if arguments are different.
		/// </summary>
		public static bool operator != (PanelGeometry left, PanelGeometry right) => !left.Equals(right);

		#endregion
	}
}