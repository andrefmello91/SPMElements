using System;
using System.Collections.Generic;
using System.Linq;
using Extensions.Number;
using MathNet.Numerics;
using OnPlaneComponents;
using UnitsNet;
using UnitsNet.Units;

namespace SPM.Elements.PanelProperties
{
	/// <summary>
	///     Panel geometry struct.
	/// </summary>
	public struct PanelGeometry : IEquatable<PanelGeometry>, IComparable<PanelGeometry>
	{
		#region Fields

		private (double a, double b, double c, double d)? _dimensions;

		// Auxiliary fields
		private Length _width;

		#endregion

		#region Properties

		/// <summary>
		///     Get panel dimensions (a, b, c, d), in mm.
		/// </summary>
		public (double a, double b, double c, double d) Dimensions => _dimensions ?? CalculateDimensions();

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
		public double[] EdgeLengths => Edges.Select(e => e.Length).ToArray();

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
		public bool Rectangular
		{
			get
			{
				// Calculate the angles between the edges
				double[] angles =
				{
					Edge2.Angle - Edge1.Angle,
					Edge4.Angle - Edge3.Angle
				};

				return
					angles.All(angle =>
						angle.Approx(Constants.PiOver2, 1E-3) || angle.Approx(Constants.Pi3Over2, 1E-3));
			}
		}

		/// <summary>
		///     Get edges' stringer dimensions as an array.
		///     <para>See: <see cref="Edge.SetStringerDimension(Length)" /></para>
		/// </summary>
		public double[] StringerDimensions => Edges.Select(e => e.StringerDimension).ToArray();

		/// <summary>
		///     Get the <see cref="LengthUnit" /> that this was constructed with.
		/// </summary>
		public LengthUnit Unit => _width.Unit;

		/// <summary>
		///     Get vertices of panel.
		/// </summary>
		public Vertices Vertices { get; }

		/// <summary>
		///     Get panel width, in mm.
		/// </summary>
		public double Width => _width.Millimeters;

		#endregion

		#region Constructors

		/// <summary>
		///     Panel geometry constructor.
		/// </summary>
		/// <param name="vertices">The collection of vertices, in any order.</param>
		/// <param name="width">Panel width, in <paramref name="unit" />.</param>
		/// <param name="unit">
		///     The <see cref="LengthUnit" /> of <paramref name="width" /> and <paramref name="vertices" />'
		///     coordinates.
		/// </param>
		public PanelGeometry(IEnumerable<Point> vertices, double width, LengthUnit unit = LengthUnit.Millimeter)
			: this (new Vertices(vertices), width, unit)
		{
		}

		/// <param name="width">Panel width.</param>
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

		/// <param name="width">Panel width.</param>
		/// <inheritdoc cref="PanelGeometry(Vertices, double, LengthUnit)" />
		public PanelGeometry(Vertices vertices, Length width)
		{
			Vertices = vertices;
			_width   = width.ToUnit(vertices.Unit);

			// Get edges
			Edge1 = new Edge(vertices.Vertex1, vertices.Vertex2);
			Edge2 = new Edge(vertices.Vertex2, vertices.Vertex3);
			Edge3 = new Edge(vertices.Vertex3, vertices.Vertex4);
			Edge4 = new Edge(vertices.Vertex4, vertices.Vertex1);

			_dimensions = null;
		}

		#endregion

		#region  Methods

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

			foreach (var edge in Edges)
				edge.ChangeUnit(unit);
		}

		/// <summary>
		///     Calculate panel dimensions (a, b, c, d).
		/// </summary>
		private (double a, double b, double c, double d) CalculateDimensions()
		{
			var x = Vertices.XCoordinates;
			var y = Vertices.YCoordinates;

			// Calculate the necessary dimensions of the panel
			double
				a = 0.5 * (x[1] + x[2] - x[0] - x[3]),
				b = 0.5 * (y[2] + y[3] - y[0] - y[1]),
				c = 0.5 * (x[2] + x[3] - x[0] - x[1]),
				d = 0.5 * (y[1] + y[2] - y[0] - y[3]);

			_dimensions = (a, b, c, d);

			return _dimensions.Value;
		}

		public int CompareTo(PanelGeometry other) => Vertices.CompareTo(other.Vertices);

		/// <summary>
		///     Returns true if all <see cref="Vertices" /> are equal.
		/// </summary>
		/// <param name="other">The other <see cref="PanelGeometry" /> to compare.</param>
		public bool Equals(PanelGeometry other) => Vertices == other.Vertices;

		public override string ToString() =>
			$"{Vertices}\n" +
			$"Width = {_width}";

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