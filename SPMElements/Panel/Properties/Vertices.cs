using System;
using System.Collections.Generic;
using System.Linq;
using OnPlaneComponents;
using UnitsNet.Units;

namespace SPM.Elements.PanelProperties
{
	/// <summary>
	///     Panel vertices struct.
	/// </summary>
	public struct Vertices : IEquatable<Vertices>, IComparable<Vertices>
	{
		/// <summary>
		///     Get <see cref="Vertices" /> approximated center point.
		/// </summary>
		public Point CenterPoint { get; }

		/// <summary>
		///     Get the <see cref="LengthUnit" /> of vertices' coordinates.
		/// </summary>
		public LengthUnit Unit
		{
			get => Vertex1.Unit;
			set => ChangeUnit(value);
		}

		/// <summary>
		///     Get vertex 1 (base left vertex).
		/// </summary>
		public Point Vertex1 { get ; private set; }

		/// <summary>
		///     Get vertex 2 (base right vertex).
		/// </summary>
		public Point Vertex2 { get ; private set; }

		/// <summary>
		///     Get vertex 3 (top right vertex).
		/// </summary>
		public Point Vertex3 { get ; private set; }

		/// <summary>
		///     Get vertex 4 (top left vertex).
		/// </summary>
		public Point Vertex4 { get ; private set; }

		/// <summary>
		///     Get X coordinates (in mm) of vertices as an array.
		/// </summary>
		public double[] XCoordinates => AsArray().Select(v => v.Convert(LengthUnit.Millimeter).X).ToArray();

		/// <summary>
		///     Get Y coordinates (in mm) of vertices as an array.
		/// </summary>
		public double[] YCoordinates => AsArray().Select(v => v.Convert(LengthUnit.Millimeter).Y).ToArray();

		/// <summary>
		///     Panel vertices object.
		/// </summary>
		/// <param name="vertex1">The base left vertex.</param>
		/// <param name="vertex2">The base right vertex.</param>
		/// <param name="vertex3">The upper right vertex.</param>
		/// <param name="vertex4">The upper left vertex.</param>
		public Vertices(Point vertex1, Point vertex2, Point vertex3, Point vertex4)
		{
			Vertex1 = vertex1;
			Vertex2 = vertex2;
			Vertex3 = vertex3;
			Vertex4 = vertex4;

			CenterPoint = CalculateCenterPoint(Vertex1, Vertex2, Vertex3, Vertex4);
		}

		/// <param name="vertices">The collection of the four <see cref="Point" /> vertices.</param>
		/// <inheritdoc cref="Vertices(Point, Point, Point, Point)" />
		public Vertices(IEnumerable<Point> vertices)
		{
			if (vertices.Count() != 4)
				throw new NotImplementedException();

			// Order points
			var verts = vertices.ToList();
			verts.Sort();

			// Set in necessary order (invert 3 and 4)
			this = new Vertices(verts[0], verts[1], verts[3], verts[2]);
		}

		/// <summary>
		///     Calculate <see cref="Vertices" /> approximated center point.
		/// </summary>
		/// <inheritdoc cref="Vertices(Point, Point, Point, Point)" />
		public static Point CalculateCenterPoint(Point vertex1, Point vertex2, Point vertex3, Point vertex4)
		{
			// Calculate the approximated center point
			var pt1 = vertex1.MidPoint(vertex3);
			var pt2 = vertex2.MidPoint(vertex4);

			return pt1.MidPoint(pt2);
		}

		/// <summary>
		///     Get vertices as an array.
		/// </summary>
		public Point[] AsArray() => new [] {Vertex1, Vertex2, Vertex3, Vertex4};

		/// <summary>
		///     Change the <see cref="LengthUnit" /> of vertices' coordinates.
		/// </summary>
		/// <param name="unit">The <see cref="LengthUnit" /> to convert.</param>
		public void ChangeUnit(LengthUnit unit)
		{
			if (Unit == unit)
				return;

			Vertex1 = Vertex1.Convert(unit);
			Vertex2 = Vertex2.Convert(unit);
			Vertex3 = Vertex3.Convert(unit);
			Vertex4 = Vertex4.Convert(unit);

			Unit = unit;
		}

		/// <summary>
		///     Convert this <see cref="Vertices" /> object to another <see cref="LengthUnit" />.
		/// </summary>
		/// <param name="unit">The desired <see cref="LengthUnit" />.</param>
		public Vertices Convert(LengthUnit unit) =>
			unit == Unit
				? this
				: new Vertices(Vertex1.Convert(unit), Vertex2.Convert(unit), Vertex3.Convert(unit),
					Vertex4.Convert(unit));

		public int CompareTo(Vertices other) => CenterPoint.CompareTo(other.CenterPoint);

		/// <summary>
		///     Returns true if all vertices are equal.
		/// </summary>
		/// <param name="other">The other <see cref="Vertices" /> to compare.</param>
		public bool Equals(Vertices other) => Vertex1 == other.Vertex1 && Vertex2 == other.Vertex2 &&
		                                      Vertex3 == other.Vertex3 && Vertex4 == other.Vertex4;

		public override bool Equals(object obj) => obj is Vertices other && Equals(other);

		public override int GetHashCode() => (int) AsArray().Sum(point => point.X * point.Y);

		public override string ToString() =>
			$"Vertex 1: ({Vertex1.X:0.00}, {Vertex1.Y:0.00})\n" +
			$"Vertex 2: ({Vertex2.X:0.00}, {Vertex2.Y:0.00})\n" +
			$"Vertex 3: ({Vertex3.X:0.00}, {Vertex3.Y:0.00})\n" +
			$"Vertex 4: ({Vertex4.X:0.00}, {Vertex4.Y:0.00})";

		/// <summary>
		///     Returns true if arguments are equal.
		/// </summary>
		public static bool operator == (Vertices left, Vertices right) => left.Equals(right);

		/// <summary>
		///     Returns true if arguments are different.
		/// </summary>
		public static bool operator != (Vertices left, Vertices right) => !left.Equals(right);
	}
}