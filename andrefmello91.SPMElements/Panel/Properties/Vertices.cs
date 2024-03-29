﻿using System;
using System.Collections.Generic;
using System.Linq;
using andrefmello91.Extensions;
using andrefmello91.OnPlaneComponents;
using UnitsNet;
using UnitsNet.Units;
#nullable disable

namespace andrefmello91.SPMElements.PanelProperties;

/// <summary>
///     Panel vertices struct.
/// </summary>
public struct Vertices : IUnitConvertible<LengthUnit>, IApproachable<Vertices, Length>, IEquatable<Vertices>, IComparable<Vertices>, ICloneable<Vertices>
{

	#region Properties

	/// <summary>
	///     Get <see cref="Vertices" /> approximated center point.
	/// </summary>
	public Point CenterPoint { get; }

	/// <summary>
	///     Returns true if this forms a rectangular geometry.
	/// </summary>
	public bool IsRectangular => Rectangular(this);

	/// <summary>
	///     Get vertex 1 (base left vertex).
	/// </summary>
	public Point Vertex1 { get; }

	/// <summary>
	///     Get vertex 2 (base right vertex).
	/// </summary>
	public Point Vertex2 { get; }

	/// <summary>
	///     Get vertex 3 (top right vertex).
	/// </summary>
	public Point Vertex3 { get; }

	/// <summary>
	///     Get vertex 4 (top left vertex).
	/// </summary>
	public Point Vertex4 { get; }

	/// <summary>
	///     Get X coordinates of vertices as an array.
	/// </summary>
	public Length[] XCoordinates => AsArray().Select(v => v.X).ToArray();

	/// <summary>
	///     Get Y coordinates of vertices as an array.
	/// </summary>
	public Length[] YCoordinates => AsArray().Select(v => v.Y).ToArray();

	/// <summary>
	///     Get the <see cref="LengthUnit" /> of vertices' coordinates.
	/// </summary>
	public LengthUnit Unit
	{
		get => Vertex1.Unit;
		set => ChangeUnit(value);
	}

	#endregion

	#region Constructors

	/// <summary>
	///     Create a panel vertices object.
	/// </summary>
	/// <param name="vertex1">The base left vertex.</param>
	/// <param name="vertex2">The base right vertex.</param>
	/// <param name="vertex3">The upper right vertex.</param>
	/// <param name="vertex4">The upper left vertex.</param>
	public Vertices(Point vertex1, Point vertex2, Point vertex3, Point vertex4)
	{
		Vertex1 = vertex1;
		Vertex2 = vertex2.Convert(vertex1.Unit);
		Vertex3 = vertex3.Convert(vertex1.Unit);
		Vertex4 = vertex4.Convert(vertex1.Unit);

		CenterPoint = CalculateCenterPoint(Vertex1, Vertex2, Vertex3, Vertex4);
	}

	#endregion

	#region Methods

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
	///     Create a panel vertices object from a collection of points.
	/// </summary>
	/// <param name="vertices">The collection of the four <see cref="Point" /> vertices, in any order.</param>
	/// <exception cref="ArgumentException">If <paramref name="vertices" /> doesn't contain 4 points.</exception>
	public static Vertices From(IEnumerable<Point> vertices)
	{
		if (vertices.Count() != 4)
			throw new ArgumentException("Vertices must contain 4 points.", nameof(vertices));

		// Order points
		var verts = vertices.OrderBy(p => p.Y).ThenBy(p => p.X).ToList();

		// Get correct order
		var (v1, v2) = verts[1].X > verts[0].X
			? (verts[0], verts[1])
			: (verts[1], verts[0]);

		var (v4, v3) = verts[3].X > verts[2].X
			? (verts[2], verts[3])
			: (verts[3], verts[2]);

		return
			new Vertices(v1, v2, v3, v4);
	}

	/// <summary>
	///     Returns true if <paramref name="vertices" /> form a rectangular geometry.
	/// </summary>
	/// <param name="vertices">The <see cref="Vertices" /> object to check.</param>
	public static bool Rectangular(Vertices vertices) =>
		vertices.Vertex1.Y.Approx(vertices.Vertex2.Y, Point.Tolerance) &&
		vertices.Vertex4.Y.Approx(vertices.Vertex3.Y, Point.Tolerance) &&
		vertices.Vertex1.X.Approx(vertices.Vertex4.X, Point.Tolerance) &&
		vertices.Vertex2.X.Approx(vertices.Vertex3.X, Point.Tolerance);

	/// <summary>
	///     Get vertices as an array.
	/// </summary>
	public Point[] AsArray() => new[] { Vertex1, Vertex2, Vertex3, Vertex4 };

	/// <summary>
	///     Convert this <see cref="Vertices" /> object to another <see cref="LengthUnit" />.
	/// </summary>
	/// <param name="unit">The desired <see cref="LengthUnit" />.</param>
	public Vertices Convert(LengthUnit unit) =>
		unit == Unit
			? Clone()
			: new Vertices(Vertex1.Convert(unit), Vertex2.Convert(unit), Vertex3.Convert(unit), Vertex4.Convert(unit));

	/// <summary>
	///     Divide a <see cref="Vertices" /> object into new ones.
	/// </summary>
	/// <remarks>
	///     This must be rectangular, otherwise an empty collection is returned.
	/// </remarks>
	/// <param name="rows">The required number of rows.</param>
	/// <param name="columns">The required number of columns.</param>
	public IEnumerable<Vertices> Divide(int rows, int columns)
	{
		if (!IsRectangular)
			yield break;

		// Get distances
		var dx = (Vertex2.X - Vertex1.X) / columns;
		var dy = (Vertex4.Y - Vertex1.Y) / rows;

		for (var r = 0; r < rows; r++)
		{
			// Get initial vertex for this row
			var v1 = new Point(Vertex1.X, Vertex1.Y + r * dy);

			for (var c = 0; c < columns; c++)
			{
				// Get other vertices
				var v2 = new Point(v1.X + dx, v1.Y);
				var v3 = new Point(v1.X + dx, v1.Y + dy);
				var v4 = new Point(v1.X, v1.Y + dy);

				// Return
				yield return new Vertices(v1, v2, v3, v4);

				// Set initial vertex for next column
				v1 = v2;
			}
		}
	}

	/// <inheritdoc />
	public override bool Equals(object obj) => obj is Vertices other && Equals(other);

	/// <inheritdoc />
	public override int GetHashCode() => AsArray().Sum(point => point.GetHashCode());

	/// <inheritdoc />
	public override string ToString() =>
		$"Vertex 1:\t({Vertex1.X.Value:0.00},\t{Vertex1.Y.Value:0.00})\n" +
		$"Vertex 2:\t({Vertex2.X.Value:0.00},\t{Vertex2.Y.Value:0.00})\n" +
		$"Vertex 3:\t({Vertex3.X.Value:0.00},\t{Vertex3.Y.Value:0.00})\n" +
		$"Vertex 4:\t({Vertex4.X.Value:0.00},\t{Vertex4.Y.Value:0.00})\n" +
		$"Coordinates in {Unit}";

	/// <inheritdoc />
	public bool Approaches(Vertices other, Length tolerance) =>
		Vertex1.Approaches(other.Vertex1, tolerance) && Vertex2.Approaches(other.Vertex2, tolerance) &&
		Vertex3.Approaches(other.Vertex3, tolerance) && Vertex4.Approaches(other.Vertex4, tolerance);

	/// <inheritdoc />
	public Vertices Clone() => From(AsArray());

	/// <inheritdoc />
	public int CompareTo(Vertices other) => CenterPoint.CompareTo(other.CenterPoint);

	/// <summary>
	///     Returns true if all vertices are equal.
	/// </summary>
	/// <param name="other">The other <see cref="Vertices" /> to compare.</param>
	public bool Equals(Vertices other) => Approaches(other, Point.Tolerance);

	/// <summary>
	///     Change the <see cref="LengthUnit" /> of vertices' coordinates.
	/// </summary>
	/// <param name="unit">The <see cref="LengthUnit" /> to convert.</param>
	public void ChangeUnit(LengthUnit unit)
	{
		if (Unit == unit)
			return;

		Vertex1.ChangeUnit(unit);
		Vertex2.ChangeUnit(unit);
		Vertex3.ChangeUnit(unit);
		Vertex4.ChangeUnit(unit);

		Unit = unit;
	}

	IUnitConvertible<LengthUnit> IUnitConvertible<LengthUnit>.Convert(LengthUnit unit) => Convert(unit);

	#endregion

	#region Operators

	/// <summary>
	///     Returns true if arguments are equal.
	/// </summary>
	public static bool operator ==(Vertices left, Vertices right) => left.Equals(right);

	/// <summary>
	///     Returns true if arguments are different.
	/// </summary>
	public static bool operator !=(Vertices left, Vertices right) => !left.Equals(right);

	#endregion

}