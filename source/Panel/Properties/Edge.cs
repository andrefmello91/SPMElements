using System;
using Extensions;
using OnPlaneComponents;
using UnitsNet;
using UnitsNet.Units;

namespace SPM.Elements.PanelProperties
{
	/// <summary>
	///     Panel edge struct.
	/// </summary>
	public struct Edge : IUnitConvertible<Edge, LengthUnit>, IEquatable<Edge>, IComparable<Edge>
	{
		// Auxiliary fields
		private Length _length, _stringerDimension;

		/// <summary>
		///     Get angle related to horizontal axis, in radians.
		/// </summary>
		public double Angle { get; }

		/// <summary>
		///     Get center point of this <see cref="Edge" />.
		/// </summary>
		public Point CenterPoint { get; }

		/// <summary>
		///     Get the final vertex of this <see cref="Edge" />.
		/// </summary>
		public Point FinalVertex { get; }

		/// <summary>
		///     Get the initial vertex of this <see cref="Edge" />.
		/// </summary>
		public Point InitialVertex { get; }

		/// <summary>
		///     Get length, in mm.
		/// </summary>
		public double Length => _length.Millimeters;

		/// <summary>
		///     Get the stringer dimension of this edge, in mm.
		/// </summary>
		public double StringerDimension => _stringerDimension.Millimeters;

		/// <summary>
		///     Get the <see cref="LengthUnit" /> that this was constructed with.
		/// </summary>
		public LengthUnit Unit
		{
			get => InitialVertex.Unit;
			set => ChangeUnit(value);
		}

		/// <summary>
		///     Panel edge constructor.
		/// </summary>
		/// <param name="initialVertex">The initial vertex.</param>
		/// <param name="finalVertex">The final vertex.</param>
		public Edge(Point initialVertex, Point finalVertex)
		{
			InitialVertex      = initialVertex;
			FinalVertex        = finalVertex.Convert(initialVertex.Unit);
			CenterPoint        = initialVertex.MidPoint(finalVertex);
			_length            = UnitsNet.Length.From(initialVertex.GetDistance(finalVertex), initialVertex.Unit);
			Angle              = initialVertex.GetAngle(finalVertex);
			_stringerDimension = UnitsNet.Length.Zero;
		}

		/// <summary>
		///     Change the <see cref="LengthUnit" /> of this.
		/// </summary>
		/// <param name="unit">The <see cref="LengthUnit" /> to convert.</param>
		public void ChangeUnit(LengthUnit unit)
		{
			if (Unit == unit)
				return;

			InitialVertex.ChangeUnit(unit);
			CenterPoint.ChangeUnit(unit);
			FinalVertex.ChangeUnit(unit);

			_length            = _length.ToUnit(unit);
			_stringerDimension = _stringerDimension.ToUnit(unit);
		}

		public Edge Convert(LengthUnit unit) => new Edge(InitialVertex.Convert(unit), FinalVertex.Convert(unit));

		public Edge Copy() => throw new NotImplementedException();

		/// <summary>
		///     Set stringer dimension in this edge.
		/// </summary>
		/// <param name="height">The height of the <seealso cref="Stringer" />, in <paramref name="unit" /> considered.</param>
		/// <param name="unit">The <see cref="LengthUnit" /> of <paramref name="height" />.</param>
		public void SetStringerDimension(double height, LengthUnit unit = LengthUnit.Millimeter) =>
			SetStringerDimension(UnitsNet.Length.From(height, unit));

		/// <param name="height">The height of the <seealso cref="Stringer" />.</param>
		/// <inheritdoc cref="SetStringerDimension(double, LengthUnit)" />
		public void SetStringerDimension(Length height) => _stringerDimension = height.ToUnit(Unit);

		public int CompareTo(Edge other) => CenterPoint.CompareTo(other.CenterPoint);

		/// <summary>
		///     Returns true if edges are equal.
		/// </summary>
		/// <param name="other">The other <see cref="Edge" /> object.</param>
		public bool Equals(Edge other) => InitialVertex == other.InitialVertex && FinalVertex == other.FinalVertex;

		public override bool Equals(object obj) => obj is Edge other && Equals(other);

		public override int GetHashCode() => (int) (InitialVertex.X * FinalVertex.X + InitialVertex.Y * FinalVertex.Y);

		public override string ToString() =>
			$"Initial vertex: ({InitialVertex.X:0.00}, {InitialVertex.Y:0.00})\n" +
			$"Final vertex: ({FinalVertex.X:0.00}, {FinalVertex.Y:0.00})\n" +
			$"Lenght = {_length}\n" +
			$"Angle = {Angle:0.00} rad";

		/// <summary>
		///     Returns true if arguments are equal.
		/// </summary>
		public static bool operator == (Edge left, Edge right) => left.Equals(right);

		/// <summary>
		///     Returns true if arguments are different.
		/// </summary>
		public static bool operator != (Edge left, Edge right) => !left.Equals(right);
	}
}