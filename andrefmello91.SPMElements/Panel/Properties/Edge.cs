using System;
using andrefmello91.OnPlaneComponents;
using Extensions;
using UnitsNet;
using UnitsNet.Units;
#nullable disable

namespace andrefmello91.SPMElements.PanelProperties
{
	/// <summary>
	///     Panel edge struct.
	/// </summary>
	public struct Edge : IUnitConvertible<Edge, LengthUnit>, IApproachable<Edge, Length>, IEquatable<Edge>, IComparable<Edge>
	{
		private Length _stringerDimension;

		#region Properties

		/// <summary>
		///     Get the <see cref="LengthUnit" /> that this was constructed with.
		/// </summary>
		public LengthUnit Unit
		{
			get => InitialVertex.Unit;
			set => ChangeUnit(value);
		}

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
		///     Get length of this edge.
		/// </summary>
		public Length Length { get; private set; }

		/// <summary>
		///     Get the stringer dimension of this edge, in mm.
		/// </summary>
		public Length StringerDimension
		{
			get => _stringerDimension;
			set => _stringerDimension = value.ToUnit(Unit);
		}

		#endregion

		#region Constructors

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
			Length             = initialVertex.GetDistance(finalVertex);
			Angle              = initialVertex.GetAngle(finalVertex);
			_stringerDimension = Length.Zero;
		}

		#endregion

		#region Methods

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

			Length             = Length.ToUnit(unit);
			_stringerDimension = _stringerDimension.ToUnit(unit);
		}

		/// <inheritdoc />
		public Edge Convert(LengthUnit unit) => new(InitialVertex.Convert(unit), FinalVertex.Convert(unit));

		/// <inheritdoc />
		public bool Approaches(Edge other, Length tolerance) =>
			InitialVertex.Approaches(other.InitialVertex, tolerance) && FinalVertex.Approaches(other.FinalVertex, tolerance) ||
			InitialVertex.Approaches(other.FinalVertex, tolerance) && FinalVertex.Approaches(other.InitialVertex, tolerance);

		/// <inheritdoc />
		public int CompareTo(Edge other) => CenterPoint.CompareTo(other.CenterPoint);

		/// <summary>
		///     Returns true if edges are equal.
		/// </summary>
		/// <param name="other">The other <see cref="Edge" /> object.</param>
		public bool Equals(Edge other) => InitialVertex == other.InitialVertex && FinalVertex == other.FinalVertex;

		/// <inheritdoc />
		public override bool Equals(object obj) => obj is Edge other && Equals(other);

		/// <inheritdoc />
		public override int GetHashCode() => InitialVertex.GetHashCode() * FinalVertex.GetHashCode();

		/// <inheritdoc />
		public override string ToString() =>
			$"Initial vertex: ({InitialVertex.X:0.00}, {InitialVertex.Y:0.00})\n" +
			$"Final vertex: ({FinalVertex.X:0.00}, {FinalVertex.Y:0.00})\n" +
			$"Lenght = {Length}\n" +
			$"Angle = {Angle:0.00} rad";

		#endregion

		#region Operators

		/// <summary>
		///     Returns true if arguments are equal.
		/// </summary>
		public static bool operator ==(Edge left, Edge right) => left.Equals(right);

		/// <summary>
		///     Returns true if arguments are different.
		/// </summary>
		public static bool operator !=(Edge left, Edge right) => !left.Equals(right);

		#endregion
	}
}