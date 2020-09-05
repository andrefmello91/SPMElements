using System;
using Autodesk.AutoCAD.Geometry;
using MathNet.Numerics;
using UnitsNet;
using UnitsNet.Units;

namespace SPMElements
{
	/// <summary>
	/// Stringer geometry struct.
	/// </summary>
	public struct StringerGeometry : IEquatable<StringerGeometry>
	{
		// Auxiliary fields
		private Length _length, _width, _height;

		/// <summary>
		/// Get the <see cref="LengthUnit"/> that this was constructed with.
		/// </summary>
		public LengthUnit Unit => _length.Unit;

		/// <summary>
		/// Get the initial <see cref="Node"/> of <see cref="Stringer"/>.
		/// </summary>
		public Node InitialNode { get; }

		/// <summary>
		/// Get the center <see cref="Node"/> of <see cref="Stringer"/>.
		/// </summary>
		public Node CenterNode { get; }

		/// <summary>
		/// Get the final <see cref="Node"/> of <see cref="Stringer"/>.
		/// </summary>
		public Node FinalNode { get; }

		/// <summary>
		/// The stringer length, in mm.
		/// </summary>
		public double Length => _length.Millimeters;

		/// <summary>
		/// The stringer angle, in radians.
		/// </summary>
		public double Angle { get; }

		/// <summary>
		/// The stringer width, in mm.
		/// </summary>
		public double Width => _width.Millimeters;

		/// <summary>
		/// The stringer height, in mm.
		/// </summary>
		public double Height => _height.Millimeters;

		/// <summary>
		/// Get cross-section are of this, in mm2.
		/// </summary>
		public double Area => Width * Height;

		/// <summary>
		/// Get direction cosines of <see cref="Angle"/>.
		/// </summary>
		public (double cos, double sin) DirectionCosines =>
			(Trig.Cos(Angle).CoerceZero(1E-6), Trig.Sin(Angle).CoerceZero(1E-6));

		/// <summary>
		/// Get the connected <see cref="Point3d"/> of this.
		/// </summary>
		public Point3d[] ConnectedPoints => new [] {InitialNode.Position, CenterNode.Position, FinalNode.Position};

		/// <summary>
		/// Stringer geometry object.
		/// </summary>
		/// <param name="initialNode">The initial <see cref="Node"/> of the <see cref="Stringer"/>.</param>
		/// <param name="centerNode">The center <see cref="Node"/> of the <see cref="Stringer"/>.</param>
		/// <param name="finalNode">The final <see cref="Node"/> of the <see cref="Stringer"/>.</param>
		/// <param name="width">The stringer width.</param>
		/// <param name="height">The stringer height.</param>
		/// <param name="geometryUnit">The <see cref="LengthUnit"/> of <paramref name="width"/>, <paramref name="height"/> and nodes' coordinates.<para>Default: <seealso cref="LengthUnit.Millimeter"/>.</para></param>
		public StringerGeometry(Node initialNode, Node centerNode, Node finalNode, double width, double height, LengthUnit geometryUnit = LengthUnit.Millimeter)
		{
			InitialNode = initialNode;
			CenterNode  = centerNode;
			FinalNode   = finalNode;

			// Calculate length and angle
			_length = UnitsNet.Length.From(initialNode.GetDistance(finalNode), geometryUnit);
			Angle   = initialNode.GetAngle(finalNode);

			// Set values
			_width  = UnitsNet.Length.From(width, geometryUnit);
			_height = UnitsNet.Length.From(height, geometryUnit);
		}

		/// <summary>
		/// Change the <see cref="LengthUnit"/> of this.
		/// </summary>
		/// <param name="unit">The <see cref="LengthUnit"/> to convert.</param>
		public void ChangeUnit(LengthUnit unit)
		{
			_length.ToUnit(unit);
			_width.ToUnit(unit);
			_height.ToUnit(unit);
		}

		public override string ToString()
		{
			return
				"Lenght = " + _length + "\n" +
				"Width = "  + _width + "\n" +
				"Height = " + _height;
		}

        /// <summary>
        /// Returns true if <see cref="InitialNode"/> and <seealso cref="FinalNode"/> of <paramref name="other"/> coincide.
        /// </summary>
        /// <param name="other">The <see cref="StringerGeometry"/> to compare.</param>
        public bool Equals(StringerGeometry other) =>
			other != null && InitialNode == other.InitialNode && FinalNode == other.FinalNode;

		public override bool Equals(object obj) => obj is StringerGeometry other && Equals(other);

		public override int GetHashCode() => (int) (Length * Area);

		/// <summary>
		/// Returns true if objects are equal.
		/// <para>See: <seealso cref="Equals(StringerGeometry)"/>.</para>
		/// </summary>
		public static bool operator == (StringerGeometry left, StringerGeometry right) => left.Equals(right);

		/// <summary>
		/// Returns true if objects are different.
		/// <para>See: <seealso cref="Equals(StringerGeometry)"/>.</para>
		/// </summary>
		public static bool operator != (StringerGeometry left, StringerGeometry right) => !(left.Equals(right));
	}
}
