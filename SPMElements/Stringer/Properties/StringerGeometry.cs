using System;
using Autodesk.AutoCAD.Geometry;
using Extensions;
using Extensions.AutoCAD;
using MathNet.Numerics;
using UnitsNet;
using UnitsNet.Units;

namespace SPM.Elements.StringerProperties
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
        /// Get the initial <see cref="Point3d"/> of <see cref="Stringer"/>.
        /// </summary>
        public Point3d InitialPoint { get; }

        /// <summary>
        /// Get the center <see cref="Point3d"/> of <see cref="Stringer"/>.
        /// </summary>
        public Point3d CenterPoint { get; }

        /// <summary>
        /// Get the final <see cref="Point3d"/> of <see cref="Stringer"/>.
        /// </summary>
        public Point3d EndPoint { get; }

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
		/// Get the connected <see cref="Point3d"/> of this.
		/// </summary>
		public Point3d[] ConnectedPoints => new [] {InitialPoint, CenterPoint, EndPoint};

        /// <summary>
        /// Stringer geometry object.
        /// </summary>
        /// <param name="initialPoint">The initial <see cref="Point3d"/> of the <see cref="Stringer"/>, with coordinates in <paramref name="unit"/> considered.</param>
        /// <param name="endPoint">The final <see cref="Point3d"/> of the <see cref="Stringer"/>, with coordinates in <paramref name="unit"/> considered.</param>
        /// <param name="width">The stringer width, in <paramref name="unit"/> considered.</param>
        /// <param name="height">The stringer height, in <paramref name="unit"/> considered.</param>
        /// <param name="unit">The <see cref="LengthUnit"/> of <paramref name="width"/>, <paramref name="height"/> and nodes' coordinates.
        /// <para>Default: <seealso cref="LengthUnit.Millimeter"/>.</para></param>
        public StringerGeometry(Point3d initialPoint, Point3d endPoint, double width, double height, LengthUnit unit = LengthUnit.Millimeter) 
	        : this (initialPoint, endPoint, UnitsNet.Length.From(width, unit), UnitsNet.Length.From(height, unit))
		{
		}

        /// <summary>
        /// Stringer geometry object.
        /// </summary>
        /// <param name="initialPoint">The initial <see cref="Point3d"/> of the <see cref="Stringer"/>, in equal unit of <paramref name="width"/> and <paramref name="height"/>.</param>
        /// <param name="endPoint">The final <see cref="Point3d"/> of the <see cref="Stringer"/>, in equal unit of <paramref name="width"/> and <paramref name="height"/>..</param>
        /// <param name="width">The stringer width.</param>
        /// <param name="height">The stringer height.</param>
        public StringerGeometry(Point3d initialPoint, Point3d endPoint, Length width, Length height)
		{
			InitialPoint = initialPoint;
			EndPoint     = endPoint;
			CenterPoint  = initialPoint.MidPoint(endPoint);

			// Calculate length and angle
			_length = UnitsNet.Length.From(initialPoint.DistanceTo(endPoint), width.Unit);
			Angle   = initialPoint.AngleTo(endPoint);

			// Set values
			_width  = width;
			_height = height;
		}

		/// <summary>
		/// Change the <see cref="LengthUnit"/> of this.
		/// </summary>
		/// <param name="unit">The <see cref="LengthUnit"/> to convert.</param>
		public void ChangeUnit(LengthUnit unit)
		{
			if (Unit == unit)
				return;

			_length = _length.ToUnit(unit);
			_width  = _width.ToUnit(unit);
			_height = _height.ToUnit(unit);
		}

		public override string ToString()
		{
			return
				"Lenght = " + _length + "\n" +
				"Width = "  + _width + "\n" +
				"Height = " + _height;
		}

        /// <summary>
        /// Returns true if <see cref="InitialPoint"/> and <seealso cref="EndPoint"/> of <paramref name="other"/> coincide.
        /// </summary>
        /// <param name="other">The <see cref="StringerGeometry"/> to compare.</param>
        public bool Equals(StringerGeometry other) => InitialPoint == other.InitialPoint && EndPoint == other.EndPoint;

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
