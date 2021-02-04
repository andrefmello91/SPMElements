using System;
using Extensions;
using OnPlaneComponents;
using UnitsNet;
using UnitsNet.Units;

namespace SPM.Elements.StringerProperties
{
	/// <summary>
	///     Stringer geometry struct.
	/// </summary>
	public struct StringerGeometry : IUnitConvertible<StringerGeometry, LengthUnit>, IEquatable<StringerGeometry>, IComparable<StringerGeometry>
	{
		#region Fields

		// Auxiliary fields
		private Length _length, _width, _height;

		#endregion

		#region Properties

		/// <summary>
		///     Get/set the <see cref="LengthUnit" /> that this was constructed with.
		/// </summary>
		public LengthUnit Unit
		{
			get => InitialPoint.Unit;
			set => ChangeUnit(value);
		}

		/// <summary>
		///     The stringer angle, in radians.
		/// </summary>
		public double Angle { get; }

		/// <summary>
		///     Get cross-section are of this, in mm2.
		/// </summary>
		public double Area => Width * Height;

		/// <summary>
		///     Get the center <see cref="Point" /> of <see cref="Stringer" />.
		/// </summary>
		public Point CenterPoint { get; private set; }

		/// <summary>
		///     Get the connected <see cref="Point" /> of this.
		/// </summary>
		public Point[] ConnectedPoints => new [] {InitialPoint, CenterPoint, EndPoint};

		/// <summary>
		///     Get the final <see cref="Point" /> of <see cref="Stringer" />.
		/// </summary>
		public Point EndPoint { get; private set; }

		/// <summary>
		///     Get/set the stringer height, in mm.
		/// </summary>
		public double Height
		{
			get => _height.Millimeters;
			set => _height = UnitsNet.Length.FromMillimeters(value).ToUnit(Unit);
		}

		/// <summary>
		///     Get the initial <see cref="Point" /> of <see cref="Stringer" />.
		/// </summary>
		public Point InitialPoint { get; private set; }

		/// <summary>
		///     The stringer length, in mm.
		/// </summary>
		public double Length => _length.Millimeters;

		/// <summary>
		///     Get/set the stringer width, in mm.
		/// </summary>
		public double Width
		{
			get => _width.Millimeters;
			set => _width = UnitsNet.Length.FromMillimeters(value).ToUnit(Unit);
		}

		#endregion

		#region Constructors

		/// <summary>
		///     Stringer geometry object.
		/// </summary>
		/// <param name="initialPoint">
		///     The initial <see cref="Point" /> of the <see cref="Stringer" />, with coordinates in
		///     <paramref name="unit" /> considered.
		/// </param>
		/// <param name="endPoint">
		///     The final <see cref="Point" /> of the <see cref="Stringer" />, with coordinates in
		///     <paramref name="unit" /> considered.
		/// </param>
		/// <param name="width">The stringer width, in <paramref name="unit" /> considered.</param>
		/// <param name="height">The stringer height, in <paramref name="unit" /> considered.</param>
		/// <param name="unit">
		///     The <see cref="LengthUnit" /> of <paramref name="width" />, <paramref name="height" /> and nodes' coordinates.
		///     <para>Default: <seealso cref="LengthUnit.Millimeter" />.</para>
		/// </param>
		public StringerGeometry(Point initialPoint, Point endPoint, double width, double height, LengthUnit unit = LengthUnit.Millimeter)
			: this (initialPoint.Convert(unit), endPoint, UnitsNet.Length.From(width, unit), UnitsNet.Length.From(height, unit))
		{
		}

		/// <summary>
		///     Stringer geometry object.
		/// </summary>
		/// <param name="initialPoint">
		///     The initial <see cref="Point" /> of the <see cref="Stringer" />, in equal unit of
		///     <paramref name="width" /> and <paramref name="height" />.
		/// </param>
		/// <param name="endPoint">
		///     The final <see cref="Point" /> of the <see cref="Stringer" />, in equal unit of
		///     <paramref name="width" /> and <paramref name="height" />.
		/// </param>
		/// <param name="width">The stringer width.</param>
		/// <param name="height">The stringer height.</param>
		public StringerGeometry(Point initialPoint, Point endPoint, Length width, Length height)
		{
			InitialPoint = initialPoint;
			EndPoint     = endPoint.Convert(initialPoint.Unit);
			CenterPoint  = initialPoint.MidPoint(endPoint);

			// Calculate length and angle
			_length = UnitsNet.Length.From(initialPoint.GetDistance(endPoint), width.Unit);
			Angle   = initialPoint.GetAngle(endPoint);

			// Set values
			_width  = width;
			_height = height;
		}

		#endregion

		#region  Methods

		/// <summary>
		///     Convert this <see cref="StringerGeometry" /> object to another <see cref="LengthUnit" />.
		/// </summary>
		/// <param name="unit">The desired <see cref="LengthUnit" />.</param>
		public StringerGeometry Convert(LengthUnit unit) =>
			unit == Unit
				? this
				: new StringerGeometry(InitialPoint.Convert(unit), EndPoint.Convert(unit), Width.ConvertFromMillimeter(unit), Height.ConvertFromMillimeter(unit), unit);

		public StringerGeometry Copy() => new StringerGeometry(InitialPoint, EndPoint, _width, _height);

		/// <summary>
		///     Change the <see cref="LengthUnit" /> of this.
		/// </summary>
		/// <param name="unit">The <see cref="LengthUnit" /> to convert.</param>
		public void ChangeUnit(LengthUnit unit)
		{
			if (Unit == unit)
				return;

			InitialPoint = InitialPoint.Convert(unit);
			EndPoint     = EndPoint.Convert(unit);
			CenterPoint  = InitialPoint.MidPoint(EndPoint);

			_length = _length.ToUnit(unit);
			_width  = _width.ToUnit(unit);
			_height = _height.ToUnit(unit);
		}

		/// <summary>
		///     Compare this <see cref="StringerGeometry" /> to <paramref name="other" />, based on <see cref="CenterPoint" />.
		/// </summary>
		/// <remarks>
		///     See: <seealso cref="Point.CompareTo" />.
		/// </remarks>
		/// <param name="other">The <see cref="StringerGeometry" /> to compare.</param>
		public int CompareTo(StringerGeometry other) => CenterPoint.CompareTo(other.CenterPoint);

		/// <summary>
		///     Returns true if <see cref="InitialPoint" /> and <seealso cref="EndPoint" /> of <paramref name="other" /> coincide.
		/// </summary>
		/// <inheritdoc cref="CompareTo" />
		public bool Equals(StringerGeometry other) =>
			InitialPoint == other.InitialPoint && EndPoint == other.EndPoint ||
			InitialPoint == other.EndPoint     && EndPoint == other.InitialPoint;

		/// <summary>
		///     Returns true if <see cref="Width" /> and <seealso cref="Height" /> of <paramref name="other" /> coincide.
		/// </summary>
		/// <inheritdoc cref="CompareTo" />
		public bool EqualsWidthAndHeight(StringerGeometry other) => Width.Approx(other.Width) && Height.Approx(other.Height);

		public override bool Equals(object obj) => obj is StringerGeometry other && Equals(other);

		public override int GetHashCode() => (int) (Length * Area);

		public override string ToString() =>
			$"Lenght = {_length}\n" +
			$"Width  = {_width}\n" +
			$"Height = {_height}";

		#endregion

		#region Operators

		/// <summary>
		///     Returns true if objects are equal.
		///     <para>See: <seealso cref="Equals(StringerGeometry)" />.</para>
		/// </summary>
		public static bool operator == (StringerGeometry left, StringerGeometry right) => left.Equals(right);

		/// <summary>
		///     Returns true if objects are different.
		///     <para>See: <seealso cref="Equals(StringerGeometry)" />.</para>
		/// </summary>
		public static bool operator != (StringerGeometry left, StringerGeometry right) => !left.Equals(right);

		#endregion
	}
}