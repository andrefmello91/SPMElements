﻿using System;
using System.Collections.Generic;
using andrefmello91.Extensions;
using andrefmello91.OnPlaneComponents;
using UnitsNet;
using UnitsNet.Units;
#nullable disable

namespace andrefmello91.SPMElements.StringerProperties;

/// <summary>
///     Stringer geometry struct.
/// </summary>
public struct StringerGeometry : IUnitConvertible<LengthUnit>, IApproachable<StringerGeometry, Length>, IEquatable<StringerGeometry>, IComparable<StringerGeometry>, ICloneable<StringerGeometry>
{

	#region Fields

	private CrossSection _section;

	#endregion

	#region Properties

	/// <summary>
	///     The stringer angle, in radians.
	/// </summary>
	public double Angle { get; }

	/// <summary>
	///     Get the center <see cref="Point" /> of <see cref="Stringer" />.
	/// </summary>
	public Point CenterPoint { get; }

	/// <summary>
	///     Get the connected <see cref="Point" />'s.
	/// </summary>
	public Point[] ConnectedPoints => new[] { InitialPoint, CenterPoint, EndPoint };

	/// <summary>
	///     Get the cross-section.
	/// </summary>
	public CrossSection CrossSection
	{
		get => _section;
		set => _section = value.Convert(Unit);
	}

	/// <summary>
	///     Get the final <see cref="Point" /> of <see cref="Stringer" />.
	/// </summary>
	public Point EndPoint { get; }

	/// <summary>
	///     Get the initial <see cref="Point" /> of <see cref="Stringer" />.
	/// </summary>
	public Point InitialPoint { get; }

	/// <summary>
	///     The stringer length.
	/// </summary>
	public Length Length { get; private set; }

	/// <summary>
	///     Get/set the <see cref="LengthUnit" /> that this was constructed with.
	/// </summary>
	public LengthUnit Unit
	{
		get => InitialPoint.Unit;
		set => ChangeUnit(value);
	}

	#endregion

	#region Constructors

	/// <inheritdoc cref="StringerGeometry(Point, Point, Length, Length)" />
	/// <param name="unit">
	///     The <see cref="LengthUnit" /> of <paramref name="width" />, <paramref name="height" /> and nodes' coordinates.
	///     <para>Default: <seealso cref="LengthUnit.Millimeter" />.</para>
	/// </param>
	public StringerGeometry(Point initialPoint, Point endPoint, double width, double height, LengthUnit unit = LengthUnit.Millimeter)
		: this(initialPoint.Convert(unit), endPoint, (Length) width.As(unit), (Length) height.As(unit))
	{
	}

	/// <inheritdoc cref="StringerGeometry(Point, Point, CrossSection)" />
	/// <param name="width">The stringer width.</param>
	/// <param name="height">The stringer height.</param>
	public StringerGeometry(Point initialPoint, Point endPoint, Length width, Length height)
		: this(initialPoint, endPoint, new CrossSection(width, height))
	{
	}

	/// <summary>
	///     Stringer geometry object.
	/// </summary>
	/// <param name="initialPoint">
	///     The initial <see cref="Point" /> of the <see cref="Stringer" />.
	/// </param>
	/// <param name="endPoint">
	///     The final <see cref="Point" /> of the <see cref="Stringer" />.
	/// </param>
	/// <param name="crossSection">The stringer cross-section.</param>
	public StringerGeometry(Point initialPoint, Point endPoint, CrossSection crossSection)
	{
		InitialPoint = initialPoint;
		EndPoint     = endPoint.Convert(initialPoint.Unit);
		CenterPoint  = initialPoint.MidPoint(endPoint);

		// Calculate length and angle
		Length = initialPoint.GetDistance(endPoint);
		Angle  = initialPoint.GetAngle(endPoint);

		// Set values
		_section = crossSection;
	}

	#endregion

	#region Methods

	/// <summary>
	///     Convert this <see cref="StringerGeometry" /> object to another <see cref="LengthUnit" />.
	/// </summary>
	/// <param name="unit">The desired <see cref="LengthUnit" />.</param>
	public StringerGeometry Convert(LengthUnit unit) =>
		unit == Unit
			? this
			: new StringerGeometry(InitialPoint.Convert(unit), EndPoint.Convert(unit), CrossSection.Convert(unit));

	/// <summary>
	///     Divide a <see cref="StringerGeometry" /> in a <paramref name="number" /> of new ones.
	/// </summary>
	/// <param name="number">The number of new <see cref="StringerGeometry" />'s. Must be bigger than 1.</param>
	/// <returns>
	///     An empty collection if <paramref name="number" /> is smaller than 2.
	/// </returns>
	public IEnumerable<StringerGeometry> Divide(int number)
	{
		if (number <= 1)
			yield break;

		var iPt = InitialPoint;
		var ePt = EndPoint;

		// Calculate distances
		var dx = (ePt.X - iPt.X) / number;
		var dy = (ePt.Y - iPt.Y) / number;


		for (var i = 0; i < number; i++)
		{
			// Get end point
			ePt = new Point(iPt.X + dx, iPt.Y + dy);

			// Return a divided geometry
			yield return new StringerGeometry(iPt, ePt, CrossSection);

			// Set initial point
			iPt = ePt;
		}
	}

	/// <inheritdoc />
	public override bool Equals(object obj) => obj is StringerGeometry other && Equals(other);

	/// <summary>
	///     Returns true if <see cref="CrossSection" /> of <paramref name="other" /> coincide to this object.
	/// </summary>
	/// <inheritdoc cref="CompareTo" />
	public bool EqualsCrossSection(StringerGeometry other) => CrossSection == other.CrossSection;

	/// <inheritdoc />
	public override int GetHashCode() => InitialPoint.GetHashCode() * EndPoint.GetHashCode();

	/// <inheritdoc />
	public override string ToString() =>
		$"Lenght = {Length}\n" +
		$"Angle = {Angle.ToDegree():0.00} deg\n" +
		CrossSection;

	/// <inheritdoc />
	public bool Approaches(StringerGeometry other, Length tolerance) =>
		InitialPoint.Approaches(other.InitialPoint, tolerance) && EndPoint.Approaches(other.EndPoint, tolerance) ||
		InitialPoint.Approaches(other!.EndPoint, tolerance) && EndPoint.Approaches(other.InitialPoint, tolerance);

	/// <inheritdoc />
	public StringerGeometry Clone() => new(InitialPoint, EndPoint, CrossSection.Clone());

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
	public bool Equals(StringerGeometry other) => Approaches(other, Point.Tolerance);

	/// <summary>
	///     Change the <see cref="LengthUnit" /> of this.
	/// </summary>
	/// <param name="unit">The <see cref="LengthUnit" /> to convert.</param>
	public void ChangeUnit(LengthUnit unit)
	{
		if (Unit == unit)
			return;

		InitialPoint.ChangeUnit(unit);
		EndPoint.ChangeUnit(unit);
		CenterPoint.ChangeUnit(unit);
		CrossSection.ChangeUnit(unit);

		Length = Length.ToUnit(unit);
	}

	IUnitConvertible<LengthUnit> IUnitConvertible<LengthUnit>.Convert(LengthUnit unit) => Convert(unit);

	#endregion

	#region Operators

	/// <summary>
	///     Returns true if objects are equal.
	///     <para>See: <seealso cref="Equals(StringerGeometry)" />.</para>
	/// </summary>
	public static bool operator ==(StringerGeometry left, StringerGeometry right) => left.Equals(right);

	/// <summary>
	///     Returns true if objects are different.
	///     <para>See: <seealso cref="Equals(StringerGeometry)" />.</para>
	/// </summary>
	public static bool operator !=(StringerGeometry left, StringerGeometry right) => !left.Equals(right);

	#endregion

}