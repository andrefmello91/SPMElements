﻿using System;
using andrefmello91.Extensions;
using andrefmello91.OnPlaneComponents;
using UnitsNet;
using UnitsNet.Units;

namespace andrefmello91.SPMElements.StringerProperties;

/// <summary>
///     Stringer cross-section struct.
/// </summary>
public struct CrossSection : IUnitConvertible<LengthUnit>, IApproachable<CrossSection, Length>, IEquatable<CrossSection>, IComparable<CrossSection>, ICloneable<CrossSection>
{

	#region Fields

	private Length _width, _height;

	#endregion

	#region Properties

	/// <summary>
	///     Get the cross-section area.
	/// </summary>
	public Area Area => (Width * Height).ToUnit(Unit.GetAreaUnit());

	/// <summary>
	///     Get/set the cross-section height.
	/// </summary>
	public Length Height
	{
		get => _height;
		set => _height = value.ToUnit(Unit);
	}

	/// <summary>
	///     Get/set the cross-section width.
	/// </summary>
	public Length Width
	{
		get => _width;
		set => _width = value.ToUnit(Unit);
	}

	/// <inheritdoc />
	public LengthUnit Unit
	{
		get => _width.Unit;
		set => ChangeUnit(value);
	}

	#endregion

	#region Constructors

	/// <summary>
	///     Stringer cross-section constructor.
	/// </summary>
	/// <param name="width">The stringer width.</param>
	/// <param name="height">The stringer height.</param>
	public CrossSection(Length width, Length height)
	{
		_width  = width;
		_height = height.ToUnit(_width.Unit);
	}

	/// <param name="unit">
	///     The <see cref="LengthUnit" /> of <paramref name="width" />, <paramref name="height" /> and nodes' coordinates.
	///     <para>Default: <seealso cref="LengthUnit.Millimeter" />.</para>
	/// </param>
	/// <inheritdoc cref="CrossSection(Length, Length)" />
	public CrossSection(double width, double height, LengthUnit unit = LengthUnit.Millimeter)
		: this((Length) width.As(unit), (Length) height.As(unit))
	{
	}

	#endregion

	#region Methods

	/// <inheritdoc cref="IUnitConvertible{TUnit}.Convert" />
	public CrossSection Convert(LengthUnit unit) => new(Width.ToUnit(unit), Height.ToUnit(unit));

	/// <inheritdoc />
	public override string ToString() =>
		$"Width = {Width}\n" +
		$"Height = {Height}\n";

	/// <inheritdoc />
	public bool Approaches(CrossSection other, Length tolerance) => Width.Approx(other.Width, tolerance) && Height.Approx(other.Height, tolerance);


	/// <inheritdoc />
	public CrossSection Clone() => new(Width, Height);

	/// <inheritdoc />
	public int CompareTo(CrossSection other) =>
		Width == other.Width && Height == other.Height
			? 0
			: Width > other.Width || Width >= other.Width && Height > other.Height
				? 1
				: -1;

	/// <inheritdoc />
	public bool Equals(CrossSection other) => Approaches(other, Point.Tolerance);

	/// <inheritdoc />
	public void ChangeUnit(LengthUnit unit)
	{
		if (Unit == unit)
			return;

		_width  = _width.ToUnit(unit);
		_height = _height.ToUnit(unit);
	}

	IUnitConvertible<LengthUnit> IUnitConvertible<LengthUnit>.Convert(LengthUnit unit) => Convert(unit);

	#endregion

	#region Operators

	/// <summary>
	///     Returns true if objects are equal.
	/// </summary>
	public static bool operator ==(CrossSection left, CrossSection right) => left.Equals(right);

	/// <summary>
	///     Returns true if objects are different.
	/// </summary>
	public static bool operator !=(CrossSection left, CrossSection right) => !left.Equals(right);

	#endregion

}