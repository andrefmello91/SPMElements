using System;
using Extensions;
using UnitsNet;
using UnitsNet.Units;
using static OnPlaneComponents.Point;

namespace SPM.Elements.StringerProperties
{
	/// <summary>
	///     Stringer cross-section struct.
	/// </summary>
	public struct CrossSection : IUnitConvertible<CrossSection, LengthUnit>, IApproachable<CrossSection, Length>, IEquatable<CrossSection>, IComparable<CrossSection>, ICloneable<CrossSection>
	{
		#region Fields

		private Length _width, _height;

		#endregion

		#region Properties

		public LengthUnit Unit
		{
			get => _width.Unit;
			set => ChangeUnit(value);
		}


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
			: this(Length.From(width, unit), Length.From(height, unit))
		{
		}

		#endregion

		#region  Methods

		public void ChangeUnit(LengthUnit unit)
		{
			if (Unit == unit)
				return;

			_width  = _width.ToUnit(unit);
			_height = _height.ToUnit(unit);
		}

		public CrossSection Convert(LengthUnit unit) => new CrossSection(Width.ToUnit(unit), Height.ToUnit(unit));

		public bool Approaches(CrossSection other, Length tolerance) => Width.Approx(other.Width, tolerance) && Height.Approx(other.Width, tolerance);


		public CrossSection Clone() => new CrossSection(Width, Height);

		public int CompareTo(CrossSection other) =>
			Width == other.Width && Height == other.Height
				? 0
				: Width > other.Width || Width >= other.Width && Height > other.Height
					?  1
					: -1;

		public bool Equals(CrossSection other) => Approaches(other, Tolerance);

		public override string ToString() =>
			$"Width = {Width}\n" +
			$"Height = {Height}\n";

		#endregion

		#region Operators

		/// <summary>
		///     Returns true if objects are equal.
		/// </summary>
		public static bool operator == (CrossSection left, CrossSection right) => left.Equals(right);

		/// <summary>
		///     Returns true if objects are different.
		/// </summary>
		public static bool operator != (CrossSection left, CrossSection right) => !left.Equals(right);

		#endregion
	}
}