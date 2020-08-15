using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace SPMElements
{
	/// <summary>
	/// Stringer geometry struct.
	/// </summary>
	public struct StringerGeometry : IEquatable<StringerGeometry>
	{
		/// <summary>
		/// The stringer length, in mm.
		/// </summary>
		public double Length { get; set; }

		/// <summary>
		/// The stringer angle, in radians.
		/// </summary>
		public double Angle { get; set; }

		/// <summary>
		/// The stringer width, in mm.
		/// </summary>
		public double Width { get; set; }

		/// <summary>
		/// The stringer height, in mm.
		/// </summary>
		public double Height { get; set; }

		/// <summary>
		/// Stringer geometry object.
		/// </summary>
		/// <param name="length">The stringer length, in mm.</param>
		/// <param name="angle">The stringer angle, in radians.</param>
		/// <param name="width">The stringer width, in mm.</param>
		/// <param name="height">The stringer height, in mm.</param>
		public StringerGeometry(double length, double angle, double width, double height)
		{
			Length = length;
			Angle  = angle;
			Width  = width;
			Height = height;
		}

        /// <summary>
        /// Stringer geometry object.
        /// </summary>
        /// <param name="initial">The initial vertex (<see cref="Vertex"/>).</param>
        /// <param name="final">The final vertex (<see cref="Vertex"/>)</param>
        /// <param name="width">The stringer width, in mm.</param>
        /// <param name="height">The stringer height, in mm.</param>
        public StringerGeometry(Vertex initial, Vertex final, double width, double height)
		{
			Length = initial.DistanceTo(final);
			Angle  = initial.AngleTo(final);
			Width  = width;
			Height = height;
		}

        /// <summary>
        /// Compare with another stringer geometry (only length and angle).
        /// </summary>
        /// <param name="other">The stringer geometry to compare.</param>
        /// <returns></returns>
        public bool Equals(StringerGeometry other) => Length == other.Length && Angle == other.Angle;

        public override bool Equals(object obj)
        {
	        if (obj is StringerGeometry other)
		        return Equals(other);

	        return false;
        }

        public override int GetHashCode()
        {
	        return (int)Length ^ (int)Angle;
        }

        public static bool operator == (StringerGeometry lhs, StringerGeometry rhs)
        {
	        return lhs.Equals(rhs);
        }

        public static bool operator != (StringerGeometry lhs, StringerGeometry rhs)
        {
	        return !(lhs.Equals(rhs));
        }
	}
}
