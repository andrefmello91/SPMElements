using System;
using System.Collections.Generic;

namespace SPMElements
{
	/// <summary>
	/// Directions
	/// </summary>
	public enum Direction
	{
		X = 1,
		Y = 2
	}

    /// <summary>
    /// Force struct.
    /// </summary>
    public struct Force : IEquatable<Force>
	{

		// Properties
		public double    Value       { get; }
		public Direction Direction   { get; }

		/// <summary>
        /// Force object.
        /// </summary>
        /// <param name="value">Value of force (in N).
        /// <para>X: positive to right.</para>
        /// <para>Y: positive upwards.</para></param>
        /// <param name="direction">The direction of force.</param>
        public Force(double value, Direction direction)
        {
	        Value      = value;
	        Direction  = direction;
        }

		/// <summary>
        /// Get a Force with zero value.
        /// </summary>
		public static Force Zero(Direction direction) => new Force(0, direction);

		/// <summary>
        /// Verify if force value is zero.
        /// </summary>
		public bool IsZero
			=> Value == 0;

		/// <summary>
        /// Compare two force objects.
        /// </summary>
        /// <param name="other">The force to compare.</param>
        /// <returns></returns>
		public bool Equals(Force other)
			=> Value == other.Value && Direction == other.Direction;

		public override string ToString()
		{
			if (Direction == Direction.X)
				return
					"Fx = " + $"{Value:0.00}" + " N";

			return
				"Fy = " + $"{Value:0.00}" + " N";
		}

		public override bool Equals(object obj)
		{
			if (obj is Force other)
				return Equals(other);

			return false;
		}

		public override int GetHashCode()
		{
			return (int)Value ^ (int)Direction;
		}

		public static bool operator == (Force lhs, Force rhs)
		{
			return lhs.Equals(rhs);
		}

		public static bool operator != (Force lhs, Force rhs)
		{
			return !(lhs.Equals(rhs));
		}
	}
}
