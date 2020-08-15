using System;

namespace SPMElements
{
    /// <summary>
    /// Force struct.
    /// </summary>
    public struct Displacement : IEquatable<Force>
	{

        // Properties
        /// <summary>
        /// Value of displacement (in mm).
        /// <para>X: positive to right.</para>
        /// <para>Y: positive upwards.</para>
        /// </summary>
        public double    Value       { get; }
        /// <summary>
        /// The direction of displacement.
        /// </summary>
        public Direction Direction   { get; }

		/// <summary>
        /// Force object.
        /// </summary>
        /// <param name="value">Value of displacement (in mm).
		/// <para>X: positive to right.</para>
		/// <para>Y: positive upwards.</para></param>
        /// <param name="direction">The direction of displacement.</param>
        public Displacement(double value, Direction direction)
        {
	        Value      = value;
	        Direction  = direction;
        }

        /// <summary>
        /// Get a Displacement with zero value.
        /// </summary>
        /// <param name="direction">The direction of displacement.</param>
        public static Displacement Zero(Direction direction) => new Displacement(0, direction);

        /// <summary>
        /// Verify if displacement value is zero.
        /// </summary>
        public bool IsZero
			=> Value == 0;

        /// <summary>
        /// Compare two displacement objects.
        /// </summary>
        /// <param name="other">The displacement to compare.</param>
        /// <returns></returns>
        public bool Equals(Force other)
			=> Value == other.Value && Direction == other.Direction;

		public override string ToString()
		{
			if (Direction == Direction.X)
				return
					"ux = " + $"{Value:0.00}" + " mm";

			return
				"uy = " + $"{Value:0.00}" + " mm";
		}

		public override bool Equals(object obj)
		{
			if (obj is Displacement other)
				return Equals(other);

			return false;
		}

		public override int GetHashCode()
		{
			return (int)Value ^ (int)Direction;
		}

		public static bool operator == (Displacement lhs, Displacement rhs)
		{
			return lhs.Equals(rhs);
		}

		public static bool operator != (Displacement lhs, Displacement rhs)
		{
			return !(lhs.Equals(rhs));
		}

    }
}
