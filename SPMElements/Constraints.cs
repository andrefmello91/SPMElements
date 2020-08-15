using System;
using System.Collections.Generic;

namespace SPMElements
{
	// Constraints related commands
	public struct Constraint : IEquatable<Constraint>
	{
		// Properties
		public (bool X, bool Y) Constrained { get; }

        // Constructor
        /// <summary>
        /// Constraint object.
        /// </summary>
        /// <param name="xConstrained">Is X direction constrained?</param>
        /// <param name="yConstrained">Is Y direction constrained?</param>
        public Constraint(bool xConstrained, bool yConstrained)
        {
	        Constrained = (xConstrained, yConstrained);
        }

        /// <summary>
        /// Verify if constraint is free.
        /// </summary>
        public bool IsFree => Constrained == (false, false);

        /// <summary>
        /// Verify if X direction is constrained.
        /// </summary>
        public bool IsXConstrained => Constrained.X;

        /// <summary>
        /// Verify if Y direction is constrained.
        /// </summary>
        public bool IsYConstrained => Constrained.Y;

        /// <summary>
        /// Verify if X and Y directions are constrained.
        /// </summary>
        public bool IsXYConstrained => Constrained == (true, true);

        /// <summary>
        /// Get a free constraint.
        /// </summary>
        /// <param name="gripNumber">The grip number.</param>
        public static Constraint Free => new Constraint(false, false);

        /// <summary>
        /// Get a constraint in X direction.
        /// </summary>
        /// <param name="gripNumber">The grip number.</param>
        public static Constraint XConstrained => new Constraint(true, false);

        /// <summary>
        /// Get a constraint in Y direction.
        /// </summary>
        /// <param name="gripNumber">The grip number.</param>
        public static Constraint YConstrained => new Constraint(false, true);

        /// <summary>
        /// Get a constraint in X and Y directions.
        /// </summary>
        /// <param name="gripNumber">The grip number.</param>
        public static Constraint XYConstrained => new Constraint(true, true);

		/// <summary>
        /// Compare two constraint objects.
        /// </summary>
        /// <param name="other">The constraint to compare.</param>
        /// <returns></returns>
        public bool Equals(Constraint other)
	        => Constrained == other.Constrained;

        public override string ToString()
		{
			string value = "Constrained directions: ";

			if (IsFree)
				return value + "Free";

			if (IsXConstrained)
				value += "X";

			if (IsYConstrained)
				value += "Y";

			return 
				value;
        }
	}

}
