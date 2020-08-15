using System;
using MathNet.Numerics.LinearAlgebra;

namespace SPMElements
{
    public class Node : SPMElement
    {
        /// <summary>
        /// Node types (All excludes displaced)
        /// </summary>
        public enum NodeType
	    {
		    All,
		    External,
		    Internal,
			Displaced
	    }

        // Properties
        /// <summary>
        /// The node type (<see cref="NodeType"/>).
        /// </summary>
        public NodeType Type { get; }

        /// <summary>
        /// Constraint condition of the node.
        /// </summary>
        public Constraint Constraint { get; }

		/// <summary>
        /// Forces in X and Y directions.
        /// </summary>
		public (Force X, Force Y) Forces { get; }

		/// <summary>
		/// Displacements in X and Y directions.
		/// </summary>
		public (Displacement X, Displacement Y) Displacements { get; set; }

        /// <summary>
        /// Node object.
        /// </summary>
        /// <param name="number">The node number.</param>
        /// <param name="type">The node type (<see cref="NodeType"/>).</param>
        /// <param name="xForce">The force in X direction.</param>
        /// <param name="yForce">The force in Y direction.</param>
        /// <param name="constraint">Constraint condition of the node.</param>
        public Node(int number, NodeType type, Force xForce, Force yForce, Constraint constraint)
		{
			// Get the node number
			Number = number;

			// Get type
			Type = type;

            // Get support conditions
            Constraint = constraint;

			// Get forces
			Forces = (xForce, yForce);
		}

		/// <summary>
        /// Get constraint conditions.
        /// </summary>
        public (bool X, bool Y) Constrained => Constraint.Constrained;

		/// <summary>
        /// Verify if the node is free.
        /// </summary>
		public bool IsFree => Constrained == (false, false);

		// Verify if displacement is set
		public bool DisplacementsNotZero => DisplacementValues != (0, 0);

		/// <summary>
        /// Get force values.
        /// </summary>
		public (double X, double Y) ForceValues => (Forces.X.Value, Forces.Y.Value);

		/// <summary>
		/// Get displacement values.
		/// </summary>
		public (double X, double Y) DisplacementValues => (Displacements.X.Value, Displacements.Y.Value);

        /// <summary>
        /// Verify if forces are not zero
        /// </summary>
        public bool ForcesNotZero => ForceValues != (0, 0);

        /// <inheritdoc/>
        public override int[] DoFIndex => GlobalIndexes(Number);

        /// <summary>
        /// Set nodal displacements.
        /// </summary>
        /// <param name="displacementVector">The global displacement vector, with values in mm.</param>
        public void SetDisplacements(Vector<double> displacementVector)
        {
	        var u = displacementVector;

	        // Get the index of the node on the list
	        var index = DoFIndex;
	        int 
		        i = index[0],
		        j = index[1];

	        // Get the displacements
	        double
		        ux = u[i],
		        uy = u[j];

	        // Save to the node
	        Displacements = (new Displacement(ux, Direction.X), new Displacement (uy, Direction.Y));
        }

        public override string ToString()
        {
	        string msgstr =
		        "Node " + Number;

	        // Read applied forces
	        if (ForcesNotZero)
	        {
		        msgstr +=
			        "\n\nApplied forces:";

		        if (!Forces.X.IsZero)
			        msgstr += "\n" + Forces.X;

		        if (!Forces.Y.IsZero)
			        msgstr += "\n" + Forces.Y;
	        }

	        // Get supports
	        if (!IsFree)
		        msgstr += "\n\n" + Constraint;

	        // Get displacements
	        if (DisplacementsNotZero)
	        {
		        // Convert displacements
		        string
			        ux = $"{DisplacementValues.X:0.00}" + " mm",
			        uy = $"{DisplacementValues.Y:0.00}" + " mm";
					
		        msgstr +=
			        "\n\nDisplacements:\n" +
			        "ux = " + ux + "\n" +
			        "uy = " + uy;
	        }

	        return msgstr;
        }
    }
}
