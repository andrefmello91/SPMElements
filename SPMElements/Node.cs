using System;
using MathNet.Numerics.LinearAlgebra;
using OnPlaneComponents;
using Autodesk.AutoCAD.DatabaseServices;
using Autodesk.AutoCAD.Geometry;
using MathNet.Numerics;
using UnitsNet.Units;
using Extensions;
using Extensions.AutoCAD;

namespace SPMElements
{
	/// <summary>
	/// Node types.
	/// </summary>
	public enum NodeType
	{
		/// <summary>
        /// All nodes (excluding displaced nodes).
        /// </summary>
		All,
		/// <summary>
        /// External nodes (grips to stringers).
        /// </summary>
		External,
		/// <summary>
		/// External nodes (mid grip to stringers and grip for panels).
		/// </summary>
		Internal,
		/// <summary>
		/// Displaced nodes (auxiliary, only for result drawing).
		/// </summary>
		Displaced
    }

	/// <summary>
    /// Constraint directions.
    /// </summary>
	public enum Constraint
	{
		/// <summary>
        /// Displacement free at both directions.
        /// </summary>
		Free,
		/// <summary>
        /// Displacement restricted at X direction.
        /// </summary>
		X,
		/// <summary>
		/// Displacement restricted at Y direction.
		/// </summary>
		Y,
		/// <summary>
		/// Displacement restricted at both directions.
		/// </summary>
		XY
    }

    public class Node : SPMElement, IEquatable<Node>
    {
		// Auxiliary fields
		private LengthUnit _geometryUnit, _displacementUnit;

        /// <summary>
        /// Get the node type (<see cref="NodeType"/>).
        /// </summary>
        public NodeType Type { get; }

		/// <summary>
        /// Get the position of the node.
        /// <para>See: <see cref="Point3d"/>.</para>
        /// </summary>
		public Point3d Position { get; }

		/// <summary>
        /// Get applied <see cref="OnPlaneComponents.Force"/>.
        /// </summary>
		public Force Force { get; }

        /// <summary>
        /// Get <see cref="SPMElements.Constraint"/> condition.
        /// </summary>
        public Constraint Constraint { get; }

		/// <summary>
		/// Get/set nodal <see cref="OnPlaneComponents.Displacement"/>
		/// </summary>
		public Displacement Displacement { get; private set; }

		/// <summary>
        /// Returns true if the node is free.
        /// </summary>
		public bool IsFree => Constraint is Constraint.Free;

		/// <summary>
        /// Returns true if <see cref="Displacement"/> is not zero.
        /// </summary>
		public bool DisplacementsNotZero => !Displacement.AreComponentsZero;

        /// <summary>
        /// Returns true if <see cref="Force"/> is not zero.
        /// </summary>
        public bool ForcesNotZero => !Force.AreComponentsZero;

        /// <inheritdoc/>
        public override int[] DoFIndex => _globalIndexes ?? GlobalIndexes(Number);

        /// <summary>
        /// Node object.
        /// </summary>
        /// <param name="objectId">The node <see cref="ObjectId"/>.</param>
        /// <param name="number">The node number.</param>
        /// <param name="position">The <seealso cref="Point3d"/> position.</param>
        /// <param name="type">The <see cref="NodeType"/>.</param>
        /// <param name="appliedForce">The applied <see cref="OnPlaneComponents.Force"/> on the node.</param>
        /// <param name="constraint"> The <see cref="SPMElements.Constraint"/> condition of the node.</param>
        /// <param name="geometryUnit">The <see cref="LengthUnit"/> of <paramref name="position"/>.</param>
        /// <param name="displacementUnit">The <see cref="LengthUnit"/> of <see cref="Displacement"/>.</param>
        public Node(ObjectId objectId, int number, Point3d position, NodeType type, Force appliedForce, Constraint constraint = Constraint.Free, LengthUnit geometryUnit = LengthUnit.Millimeter, LengthUnit displacementUnit = LengthUnit.Millimeter) : base(objectId, number)
        {
	        // Get the position
	        Position = position;

	        // Get type
	        Type = type;

	        // Get support conditions
	        Constraint = constraint;

	        // Get forces
	        Force = appliedForce;

	        // Set units
	        _geometryUnit = geometryUnit;
	        _displacementUnit = displacementUnit;

	        // Initiate displacements
	        Displacement = Displacement.Zero;
        }

        /// <summary>
        /// Return the distance to another <see cref="Node"/>.
        /// <para>See: <seealso cref="Point3d.DistanceTo"/></para>
        /// </summary>
        /// <param name="otherNode">The other <see cref="Node"/> object.</param>
        public double GetDistance(Node otherNode) => otherNode != null ? Position.DistanceTo(otherNode.Position) : 0;

        /// <summary>
        /// Return the angle, related to horizontal axis, of a line that connects this to <paramref name="otherNode"/> (in radians).
        /// </summary>
        /// <param name="otherNode">The other <see cref="Node"/> object.</param>
        public double GetAngle(Node otherNode) => otherNode != null ? Position.AngleTo(otherNode.Position) : 0;

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

	        // Save to the node, in mm
	        Displacement = new Displacement(ux, uy);

			// Change unit
			ChangeDisplacementUnit(_displacementUnit);
        }

        /// <summary>
        /// Change the unit of <see cref="Displacement"/>.
        /// </summary>
        /// <param name="displacementUnit">The <see cref="LengthUnit"/> to convert.</param>
        public void ChangeDisplacementUnit(LengthUnit displacementUnit)
        {
	        Displacement.ChangeUnit(displacementUnit);
	        _displacementUnit = displacementUnit;
        }


        public override string ToString()
        {
	        string msgstr =
		        "Node " + Number + "\n" +
		        "Position: (" + Position.X + ", " + Position.Y + ")";

	        // Read applied forces
	        if (ForcesNotZero)
		        msgstr +=
			        "\n\nApplied forces: \n" + Force;

	        // Get supports
	        if (!IsFree)
		        msgstr +=
			        "\n\nConstraints: " + Constraint;

	        // Get displacements
	        if (DisplacementsNotZero)
		        msgstr +=
			        "\n\nDisplacements: \n" + Displacement;

	        return msgstr;
        }

		/// <summary>
        /// Returns true if both nodes positions are equal.
        /// </summary>
        /// <param name="other">The other <see cref="Node"/> object.</param>
        public bool Equals(Node other) => other != null && Position == other.Position;

		/// <summary>
		/// Returns true if <paramref name="other"/> is a <see cref="Node"/> and both positions are equal.
		/// </summary>
		/// <param name="other">The other <see cref="Node"/> object.</param>
		public override bool Equals(object other) => other is Node otherNode && Equals(otherNode);

		public override int GetHashCode() => Position.GetHashCode();

		/// <summary>
        /// Returns true if both nodes positions are equal.
        /// </summary>
        public static bool operator == (Node left, Node right) => left != null && left.Equals(right);

        /// <summary>
        /// Returns true if both nodes positions are different.
        /// </summary>
        public static bool operator != (Node left, Node right) => left != null && !left.Equals(right);
    }
}
