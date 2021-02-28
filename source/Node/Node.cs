using System;
using System.Linq;
using MathNet.Numerics.LinearAlgebra;
using OnPlaneComponents;
using UnitsNet;
using UnitsNet.Units;
using static SPM.Elements.Extensions;

#nullable enable

namespace SPM.Elements
{
	/// <summary>
	///     Node class.
	/// </summary>
	public class Node : INumberedElement, IEquatable<Node>, IComparable<Node>
	{
		#region Properties

		public int[] DoFIndex => GlobalIndexes(Number).ToArray();

		public int Number { get; set; }

		/// <summary>
		///     Get/set <see cref="OnPlaneComponents.Constraint" /> condition.
		/// </summary>
		public Constraint Constraint { get; set; } = Constraint.Free;

		/// <summary>
		///     Get/set nodal <see cref="PlaneDisplacement" />
		/// </summary>
		public PlaneDisplacement Displacement { get; set; } = PlaneDisplacement.Zero;

		/// <summary>
		///     Get/set the <see cref="LengthUnit" /> of <see cref="Displacement" />.
		/// </summary>
		public LengthUnit DisplacementUnit
		{
			get => Displacement.Unit;
			set => Displacement.ChangeUnit(value);
		}

		/// <summary>
		///     Get/set applied <see cref="PlaneForce" />.
		/// </summary>
		public PlaneForce PlaneForce { get; set; } = PlaneForce.Zero;

		/// <summary>
		///     Get/set the <see cref="UnitsNet.Units.ForceUnit" /> of <see cref="PlaneForce" />.
		/// </summary>
		public ForceUnit ForceUnit
		{
			get => PlaneForce.Unit;
			set => PlaneForce.ChangeUnit(value);
		}

		/// <summary>
		///     Get/set the <see cref="LengthUnit" /> of <see cref="Position" />.
		/// </summary>
		public LengthUnit GeometryUnit
		{
			get => Position.Unit;
			set => Position.ChangeUnit(value);
		}

		/// <summary>
		///     Returns true if the node is free.
		/// </summary>
		public bool IsFree => Constraint.Direction == ComponentDirection.None;

		/// <summary>
		///     Get the position of the node.
		///     <para>See: <see cref="Point" />.</para>
		/// </summary>
		public Point Position { get; }

		/// <summary>
		///     Get the node type (<see cref="NodeType" />).
		/// </summary>
		public NodeType Type { get; }

		#endregion

		#region Constructors

		/// <summary>
		///     Node object.
		/// </summary>
		/// <param name="position">The <seealso cref="Point" /> position.</param>
		/// <param name="type">The <see cref="NodeType" />.</param>
		/// <param name="displacementUnit">The <see cref="LengthUnit" /> of <see cref="Displacement" />.</param>
		public Node(Point position, NodeType type, LengthUnit displacementUnit = LengthUnit.Millimeter)
		{
			// Get the position
			Position = position;

			// Get type
			Type = type;

			// Set units
			DisplacementUnit = displacementUnit;
		}

		#endregion

		#region  Methods

		/// <summary>
		///     Return the distance to another <see cref="Node" />.
		/// </summary>
		/// <param name="otherNode">The other <see cref="Node" /> object.</param>
		public Length GetDistance(Node? otherNode) => !(otherNode is null) ? Position.GetDistance(otherNode.Position) : Length.Zero;

		/// <summary>
		///     Return the angle, related to horizontal axis, of a line that connects this to <paramref name="otherNode" /> (in
		///     radians).
		/// </summary>
		/// <inheritdoc cref="GetDistance" />
		public double GetAngle(Node otherNode) => !(otherNode is null) ? Position.GetAngle(otherNode.Position) : 0;

		/// <summary>
		///     Set nodal displacements.
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
			var disp = new PlaneDisplacement(ux, uy).Convert(DisplacementUnit);
			Displacement = disp;
		}

		/// <summary>
		///     Returns true if both nodes positions are equal.
		/// </summary>
		/// <param name="other">The other <see cref="Node" /> object.</param>
		public bool Equals(Node other) => !(other is null) && Position == other.Position;

		public override string ToString()
		{
			var msgstr =
				$"Node {Number}\n" +
				$"Position: ({Position.X:0.00}, {Position.Y:0.00})";

			// Read applied forces
			if (!PlaneForce.IsZero)
				msgstr +=
					"\n\n" +
					"Applied forces:\n" +
					$"{PlaneForce}";

			// Get supports
			if (!IsFree)
				msgstr +=
					"\n\n" +
					$"Constraints: {Constraint}";

			// Get displacements
			if (!Displacement.IsZero)
				msgstr +=
					"\n\n" +
					"Displacements:\n" +
					$"{Displacement}";

			return msgstr;
		}

		public int CompareTo(Node? other) => other is null 
			? 1 
			: Position.CompareTo(other.Position);

		/// <summary>
		///     Returns true if <paramref name="other" /> is a <see cref="Node" /> and both positions are equal.
		/// </summary>
		/// <param name="other">The other <see cref="Node" /> object.</param>
		public override bool Equals(object? other) => other is Node otherNode && Equals(otherNode);

		public override int GetHashCode() => Position.GetHashCode();

		#endregion

		#region Operators

		/// <summary>
		///     Returns true if both nodes positions are equal.
		/// </summary>
		public static bool operator == (Node left, Node right) => !(left is null) && left.Equals(right);

		/// <summary>
		///     Returns true if both nodes positions are different.
		/// </summary>
		public static bool operator != (Node left, Node right) => !(left is null) && !left.Equals(right);

		#endregion
	}
}