using System;
using System.Linq;
using MathNet.Numerics.LinearAlgebra;
using OnPlaneComponents;
using UnitsNet.Units;
using static SPM.Elements.Extensions;

namespace SPM.Elements
{
	/// <summary>
	///     Node class.
	/// </summary>
	public class Node : INumberedElement, IEquatable<Node>
	{
		public int[] DoFIndex => GlobalIndexes(Number).ToArray();

		public int Number { get; set; }

		/// <summary>
		///     Get/set <see cref="Elements.Constraint" /> condition.
		/// </summary>
		public Constraint Constraint { get; set; } = Constraint.Free;

		/// <summary>
		///     Get/set nodal <see cref="OnPlaneComponents.Displacement" />
		/// </summary>
		public Displacement Displacement { get; set; } = Displacement.Zero;

		/// <summary>
		///     Returns true if <see cref="Displacement" /> is not zero.
		/// </summary>
		public bool DisplacementsNotZero => !Displacement.AreComponentsZero;

		/// <summary>
		///     Get/set the <see cref="LengthUnit" /> of <see cref="Displacement" />.
		/// </summary>
		public LengthUnit DisplacementUnit
		{
			get => Displacement.Unit;
			set => Displacement = Displacement.Convert(value);
		}

		/// <summary>
		///     Get/set applied <see cref="OnPlaneComponents.Force" />.
		/// </summary>
		public Force Force { get; set; } = Force.Zero;

		/// <summary>
		///     Returns true if <see cref="Force" /> is not zero.
		/// </summary>
		public bool ForcesNotZero => !Force.AreComponentsZero;

		/// <summary>
		///     Get/set the <see cref="UnitsNet.Units.ForceUnit" /> of <see cref="Force" />.
		/// </summary>
		public ForceUnit ForceUnit
		{
			get => Force.Unit;
			set => Force = Force.Convert(value);
		}

		/// <summary>
		///     Get/set the <see cref="LengthUnit" /> of <see cref="Position" />.
		/// </summary>
		public LengthUnit GeometryUnit
		{
			get => Position.Unit;
			set => Position = Position.Convert(value);
		}

		/// <summary>
		///     Returns true if the node is free.
		/// </summary>
		public bool IsFree => Constraint is Constraint.Free;

		/// <summary>
		///     Get the position of the node.
		///     <para>See: <see cref="Point" />.</para>
		/// </summary>
		public Point Position { get; private set; }

		/// <summary>
		///     Get the node type (<see cref="NodeType" />).
		/// </summary>
		public NodeType Type { get; }

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

		/// <summary>
		///     Return the distance to another <see cref="Node" />.
		/// </summary>
		/// <param name="otherNode">The other <see cref="Node" /> object.</param>
		public double GetDistance(Node otherNode) =>
			!(otherNode is null) ? Position.GetDistance(otherNode.Position) : 0;

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
			var disp = new Displacement(ux, uy).Convert(DisplacementUnit);
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
			if (ForcesNotZero)
				msgstr +=
					"\n\n" +
					"Applied forces:\n" +
					$"{Force}";

			// Get supports
			if (!IsFree)
				msgstr +=
					"\n\n" +
					$"Constraints: {Constraint}";

			// Get displacements
			if (DisplacementsNotZero)
				msgstr +=
					"\n\n" +
					"Displacements:\n" +
					$"{Displacement}";

			return msgstr;
		}

		/// <summary>
		///     Returns true if <paramref name="other" /> is a <see cref="Node" /> and both positions are equal.
		/// </summary>
		/// <param name="other">The other <see cref="Node" /> object.</param>
		public override bool Equals(object other) => other is Node otherNode && Equals(otherNode);

		public override int GetHashCode() => Position.GetHashCode();

		/// <summary>
		///     Returns true if both nodes positions are equal.
		/// </summary>
		public static bool operator == (Node left, Node right) => !(left is null) && left.Equals(right);

		/// <summary>
		///     Returns true if both nodes positions are different.
		/// </summary>
		public static bool operator != (Node left, Node right) => !(left is null) && !left.Equals(right);
	}
}