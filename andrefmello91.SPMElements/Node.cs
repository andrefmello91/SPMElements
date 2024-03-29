﻿using System;
using System.Linq;
using andrefmello91.FEMAnalysis;
using andrefmello91.OnPlaneComponents;
using UnitsNet;
using UnitsNet.Units;
using static andrefmello91.FEMAnalysis.Extensions;

#nullable enable

namespace andrefmello91.SPMElements;

/// <summary>
///     Node class.
/// </summary>
public class Node : IGrip, IMonitoredElement, IEquatable<Node>, IComparable<Node>
{

	#region Properties

	/// <summary>
	///     Get/set the <see cref="LengthUnit" /> of <see cref="Displacement" />.
	/// </summary>
	public LengthUnit DisplacementUnit
	{
		get => Displacement.Unit;
		set => Displacement.ChangeUnit(value);
	}

	/// <summary>
	///     Get/set the <see cref="UnitsNet.Units.ForceUnit" /> of <see cref="Force" />.
	/// </summary>
	public ForceUnit ForceUnit
	{
		get => Force.Unit;
		set => Force.ChangeUnit(value);
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
	public bool IsFree => Constraint.Direction is ComponentDirection.None;

	/// <summary>
	///     Get the position of the node.
	///     <para>See: <see cref="Point" />.</para>
	/// </summary>
	public Point Position { get; }

	/// <summary>
	///     Get the node type (<see cref="NodeType" />).
	/// </summary>
	public NodeType Type { get; }

	/// <summary>
	///     Get/set <see cref="OnPlaneComponents.Constraint" /> condition.
	/// </summary>
	public Constraint Constraint { get; set; } = Constraint.Free;

	/// <summary>
	///     Get/set nodal <see cref="PlaneDisplacement" />
	/// </summary>
	public PlaneDisplacement Displacement { get; set; } = PlaneDisplacement.Zero;

	/// <inheritdoc />
	public int[] DoFIndex => GlobalIndexes(this).ToArray();

	/// <summary>
	///     Get/set applied <see cref="Force" />.
	/// </summary>
	public PlaneForce Force { get; set; } = PlaneForce.Zero;

	/// <inheritdoc />
	public string Name => $"Node {Number}";

	/// <inheritdoc />
	public int Number { get; set; }

	/// <inheritdoc />
	public PlaneForce Reaction { get; set; }

	/// <inheritdoc />
	public ElementMonitor? Monitor { get; private set; }

	/// <summary>
	///     Get/set monitoring of this node.
	/// </summary>
	public bool Monitored
	{
		get => Monitor is not null;
		set
		{
			if (value)
				Monitor ??= new GripMonitor(Name, DisplacementUnit);
			else
				Monitor = null;
		}
	}

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

	#region Methods

	/// <summary>
	///     Returns true if <paramref name="other" /> is a <see cref="Node" /> and both positions are equal.
	/// </summary>
	/// <param name="other">The other <see cref="Node" /> object.</param>
	public override bool Equals(object? other) => other is Node otherNode && Equals(otherNode);

	/// <summary>
	///     Return the angle, related to horizontal axis, of a line that connects this to <paramref name="otherNode" /> (in
	///     radians).
	/// </summary>
	/// <inheritdoc cref="GetDistance" />
	public double GetAngle(Node? otherNode) => otherNode is not null ? Position.GetAngle(otherNode.Position) : 0;

	/// <summary>
	///     Return the distance to another <see cref="Node" />.
	/// </summary>
	/// <param name="otherNode">The other <see cref="Node" /> object.</param>
	public Length GetDistance(Node? otherNode) => otherNode is not null ? Position.GetDistance(otherNode.Position) : Length.Zero;

	/// <inheritdoc />
	public override int GetHashCode() => Position.GetHashCode();

	/// <inheritdoc />
	public override string ToString()
	{
		var msgstr =
			$"Node {Number}\n" +
			$"Position:\t({Position.X}, {Position.Y})\n" +
			$"DoFIndex:\t{DoFIndex.Select(i => i.ToString()).Aggregate((i, f) => $"{i} - {f}")}";

		// Read applied forces
		if (!Force.IsZero)
			msgstr +=
				"\n\n" +
				"Applied forces:\n" +
				$"{Force}";

		// Get supports
		if (!IsFree)
			msgstr +=
				"\n\n" +
				$"Constraints:\t{Constraint.Direction}";

		// Get displacements
		if (!Displacement.IsZero)
			msgstr +=
				"\n\n" +
				"Displacements:\n" +
				$"{Displacement}";

		return msgstr;
	}

	/// <inheritdoc />
	public int CompareTo(Node? other) => other is null
		? 1
		: Position.CompareTo(other.Position);

	/// <summary>
	///     Returns true if both nodes positions are equal.
	/// </summary>
	/// <param name="other">The other <see cref="Node" /> object.</param>
	public bool Equals(Node? other) => other is not null && Position == other.Position;

	/// <inheritdoc />
	int IComparable<IGrip>.CompareTo(IGrip? other) => other is Node node
		? CompareTo(node)
		: 0;

	/// <inheritdoc />
	bool IEquatable<IGrip>.Equals(IGrip? other) => other is Node node && Equals(node);

	/// <inheritdoc />
	public void AddValue(double loadFactor) => Monitor?.AddMonitoredValue(loadFactor, this);

	#endregion

	#region Operators

	/// <summary>
	///     Returns true if both nodes positions are equal.
	/// </summary>
	public static bool operator ==(Node? left, Node? right) => left is not null && left.Equals(right);

	/// <summary>
	///     Returns true if both nodes positions are different.
	/// </summary>
	public static bool operator !=(Node? left, Node? right) => left is not null && !left.Equals(right);

	#endregion

}