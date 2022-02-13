using System;
using andrefmello91.FEMAnalysis;

namespace andrefmello91.SPMElements;

/// <summary>
///     Interface for SPM elements
/// </summary>
public interface ISPMElement : IFiniteElement
{

	/// <summary>
	///     The nodes of this element.
	/// </summary>
	new Node[] Grips { get; }

	/// <summary>
	///     The <see cref="ElementModel" /> of this SPM element.
	/// </summary>
	ElementModel Model { get; }

	/// <summary>
	///     The name of this element.
	/// </summary>
	string Name { get; }
}

/// <summary>
///     Generic interface for SPM elements
/// </summary>
/// <typeparam name="TGeometry">The struct that represents the geometry of the element.</typeparam>
public interface ISPMElement<out TGeometry> : ISPMElement
	where TGeometry : struct, IEquatable<TGeometry>, IComparable<TGeometry>
{

	/// <summary>
	///     The geometry of this element.
	/// </summary>
	TGeometry Geometry { get; }
}

/// <summary>
///     Interface for nonlinear SPM elements.
/// </summary>
public interface INonlinearSPMElement : ISPMElement, IMonitoredElement
{

	/// <summary>
	///     True if concrete is cracked in this element.
	/// </summary>
	bool ConcreteCracked { get; }

	/// <summary>
	///     True if concrete crushed in this element.
	/// </summary>
	bool ConcreteCrushed { get; }

	/// <summary>
	///     True if concrete yielded in this element.
	/// </summary>
	bool ConcreteYielded { get; }

	/// <summary>
	///     True if steel yielded in this element.
	/// </summary>
	bool SteelYielded { get; }

	/// <summary>
	///     Execute when the state of this element changes.
	/// </summary>
	event EventHandler<StateEventArgs>? StateChanged;
}