using System;
using andrefmello91.FEMAnalysis;

namespace andrefmello91.SPMElements
{
	/// <summary>
	///     Interface for SPM elements
	/// </summary>
	public interface ISPMElement : IFiniteElement
	{

		#region Properties

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

		#endregion

	}

	/// <summary>
	///     Generic interface for SPM elements
	/// </summary>
	/// <typeparam name="TGeometry">The struct that represents the geometry of the element.</typeparam>
	public interface ISPMElement<out TGeometry> : ISPMElement
		where TGeometry : struct, IEquatable<TGeometry>, IComparable<TGeometry>
	{

		#region Properties

		/// <summary>
		///     The geometry of this element.
		/// </summary>
		TGeometry Geometry { get; }

		#endregion

	}

	/// <summary>
	///     Interface for nonlinear SPM elements.
	/// </summary>
	public interface INonlinearSPMElement : ISPMElement
	{

		#region Properties

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

		#endregion

		#region Events

		/// <summary>
		///     Execute when the state of this element changes.
		/// </summary>
		event EventHandler? StateChanged;

		#endregion

	}
}