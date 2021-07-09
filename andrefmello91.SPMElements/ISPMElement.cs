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
}