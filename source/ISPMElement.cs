﻿using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;

namespace SPM.Elements
{
	/// <summary>
    /// SPM element types.
    /// </summary>
	public enum ElementType
	{
		Default,
		Node,
		Stringer,
		Panel,
		Support,
		Force
	}

	/// <summary>
    /// The analysis types.
    /// </summary>
	public enum AnalysisType
	{
		Linear,
		NonLinear
	}

	/// <summary>
    /// Interface for numbered elements
    /// </summary>
    public interface INumberedElement
    {
		/// <summary>
        /// Get/set the number of this element.
        /// </summary>
	    int Number { get; set ; }

		/// <summary>
		/// Get the DoF index of this element.
		/// </summary>
		int[] DoFIndex { get; }
    }

	/// <summary>
	/// Interface for finite elements.
	/// </summary>
	public interface IFiniteElement : INumberedElement
	{
		/// <summary>
		/// Get the grip numbers of this element.
		/// </summary>
		int[] Grips { get; }

		/// <summary>
		/// Get the local displacement <see cref="Vector"/>.
		/// </summary>
		Vector<double> LocalDisplacements { get; }

		/// <summary>
		/// Get/set global displacement <see cref="Vector"/>.
		/// </summary>
		Vector<double> Displacements { get; }

		/// <summary>
		/// Get/set local force <see cref="Vector"/>.
		/// </summary>
		Vector<double> LocalForces { get; }

		/// <summary>
		/// Get stringer global force <see cref="Vector"/>.
		/// </summary>
		Vector<double> Forces { get; }

		/// <summary>
		/// Get the absolute maximum force in this.
		/// </summary>
		double MaxForce { get; }

		/// <summary>
		/// Get the transformation <see cref="Matrix"/>.
		/// </summary>
		Matrix<double> TransformationMatrix { get; }

		/// <summary>
		/// Get local stiffness <see cref="Matrix"/>.
		/// </summary>
		Matrix<double> LocalStiffness { get; }

		/// <summary>
		/// Get global stiffness <see cref="Matrix"/>.
		/// </summary>
		Matrix<double> Stiffness { get; }

		/// <summary>
		/// Set displacements from global displacement <see cref="Vector"/>.
		/// </summary>
		/// <param name="globalDisplacements">The global displacement <see cref="Vector"/>.</param>
		void SetDisplacements(Vector<double> globalDisplacements);

		/// <summary>
		/// Analyze and calculate forces in this element.
		/// </summary>
		/// <inheritdoc cref="SetDisplacements"/>
		void Analysis(Vector<double> globalDisplacements = null);
	}
}