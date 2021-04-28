using System;
using System.Collections.Generic;
using System.Linq;
using andrefmello91.Extensions;
using andrefmello91.FEMAnalysis;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;
using UnitsNet;
using UnitsNet.Units;
using static andrefmello91.FEMAnalysis.Extensions;

namespace andrefmello91.SPMElements
{
	/// <summary>
	///     Base class for SPM elements.
	/// </summary>
	public abstract class SPMElement : IFiniteElement, IEquatable<SPMElement>
	{

		#region Properties

		/// <summary>
		///     Get the grip numbers of this element.
		/// </summary>
		public int[] GripNumbers => Grips.Select(g => g.Number).ToArray();

		/// <summary>
		///     Get the nodes of this element.
		/// </summary>
		public abstract Node[] Grips { get; }

		/// <summary>
		///     Get the maximum local force at this element.
		/// </summary>
		public abstract Force MaxForce { get; }

		/// <summary>
		///     Get the displacement <see cref="Vector" />, in local coordinate system.
		/// </summary>
		/// <remarks>
		///     Components in <see cref="LengthUnit.Millimeter" />.
		/// </remarks>
		protected virtual Vector<double> LocalDisplacements { get; set; }

		/// <summary>
		///     Get the force <see cref="Vector" />, in local coordinate system.
		/// </summary>
		/// <remarks>
		///     Components in <see cref="ForceUnit.Newton" />.
		/// </remarks>
		protected virtual Vector<double> LocalForces { get; set; }

		/// <summary>
		///     Get the local stiffness <see cref="Matrix" />.
		/// </summary>
		protected virtual Matrix<double> LocalStiffness { get; set; }

		/// <summary>
		///     Get the transformation matrix to transform from local to global coordinate systems.
		/// </summary>
		protected Matrix<double> TransformationMatrix { get; set; }

		/// <inheritdoc />
		public virtual Vector<double> Displacements { get; set; }

		/// <inheritdoc />
		public virtual Vector<double> Forces { get; set; }

		/// <inheritdoc />
		public virtual Matrix<double> Stiffness { get; set; }

		/// <inheritdoc />
		IGrip[] IFiniteElement.Grips => Grips.Cast<IGrip>().ToArray();

		/// <inheritdoc />
		public int[] DoFIndex => GlobalIndexes(Grips).ToArray();

		/// <inheritdoc />
		public int Number { get; set; }

		#endregion

		#region Methods

		/// <summary>
		///     The initial iteration values for SPM elements.
		/// </summary>
		/// <param name="size">The size of element's matrices and vectors.</param>
		internal static IEnumerable<IterationResult> InitialValues(int size)
		{
			for (var i = 0; i < 3; i++)
				yield return new IterationResult(size);
		}

		/// <inheritdoc />
		public abstract int CompareTo(IFiniteElement? other);

		/// <inheritdoc />
		public abstract bool Equals(IFiniteElement? other);

		/// <inheritdoc />
		public abstract bool Equals(SPMElement? other);

		/// <inheritdoc />
		public virtual void CalculateForces()
		{
			// Calculate local forces
			LocalForces = LocalStiffness * LocalDisplacements;

			// Approximate small values to zero
			LocalForces.CoerceZero(0.001);
			Forces = TransformationMatrix.Transpose() * LocalForces;
		}

		/// <inheritdoc />
		public virtual void UpdateDisplacements()
		{
			Displacements      = this.GetDisplacementsFromGrips();
			LocalDisplacements = TransformationMatrix * Displacements;
		}

		/// <inheritdoc />
		public abstract void UpdateStiffness();

		#endregion

		#region Operators

		/// <summary>
		///     Returns true if arguments are equal.
		/// </summary>
		public static bool operator ==(SPMElement? left, SPMElement? right) => left.IsEqualTo(right);

		/// <summary>
		///     Returns true if arguments are different.
		/// </summary>
		public static bool operator !=(SPMElement? left, SPMElement? right) => left.IsNotEqualTo(right);

		#endregion

	}
}