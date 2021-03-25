using System;
using System.Linq;
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
	public abstract class SPMElement : IFiniteElement
	{
		/// <summary>
		///		Auxiliary field to <see cref="LocalStiffness"/>.
		/// </summary>
		protected Lazy<Matrix<double>> LocStiffness = null!;
		
		/// <summary>
		///		Auxiliary field to <see cref="TransformationMatrix"/>.
		/// </summary>
		protected Lazy<Matrix<double>> TransMatrix  = null!;

		#region Properties

		/// <summary>
		///     Get the grip numbers of this element.
		/// </summary>
		public int[] GripNumbers => Grips.Select(g => g.Number).ToArray();

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
		protected Vector<double> LocalDisplacements => TransformationMatrix * Displacements;

		/// <summary>
		///     Get the force <see cref="Vector" />, in local coordinate system.
		/// </summary>
		/// <remarks>
		///     Components in <see cref="ForceUnit.Newton" />.
		/// </remarks>
		protected abstract Vector<double> LocalForces { get; set; }

		/// <summary>
		///     Get the local stiffness <see cref="Matrix" />.
		/// </summary>
		protected Matrix<double> LocalStiffness => LocStiffness.Value;

		/// <summary>
		///     Get the transformation matrix to transform from local to global coordinate systems.
		/// </summary>
		protected Matrix<double> TransformationMatrix => TransMatrix.Value;

		/// <inheritdoc />
		public Vector<double> Displacements => this.GetDisplacementsFromGrips();

		/// <inheritdoc />
		public Vector<double> Forces => TransformationMatrix.Transpose() * LocalForces;

		/// <inheritdoc />
		public abstract IGrip[] Grips { get; }

		/// <inheritdoc />
		public Matrix<double> Stiffness => TransformationMatrix.Transpose() * LocalStiffness * TransformationMatrix;

		/// <inheritdoc />
		public int[] DoFIndex => GlobalIndexes(Grips).ToArray();

		/// <inheritdoc />
		public int Number { get; set; }

		#endregion
		#region Methods

		/// <inheritdoc />
		public abstract int CompareTo(IFiniteElement? other);

		/// <inheritdoc />
		public abstract bool Equals(IFiniteElement? other);

		/// <inheritdoc />
		public virtual void CalculateForces()
		{
			// Calculate local forces
			LocalForces = LocalStiffness * LocalDisplacements;

			// Approximate small values to zero
			LocalForces.CoerceZero(0.001);
		}

		#endregion
	}
}