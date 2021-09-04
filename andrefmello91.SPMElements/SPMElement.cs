using System;
using System.Collections.Generic;
using System.Linq;
using andrefmello91.Extensions;
using andrefmello91.FEMAnalysis;
using andrefmello91.OnPlaneComponents;
using MathNet.Numerics.LinearAlgebra;
using UnitsNet;
using static andrefmello91.FEMAnalysis.Extensions;

namespace andrefmello91.SPMElements
{

	/// <summary>
	///     Base class for SPM elements.
	/// </summary>
	/// <inheritdoc cref="ISPMElement{TGeometry}" />
	public abstract class SPMElement<TGeometry> : ISPMElement<TGeometry>, IEquatable<SPMElement<TGeometry>>, IComparable<SPMElement<TGeometry>>
		where TGeometry : struct, IEquatable<TGeometry>, IComparable<TGeometry>
	{

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
		///     Get the displacement vector, in local coordinate system.
		/// </summary>
		protected DisplacementVector LocalDisplacements { get; set; }

		/// <summary>
		///     Get the force vector, in local coordinate system.
		/// </summary>
		protected ForceVector LocalForces { get; set; }

		/// <summary>
		///     Get the local stiffness matrix.
		/// </summary>
		protected StiffnessMatrix LocalStiffness { get; set; }

		/// <summary>
		///     Get the transformation matrix to transform from local to global coordinate systems.
		/// </summary>
		protected Matrix<double> TransformationMatrix { get; set; }

		#endregion

		#region Constructors

		/// <summary>
		///     SPM element base constructor.
		/// </summary>
		/// <param name="geometry">The element's geometry.</param>
		/// <param name="grips">The collection of grips of this element.</param>
		protected SPMElement(TGeometry geometry, IEnumerable<Node> grips)
		{
			Geometry           = geometry;
			Grips              = grips.ToArray();
			LocalForces        = ForceVector.Zero(Grips.Length);
			LocalDisplacements = DisplacementVector.Zero(Grips.Length);
			Forces             = ForceVector.Zero(2 * Grips.Length);
			Displacements      = DisplacementVector.Zero(2 * Grips.Length);
		}

		#endregion

		#region Operators

		/// <summary>
		///     Returns true if arguments are equal.
		/// </summary>
		public static bool operator ==(SPMElement<TGeometry>? left, SPMElement<TGeometry>? right) => left.IsEqualTo(right);

		/// <summary>
		///     Returns true if arguments are different.
		/// </summary>
		public static bool operator !=(SPMElement<TGeometry>? left, SPMElement<TGeometry>? right) => left.IsNotEqualTo(right);

		#endregion

		#region Interface Implementations

		/// <inheritdoc />
		public DisplacementVector Displacements { get; protected set; }

		/// <inheritdoc />
		public int[] DoFIndex => GlobalIndexes(Grips).ToArray();

		/// <inheritdoc />
		public ForceVector Forces { get; protected set; }

		/// <inheritdoc />
		public TGeometry Geometry { get; }

		/// <inheritdoc />
		public Node[] Grips { get; }

		/// <inheritdoc />
		IGrip[] IFiniteElement.Grips => Grips.Cast<IGrip>().ToArray();

		/// <summary>
		///     The <see cref="ElementModel" /> of this SPM element.
		/// </summary>
		public ElementModel Model => this switch
		{
			NLStringer or NLPanel => ElementModel.Nonlinear,
			_                     => ElementModel.Elastic
		};

		/// <inheritdoc />
		public int Number { get; set; }

		/// <inheritdoc />
		public StiffnessMatrix Stiffness { get; protected set; }

		#endregion

		#region Interface Implementations

		/// <inheritdoc />
		public virtual void CalculateForces()
		{
			// Calculate local forces
			LocalForces = LocalStiffness * LocalDisplacements;

			// Approximate small values to zero
			Forces = (ForceVector) (TransformationMatrix.Transpose() * LocalForces);
		}

		/// <inheritdoc />
		public int CompareTo(SPMElement<TGeometry>? other) => other?.Geometry.CompareTo(other.Geometry) ?? 0;

		/// <inheritdoc />
		int IComparable<IFiniteElement>.CompareTo(IFiniteElement? other) => other is SPMElement<TGeometry> spmElement
			? CompareTo(spmElement)
			: 0;

		/// <inheritdoc />
		public bool Equals(SPMElement<TGeometry>? other) => other is not null && Geometry.Equals(other.Geometry);

		/// <inheritdoc />
		bool IEquatable<IFiniteElement>.Equals(IFiniteElement? other) => other is SPMElement<TGeometry> spmElement && Equals(spmElement);

		/// <inheritdoc />
		public virtual void UpdateDisplacements()
		{
			Displacements      = this.GetDisplacementsFromGrips();
			LocalDisplacements = (DisplacementVector) (TransformationMatrix * Displacements);
		}

		/// <inheritdoc />
		public abstract void UpdateStiffness();

		#endregion

		#region Object override

		/// <inheritdoc />
		public override bool Equals(object? obj) => obj is SPMElement<TGeometry> spmElement && Equals(spmElement);

		/// <inheritdoc />
		public override int GetHashCode() => Geometry.GetHashCode();

		#endregion

	}
}