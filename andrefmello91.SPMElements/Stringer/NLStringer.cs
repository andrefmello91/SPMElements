using System.Collections.Generic;
using System.Diagnostics.CodeAnalysis;
using System.Linq;
using andrefmello91.Extensions;
using andrefmello91.FEMAnalysis;
using andrefmello91.Material.Concrete;
using andrefmello91.Material.Reinforcement;
using andrefmello91.OnPlaneComponents;
using andrefmello91.SPMElements.StringerProperties;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;
using UnitsNet;
using UnitsNet.Units;
#nullable enable

namespace andrefmello91.SPMElements
{
	/// <summary>
	///     Nonlinear stringer class.
	/// </summary>
	internal class NLStringer : Stringer
	{

		#region Fields

		// Auxiliary fields
		private readonly Matrix<double> _bMatrix;

		private Force _n1, _n3;

		#endregion

		#region Properties

		/// <inheritdoc />
		public override Length[] CrackOpenings => Strains.Select(eps => CrackOpening(Reinforcement, eps)).ToArray();

		/// <summary>
		///     Get the strain <see cref="Vector" />.
		/// </summary>
		private Vector<double> Strains => _bMatrix * LocalDisplacements.Convert(LengthUnit.Millimeter);

		#endregion

		#region Constructors

		/// <summary>
		///     Nonlinear stringer object.
		/// </summary>
		/// <inheritdoc cref="Stringer(Node, Node, Node, CrossSection, IParameters, ConstitutiveModel, UniaxialReinforcement)" />
		internal NLStringer(Node grip1, Node grip2, Node grip3, CrossSection crossSection, IParameters concreteParameters, ConstitutiveModel model = ConstitutiveModel.MCFT, UniaxialReinforcement? reinforcement = null)
			: base(grip1, grip2, grip3, crossSection)
		{
			Reinforcement = reinforcement;
			Concrete      = new UniaxialConcrete(concreteParameters, GetConcreteArea(this), model);

			if (Reinforcement is not null)
				Reinforcement.ConcreteArea = Concrete.Area;

			_bMatrix = CalculateBMatrix(Geometry.Length);

			// Calculate matrices
			TransformationMatrix = CalculateTransformationMatrix(Geometry.Angle);
			LocalStiffness       = CalculateStiffness(Concrete.Stiffness, Geometry.Length);
			Stiffness            = (StiffnessMatrix) LocalStiffness.Transform(TransformationMatrix);
		}

		#endregion

		#region Methods

		/// <summary>
		///     Calculate B Matrix based on stringer length.
		/// </summary>
		/// <param name="stringerLength">The length of the stringer.</param>
		private static Matrix<double> CalculateBMatrix(Length stringerLength) =>
			1D / stringerLength.Millimeters * new double[,]
			{
				{ -3, 4, -1 },
				{ -1, 0, 1 },
				{ 1, -4, 3 }
			}.ToMatrix();

		/// <summary>
		///     Calculate force for <paramref name="concrete" /> and <paramref name="reinforcement" /> based on strain.
		/// </summary>
		/// <param name="strain">Current strain.</param>
		/// <param name="concrete">The uniaxial concrete of the stringer.</param>
		/// <param name="reinforcement">The <see cref="UniaxialReinforcement" /> of the stringer.</param>
		private static Force CalculateForce(double strain, [NotNull] UniaxialConcrete concrete, UniaxialReinforcement? reinforcement) =>
			strain.ApproxZero(1E-9)
				? Force.Zero
				: concrete.CalculateForce(strain, reinforcement) + (reinforcement?.CalculateForce(strain) ?? Force.Zero);

		/// <summary>
		///     Calculate the average crack opening.
		/// </summary>
		/// <param name="reinforcement">The <see cref="UniaxialReinforcement" />.</param>
		/// <param name="strain">The strain.</param>
		private static Length CrackOpening(UniaxialReinforcement? reinforcement, double strain) => strain < 0 || strain.ApproxZero(1E-9)
			? Length.Zero
			: strain * CrackSpacing(reinforcement);

		/// <summary>
		///     Calculate the crack spacing at <paramref name="reinforcement" />, according to Kaklauskas (2019)
		///     expression.
		///     <para>sm = 21 mm + 0.155 phi / rho</para>
		/// </summary>
		/// <param name="reinforcement">The <see cref="UniaxialReinforcement" />.</param>
		private static Length CrackSpacing(UniaxialReinforcement? reinforcement) =>
			reinforcement is null || reinforcement.BarDiameter.ApproxZero(Point.Tolerance) || reinforcement.Ratio.ApproxZero()
				? Length.FromMillimeters(21)
				: Length.FromMillimeters(21) + 0.155 * reinforcement.BarDiameter / reinforcement.Ratio;

		/// <inheritdoc />
		public override void CalculateForces()
		{
			// Get strains
			var eps = Strains;

			// Calculate normal forces
			_n1 = CalculateForce(eps[0], Concrete, Reinforcement).ToUnit(ForceUnit.Newton);
			_n3 = CalculateForce(eps[2], Concrete, Reinforcement).ToUnit(ForceUnit.Newton);

			// Update forces
			LocalForces = new ForceVector(new[] { -_n1, _n1 - _n3, _n3 });

			Forces = (ForceVector) (TransformationMatrix.Transpose() * LocalForces);
		}

		#endregion

	}
}