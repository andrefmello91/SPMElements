using System;
using System.Collections.Generic;
using System.Linq;
using Extensions.LinearAlgebra;
using Extensions.Number;
using Material.Concrete;
using Material.Concrete.Uniaxial;
using Material.Reinforcement;
using Material.Reinforcement.Uniaxial;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;
using OnPlaneComponents;
using UnitsNet;
using UnitsNet.Units;

namespace SPM.Elements
{
	/// <summary>
	/// Nonlinear stringer class.
	/// </summary>
	public class NLStringer : Stringer
	{
		// Auxiliary fields
		private Matrix<double> _BMatrix;
		private double _N1, _N3;

		protected override double ConcreteArea => Geometry.Area - (Reinforcement?.Area ?? 0);

		/// <summary>
		/// Get B <see cref="Matrix"/> to transform displacements in strains.
		/// </summary>
		private Matrix<double> BMatrix => _BMatrix ?? CalculateBMatrix();

		/// <summary>
		/// Get the strain <see cref="Vector"/>
		/// </summary>
		private Vector<double> Strains => BMatrix * LocalDisplacements;

		/// <inheritdoc/>
		public override Vector<double> LocalForces => new[] { -_N1, _N1 - _N3, _N3 }.ToVector();

		/// <inheritdoc/>
		public override double[] CrackOpenings => Strains.Select(eps => CrackOpening(Reinforcement, eps)).ToArray();

		/// <param name="width">The stringer width, in <paramref name="unit"/> considered.</param>
		/// <param name="height">The stringer height, in <paramref name="unit"/> considered.</param>
		/// <param name="unit">The <see cref="LengthUnit"/> of <paramref name="width"/> and <paramref name="height"/>.
		/// <para>Default: <seealso cref="LengthUnit.Millimeter"/>.</para></param>
		/// <inheritdoc cref="NLStringer(Node, Node, Node, Length, Length, Parameters, ConstitutiveModel, UniaxialReinforcement)"/>
		public NLStringer(Node grip1, Node grip2, Node grip3, double width, double height, Parameters concreteParameters, ConstitutiveModel model, UniaxialReinforcement reinforcement = null, LengthUnit unit = LengthUnit.Millimeter)
			: this(grip1, grip2, grip3, Length.From(width, unit), Length.From(height, unit), concreteParameters, model, reinforcement)
		{
		}

		/// <summary>
		/// Nonlinear stringer object
		/// </summary>
		/// <inheritdoc/>
		public NLStringer(Node grip1, Node grip2, Node grip3, Length width, Length height, Parameters concreteParameters, ConstitutiveModel model, UniaxialReinforcement reinforcement = null)
			: base(grip1, grip2, grip3, width, height, concreteParameters, model, reinforcement)
		{
		}

		/// <inheritdoc cref="NLStringer(Node, Node, Node, Length, Length, Parameters, ConstitutiveModel, UniaxialReinforcement)"/>
		/// <inheritdoc/>
		public NLStringer(IEnumerable<Node> nodes, Point grip1Position, Point grip3Position, double width, double height, Parameters concreteParameters, ConstitutiveModel model, UniaxialReinforcement reinforcement = null, LengthUnit unit = LengthUnit.Millimeter)
			: this(nodes, grip1Position, grip3Position, Length.From(width, unit), Length.From(height, unit), concreteParameters, model, reinforcement)
		{
		}

		/// <inheritdoc cref="NLStringer(Node, Node, Node, Length, Length, Parameters, ConstitutiveModel, UniaxialReinforcement)"/>
		/// <inheritdoc/>
		public NLStringer(IEnumerable<Node> nodes, Point grip1Position, Point grip3Position, Length width, Length height, Parameters concreteParameters, ConstitutiveModel model, UniaxialReinforcement reinforcement = null)
			: base(nodes, grip1Position, grip3Position, width, height, concreteParameters, model, reinforcement)
		{
		}

		/// <summary>
		/// Calculate the crack spacing at <paramref name="reinforcement"/> (in mm), according to Kaklauskas (2019) expression.
		/// <para>sm = 21 mm + 0.155 phi / rho</para>
		/// </summary>
		/// <param name="reinforcement">The <see cref="UniaxialReinforcement"/>.</param>
		public static double CrackSpacing(UniaxialReinforcement reinforcement) =>
			reinforcement is null || reinforcement.BarDiameter.ApproxZero() || reinforcement.Ratio.ApproxZero()
				? 21
				: 21 + 0.155 * reinforcement.BarDiameter / reinforcement.Ratio;

		/// <summary>
		/// Calculate the average crack opening, in mm.
		/// </summary>
		/// <param name="reinforcement">The <see cref="UniaxialReinforcement"/>.</param>
		/// <param name="strain">The strain.</param>
		public static double CrackOpening(UniaxialReinforcement reinforcement, double strain) => strain <0 || strain.ApproxZero(1E-9) ? 0 : strain  * CrackSpacing(reinforcement);

		/// <summary>
		/// Calculate B Matrix.
		/// </summary>
		private Matrix<double> CalculateBMatrix()
		{
			_BMatrix = 1 / Geometry.Length * new double[,]
			{
				{-3,  4, -1},
				{-1,  0,  1},
				{ 1, -4,  3}
			}.ToMatrix();

			return _BMatrix;
		}

		public override void Analysis(Vector<double> globalDisplacements = null)
		{
			// Set displacements
			if (globalDisplacements != null)
				SetDisplacements(globalDisplacements);

			// Get strains
			var eps = Strains;

			// Calculate normal forces
			_N1 = Force(eps[0]);
			_N3 = Force(eps[2]);
		}

		/// <summary>
		/// Calculate force based on strain.
		/// </summary>
		/// <param name="strain">Current strain.</param>
		private double Force(double strain) => strain.ApproxZero(1E-6) ? 0 : Concrete.CalculateForce(strain, Reinforcement) + (Reinforcement?.CalculateForce(strain) ?? 0);
	}
}

