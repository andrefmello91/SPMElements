using System.Collections.Generic;
using System.Linq;
using Extensions;
using Material.Concrete;
using Material.Reinforcement.Uniaxial;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;
using OnPlaneComponents;
using SPM.Elements.StringerProperties;
using UnitsNet;
using UnitsNet.Units;
using static OnPlaneComponents.Point;
using Force = UnitsNet.Force;

#nullable enable

namespace SPM.Elements
{
	/// <summary>
	///     Nonlinear stringer class.
	/// </summary>
	public class NLStringer : Stringer
	{
		#region Fields

		// Auxiliary fields
		private Matrix<double> _BMatrix;
		private Force _N1, _N3;

		#endregion

		#region Properties

		/// <summary>
		///     Get B <see cref="Matrix" /> to transform displacements in strains.
		/// </summary>
		private Matrix<double> BMatrix => _BMatrix ?? CalculateBMatrix();

		protected override Area ConcreteArea => Geometry.Area - (Reinforcement?.Area ?? Area.Zero);

		/// <inheritdoc />
		public override Length[] CrackOpenings => Strains.Select(eps => CrackOpening(Reinforcement, eps)).ToArray();

		/// <inheritdoc />
		public override Vector<double> LocalForces => new[] { -_N1.Newtons, _N1.Newtons - _N3.Newtons, _N3.Newtons }.ToVector();

		/// <summary>
		///     Get the strain <see cref="Vector" />
		/// </summary>
		private Vector<double> Strains => BMatrix * LocalDisplacements;

		#endregion

		#region Constructors

		/// <param name="unit">
		///     The <see cref="LengthUnit" /> of <paramref name="width" /> and <paramref name="height" />.
		///     <para>Default: <seealso cref="LengthUnit.Millimeter" />.</para>
		/// </param>
		/// <inheritdoc cref="NLStringer(Node, Node, Node, Length, Length, Parameters, ConstitutiveModel, UniaxialReinforcement)" />
		public NLStringer(Node grip1, Node grip2, Node grip3, double width, double height, Parameters concreteParameters, ConstitutiveModel model = ConstitutiveModel.MCFT, UniaxialReinforcement? reinforcement = null, LengthUnit unit = LengthUnit.Millimeter)
			: this(grip1, grip2, grip3, Length.From(width, unit), Length.From(height, unit), concreteParameters, model, reinforcement)
		{
		}

		/// <summary>
		///     Nonlinear stringer object
		/// </summary>
		/// <inheritdoc />
		public NLStringer(Node grip1, Node grip2, Node grip3, Length width, Length height, Parameters concreteParameters, ConstitutiveModel model = ConstitutiveModel.MCFT, UniaxialReinforcement? reinforcement = null)
			: base(grip1, grip2, grip3, width, height, concreteParameters, model, reinforcement)
		{
		}

		/// <inheritdoc cref="NLStringer(Node, Node, Node, Length, Length, Parameters, ConstitutiveModel, UniaxialReinforcement)" />
		/// <inheritdoc />
		public NLStringer(IEnumerable<Node> nodes, Point grip1Position, Point grip3Position, double width, double height, Parameters concreteParameters, ConstitutiveModel model = ConstitutiveModel.MCFT, UniaxialReinforcement? reinforcement = null, LengthUnit unit = LengthUnit.Millimeter)
			: this(nodes, grip1Position, grip3Position, Length.From(width, unit), Length.From(height, unit), concreteParameters, model, reinforcement)
		{
		}

		/// <inheritdoc cref="NLStringer(Node, Node, Node, Length, Length, Parameters, ConstitutiveModel, UniaxialReinforcement)" />
		/// <inheritdoc />
		public NLStringer(IEnumerable<Node> nodes, Point grip1Position, Point grip3Position, Length width, Length height, Parameters concreteParameters, ConstitutiveModel model = ConstitutiveModel.MCFT, UniaxialReinforcement? reinforcement = null)
			: base(nodes, grip1Position, grip3Position, width, height, concreteParameters, model, reinforcement)
		{
		}

		/// <inheritdoc cref="NLStringer(Node, Node, Node, Length, Length, Parameters, ConstitutiveModel, UniaxialReinforcement)" />
		/// <inheritdoc />
		public NLStringer(IEnumerable<Node> nodes, StringerGeometry geometry, Parameters concreteParameters, ConstitutiveModel model = ConstitutiveModel.MCFT, UniaxialReinforcement? reinforcement = null)
			: base(nodes, geometry, concreteParameters, model, reinforcement)
		{
		}

		#endregion

		#region Methods

		/// <summary>
		///     Calculate the crack spacing at <paramref name="reinforcement" />, according to Kaklauskas (2019)
		///     expression.
		///     <para>sm = 21 mm + 0.155 phi / rho</para>
		/// </summary>
		/// <param name="reinforcement">The <see cref="UniaxialReinforcement" />.</param>
		public static Length CrackSpacing(UniaxialReinforcement? reinforcement) =>
			reinforcement is null || reinforcement.BarDiameter.ApproxZero(Tolerance) || reinforcement.Ratio.ApproxZero()
				? Length.FromMillimeters(21)
				: Length.FromMillimeters(21) + 0.155 * reinforcement.BarDiameter / reinforcement.Ratio;

		/// <summary>
		///     Calculate the average crack opening.
		/// </summary>
		/// <param name="reinforcement">The <see cref="UniaxialReinforcement" />.</param>
		/// <param name="strain">The strain.</param>
		public static Length CrackOpening(UniaxialReinforcement? reinforcement, double strain) => strain < 0 || strain.ApproxZero(1E-9) ? Length.Zero : strain  * CrackSpacing(reinforcement);

		public override void Analysis(Vector<double>? globalDisplacements = null)
		{
			// Set displacements
			if (globalDisplacements != null)
				SetDisplacements(globalDisplacements);

			// Get strains
			var eps = Strains;

			// Calculate normal forces
			_N1 = CalculateForce(eps[0]);
			_N3 = CalculateForce(eps[2]);
		}

		/// <summary>
		///     Calculate B Matrix.
		/// </summary>
		private Matrix<double> CalculateBMatrix()
		{
			_BMatrix = 1 / Geometry.Length.Millimeters * new double[,]
			{
				{-3,  4, -1},
				{-1,  0,  1},
				{ 1, -4,  3}
			}.ToMatrix();

			return _BMatrix;
		}

		/// <summary>
		///     Calculate force based on strain.
		/// </summary>
		/// <param name="strain">Current strain.</param>
		private Force CalculateForce(double strain) =>
			strain.ApproxZero(1E-9)
				? Force.Zero
				: Concrete.CalculateForce(strain, Reinforcement) + (Reinforcement?.CalculateForce(strain) ?? Force.Zero);

		#endregion
	}
}