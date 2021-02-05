using System.Collections.Generic;
using System.Linq;
using Extensions;
using Material.Concrete;
using Material.Reinforcement.Biaxial;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;
using OnPlaneComponents;
using RCMembrane;
using SPM.Elements.PanelProperties;
using UnitsNet;
using UnitsNet.Units;

#nullable enable

namespace SPM.Elements
{
	public class NLPanel : Panel
	{
		#region Fields

		// Auxiliary fields
		private Matrix<double> _BA;

		#endregion

		#region Properties

		public override StressState AverageStresses
		{
			get
			{
				// Get stress vector
				var sigma = Stresses;

				// Calculate average stresses
				var avg = new double[3];

				for (var i = 0; i < 3; i++)
					avg[i] = 0.25 * (sigma[i] + sigma[i + 3] + sigma[i + 6] + sigma[i + 9]);

				return new StressState(avg[0], avg[1], avg[2]);
			}
		}

		public override PrincipalStrainState ConcretePrincipalStrains
		{
			get
			{
				var eps = StrainState.Zero;

				for (var i = 0; i < 4; i++)
					eps += IntegrationPoints[i].Concrete.Strains;

				// Calculate average
				eps = 0.25 * eps;

				// Return principal
				return PrincipalStrainState.FromStrain(eps);
			}
		}

		public override PrincipalStressState ConcretePrincipalStresses
		{
			get
			{
				var sigma = StressState.Zero;

				for (var i = 0; i < 4; i++)
					sigma += IntegrationPoints[i].Concrete.Stresses;

				// Calculate average
				sigma = 0.25 * sigma;

				// Return principal
				return PrincipalStressState.FromStress(sigma);
			}
		}

		/// <summary>
		///     Get/set panel concrete stress <see cref="Vector" />.
		/// </summary>
		/// <inheritdoc cref="Stresses" />
		public Vector<double> ConcreteStresses { get; private set; }

		public override Length CrackOpening => Membrane.CrackOpening(Reinforcement, ConcretePrincipalStrains);

		/// <inheritdoc />
		public override Vector<double> Forces => GlobalForces;

		/// <summary>
		///     Get <see cref="Membrane" /> integration points.
		/// </summary>
		public Membrane[] IntegrationPoints  { get; }

		/// <summary>
		///     Get/set panel reinforcement stress <see cref="Vector" />.
		/// </summary>
		/// <inheritdoc cref="Stresses" />
		public Vector<double> ReinforcementStresses { get; private set; }

		public override Matrix<double> Stiffness => InitialStiffness();

		/// <summary>
		///     Get panel strain <see cref="Vector" />.
		/// </summary>
		public Vector<double> StrainVector => (_BA ?? CalculateBA()) * Displacements;

		/// <summary>
		///     Get panel stress <see cref="Vector" />.
		/// </summary>
		/// <remarks>
		///     Components in <see cref="PressureUnit.Megapascal" />.
		/// </remarks>
		public Vector<double> Stresses => ConcreteStresses + ReinforcementStresses;

		#endregion

		#region Constructors

		/// <summary>
		///     Nonlinear panel object.
		/// </summary>
		/// <inheritdoc />
		public NLPanel(Node grip1, Node grip2, Node grip3, Node grip4, Vertices vertices, double width, Parameters concreteParameters, ConstitutiveModel model = ConstitutiveModel.MCFT, WebReinforcement? reinforcement = null, LengthUnit unit = LengthUnit.Millimeter)
			: this(grip1, grip2, grip3, grip4, vertices, Length.From(width, unit), concreteParameters, model, reinforcement)
		{
		}

		/// <summary>
		///     Nonlinear panel object.
		/// </summary>
		/// <inheritdoc />
		public NLPanel(Node grip1, Node grip2, Node grip3, Node grip4, Vertices vertices, Length width, Parameters concreteParameters, ConstitutiveModel model = ConstitutiveModel.MCFT, WebReinforcement? reinforcement = null)
			: this(grip1, grip2, grip3, grip4, new PanelGeometry(vertices, width), concreteParameters, model, reinforcement)
		{
		}

		/// <summary>
		///     Nonlinear panel object.
		/// </summary>
		/// <inheritdoc />
		public NLPanel(Node grip1, Node grip2, Node grip3, Node grip4, PanelGeometry geometry, Parameters concreteParameters, ConstitutiveModel model = ConstitutiveModel.MCFT, WebReinforcement? reinforcement = null)
			: base(grip1, grip2, grip3, grip4, geometry, concreteParameters, model, reinforcement) => IntegrationPoints = IntPoints().ToArray();

		/// <summary>
		///     Nonlinear panel object.
		/// </summary>
		/// <inheritdoc />
		public NLPanel(IEnumerable<Node> nodes, Vertices vertices, double width, Parameters concreteParameters, ConstitutiveModel model = ConstitutiveModel.MCFT, WebReinforcement? reinforcement = null, LengthUnit unit = LengthUnit.Millimeter)
			: this(nodes, vertices, Length.From(width, unit), concreteParameters, model, reinforcement)
		{
		}

		/// <summary>
		///     Nonlinear panel object.
		/// </summary>
		/// <inheritdoc />
		public NLPanel(IEnumerable<Node> nodes, Vertices vertices, Length width, Parameters concreteParameters, ConstitutiveModel model = ConstitutiveModel.MCFT, WebReinforcement? reinforcement = null)
			: this(nodes, new PanelGeometry(vertices, width), concreteParameters, model, reinforcement)
		{
		}

		/// <summary>
		///     Nonlinear panel object.
		/// </summary>
		/// <inheritdoc />
		public NLPanel(IEnumerable<Node> nodes, PanelGeometry geometry, Parameters concreteParameters, ConstitutiveModel model = ConstitutiveModel.MCFT, WebReinforcement? reinforcement = null)
			: base(nodes, geometry, concreteParameters, model, reinforcement) => IntegrationPoints = IntPoints().ToArray();

		#endregion

		#region  Methods

		/// <summary>
		///     Set displacements and calculate forces.
		/// </summary>
		/// <param name="globalDisplacements">The global displacement <see cref="Vector" />.</param>
		public override void Analysis(Vector<double>? globalDisplacements = null)
		{
			// Set displacements
			if (globalDisplacements != null)
				SetDisplacements(globalDisplacements);

			// Calculate stresses, forces and update stiffness
			CalculateStresses();
			CalculateForces();
		}

		/// <summary>
		///     Initiate <see cref="Membrane" /> integration points.
		/// </summary>
		private IEnumerable<Membrane> IntPoints()
		{
			for (var i = 0; i < 4; i++)
				yield return Membrane.Read(Concrete.Parameters, Reinforcement?.Clone(), Geometry.Width);
		}

		/// <summary>
		///     Calculate BA matrix.
		/// </summary>
		private Matrix<double> CalculateBA()
		{
			var (a, b, c, d) = Geometry.DimensionsInMillimeters();

			// Calculate t1, t2 and t3
			double
				t1 = a * b - c * d,
				t2 = 0.5 * (a * a - c * c) + b * b - d * d,
				t3 = 0.5 * (b * b - d * d) + a * a - c * c;

			// Calculate the components of A matrix
			double
				a_t1 = a / t1,
				b_t1 = b / t1,
				c_t1 = c / t1,
				d_t1 = d / t1,
				a_t2 = a / t2,
				b_t3 = b / t3,
				a_2t1 = a_t1 / 2,
				b_2t1 = b_t1 / 2,
				c_2t1 = c_t1 / 2,
				d_2t1 = d_t1 / 2;

			// Create A matrix
			var A = new[,]
			{
				{   d_t1,     0,   b_t1,     0, -d_t1,      0, -b_t1,      0 },
				{      0, -a_t1,      0, -c_t1,     0,   a_t1,     0,   c_t1 },
				{ -a_2t1, d_2t1, -c_2t1, b_2t1, a_2t1, -d_2t1, c_2t1, -b_2t1 },
				{ - a_t2,     0,   a_t2,     0, -a_t2,      0,  a_t2,      0 },
				{      0,  b_t3,      0, -b_t3,     0,   b_t3,     0,  -b_t3 }
			}.ToMatrix();

			// Calculate the components of B matrix
			double
				c_a = c / a,
				d_b = d / b;

			// Create B matrix
			var B = new[,]
			{
				{1, 0, 0, -c_a,    0 },
				{0, 1, 0,    0,   -1 },
				{0, 0, 2,    0,    0 },
				{1, 0, 0,    1,    0 },
				{0, 1, 0,    0,  d_b },
				{0, 0, 2,    0,    0 },
				{1, 0, 0,  c_a,    0 },
				{0, 1, 0,    0,    1 },
				{0, 0, 2,    0,    0 },
				{1, 0, 0,   -1,    0 },
				{0, 1, 0,    0, -d_b },
				{0, 0, 2,    0,    0 }
			}.ToMatrix();

			_BA = B * A;

			return _BA;
		}

		/// <summary>
		///     Calculate Q matrix.
		/// </summary>
		private Matrix<double> CalculateQ()
		{
			// Get dimensions
			var (a, b, c, d) = Geometry.DimensionsInMillimeters();

			// Calculate t4
			var t4 = a * a + b * b;

			// Calculate the components of Q matrix
			double
				a2     = a * a,
				bc     = b * c,
				bdMt4  = b * d - t4,
				ab     = a * b,
				MbdMt4 = -b * d - t4,
				Tt4    = 2 * t4,
				acMt4  = a * c - t4,
				ad     = a * d,
				b2     = b * b,
				MacMt4 = -a * c - t4;

			// Create Q matrix
			return
				1.0 / Tt4 * new[,]
				{
					{  a2,     bc,  bdMt4, -ab, -a2,    -bc, MbdMt4,  ab },
					{   0,    Tt4,      0,   0,   0,      0,      0,   0 },
					{   0,      0,    Tt4,   0,   0,      0,      0,   0 },
					{ -ab,  acMt4,     ad,  b2,  ab, MacMt4,    -ad, -b2 },
					{ -a2,    -bc, MbdMt4,  ab,  a2,     bc,  bdMt4, -ab },
					{   0,      0,      0,   0,   0,    Tt4,      0,   0 },
					{   0,      0,      0,   0,   0,      0,    Tt4,   0 },
					{  ab, MacMt4,    -ad, -b2, -ab,  acMt4,     ad,  b2 }
				}.ToMatrix();
		}

		/// <summary>
		///     Calculate P matrices for concrete and steel
		/// </summary>
		private (Matrix<double> Pc, Matrix<double> Ps) CalculateP()
		{
			// Get dimensions
			double[]
				x = Geometry.Vertices.XCoordinates.Select(cx => cx.Millimeters).ToArray(),
				y = Geometry.Vertices.YCoordinates.Select(cy => cy.Millimeters).ToArray(),
				c = Geometry.StringerDimensions.Select(s => s.Millimeters).ToArray();

			var t = Geometry.Width.Millimeters;

			// Create P matrices
			var Pc = Matrix<double>.Build.Dense(8, 12);
			var Ps = Matrix<double>.Build.Dense(8, 12);

			// Calculate the components of Pc
			Pc[0, 0] = Pc[1, 2] = t * (y[1] - y[0]);
			Pc[0, 2] = t * (x[0] - x[1]);
			Pc[1, 1] = t * (x[0] - x[1] + c[1] + c[3]);

			Pc[2, 3] = t * (y[2] - y[1] - c[2] - c[0]);
			Pc[2, 5] = Pc[3, 4] = t * (x[1] - x[2]);
			Pc[3, 5] = t * (y[2] - y[1]);

			Pc[4, 6] = Pc[5, 8] = t * (y[3] - y[2]);
			Pc[4, 8] = t * (x[2] - x[3]);
			Pc[5, 7] = t * (x[2] - x[3] - c[1] - c[3]);

			Pc[6, 9] = t * (y[0] - y[3] + c[0] + c[2]);
			Pc[6, 11] = Pc[7, 10] = t * (x[3] - x[0]);
			Pc[7, 11] = t * (y[0] - y[3]);

			// Calculate the components of Ps
			Ps[0, 0] = Pc[0, 0];
			Ps[1, 1] = t * (x[0] - x[1]);

			Ps[2, 3] = t * (y[2] - y[1]);
			Ps[3, 4] = Pc[3, 4];

			Ps[4, 6] = Pc[4, 6];
			Ps[5, 7] = t * (x[2] - x[3]);

			Ps[6, 9] = t * (y[0] - y[3]);
			Ps[7, 10] = Pc[7, 10];

			return
				(Pc, Ps);
		}

		/// <summary>
		///     Calculate and set stress vectors.
		///     <para>See: <see cref="ConcreteStresses" />, <see cref="ReinforcementStresses" />.</para>
		/// </summary>
		private void CalculateStresses()
		{
			// Get the vector strains and stresses
			var ev = StrainVector;

			// Calculate stresses based on strains
			for (var i = 0; i < 4; i++)
			{
				// Get the strains and stresses
				var e = StrainState.FromVector(ev.SubVector(3 * i, 3));

				// Calculate stresses
				IntegrationPoints[i].Calculate(e);
			}

			// Update vectors
			ConcreteStresses = Vector<double>.Build.Dense(12);
			ReinforcementStresses = Vector<double>.Build.Dense(12);

			for (var i = 0; i < 4; i++)
			{
				// Get the stiffness
				var sigC = IntegrationPoints[i].Concrete.Stresses;
				var sigS = IntegrationPoints[i].Reinforcement?.Stresses ?? StressState.Zero;

				// Set to stiffness
				ConcreteStresses.SetSubVector(3 * i, 3, sigC.AsVector());
				ReinforcementStresses.SetSubVector(3 * i, 3, sigS.AsVector());
			}
		}

		/// <summary>
		///     Calculate and set global force vector.
		///     <para>See: <see cref="Panel.Forces" />.</para>
		/// </summary>
		private void CalculateForces()
		{
			double t0, t1, t2, t3, t4;

			// Get dimensions
			double[]
				x = Geometry.Vertices.XCoordinates.Select(cx => cx.Millimeters).ToArray(),
				y = Geometry.Vertices.YCoordinates.Select(cy => cy.Millimeters).ToArray();

			var (a, b, c, d) = Geometry.DimensionsInMillimeters();
			var s            = Geometry.StringerDimensions.Select(st => st.Millimeters).ToArray();
			;
			var t            = Geometry.Width.Millimeters;

			// Get stresses
			Vector<double>
				sigC = ConcreteStresses,
				sigS = ReinforcementStresses,
				sig  = Stresses;

			var (sig1, sigC1, sigS1) = (sig.SubVector(0, 3), sigC.SubVector(0, 3), sigS.SubVector(0, 3));
			var (sig2, sigC2, sigS2) = (sig.SubVector(3, 3), sigC.SubVector(3, 3), sigS.SubVector(3, 3));
			var (sig3, sigC3, sigS3) = (sig.SubVector(6, 3), sigC.SubVector(6, 3), sigS.SubVector(6, 3));
			var (sig4, sigC4, sigS4) = (sig.SubVector(9, 3), sigC.SubVector(9, 3), sigS.SubVector(9, 3));

			// Calculate forces
			t1 = y[1] - y[0];
			t2 = x[1] - x[0];
			t3 = CheckT3(t2 - s[1] - s[3]);

			double
				f1 = ( sig1 [0] * t1 - sig1 [2] * t2) * t,
				f2 = (-sigC1[1] * t3 - sigS1[1] * t2 + sig1[2] * t1) * t;

			t1 = y[2] - y[1];
			t2 = x[2] - x[1];
			t3 = CheckT3(t1 - s[2] - s[0]);

			double
				f3 = ( sigC2[0] * t3 + sigS2[0] * t1 - sig2[2] * t2) * t,
				f4 = (-sig2 [1] * t2 + sig2 [2] * t1) * t;

			t1 = y[2] - y[3];
			t2 = x[2] - x[3];
			t3 = CheckT3(t2 - s[1] - s[3]);

			double
				f5 = (-sig3 [0] * t1 + sig3 [2] * t2) * t,
				f6 = ( sigC3[1] * t3 + sigS3[1] * t2 - sig3[2] * t1) * t;

			t1 = y[3] - y[0];
			t2 = x[3] - x[0];
			t3 = CheckT3(t1 - s[0] - s[2]);

			double
				f7 = (-sigC4[0] * t3 - sigS4[0] * t1 - sig4[2] * t2) * t,
				f8 = ( sig4 [1] * t2 - sig4 [2] * t1) * t;

			// Correct forces
			t0 = 2 * (a * a + b * b);
			t1 = (a * (f1 - f5) - b * (f4 - f8)) / t0;
			t2 = (c * (f2 - f6) + d * (f3 - f7)) / t0;
			t3 = (f3 + f7) * 0.5;
			t4 = (f2 + f6) * 0.5;

			f1 = a * t1 + b * t2 - t3;
			f4 = -b * t1 + a * t2 - t4;
			f5 = -a * t1 - b * t2 - t3;
			f8 = b * t1 - a * t2 - t4;

			GlobalForces =
				new []
				{
					f1, f2, f3, f4, f5, f6, f7, f8
				}.ToVector();

			// Check value of t3
			static double CheckT3(double value) => value < 0 ? 0 : value;
		}

		/// <summary>
		///     Calculate initial stiffness <see cref="Matrix<double>"/>.
		/// </summary>
		private Matrix<double> InitialStiffness()
		{
			var (Dc, Ds) = InitialMaterialStiffness();
			var (Pc, Ps) = CalculateP();

			Matrix<double>
				BA = CalculateBA(),
				Q  = CalculateQ();

			var QPs = Q * Ps;
			var QPc = Q * Pc;

			var kc = QPc * Dc * BA;
			var ks = QPs * Ds * BA;

			return kc + ks;
		}

		/// <summary>
		///     Calculate initial material stiffness matrices.
		/// </summary>
		private (Matrix<double> Dc, Matrix<double> Ds) InitialMaterialStiffness()
		{
			var Dc = Matrix<double>.Build.Dense(12, 12);
			var Ds = Matrix<double>.Build.Dense(12, 12);

			for (var i = 0; i < 4; i++)
			{
				// Get the stiffness
				var Dci = IntegrationPoints[i].Concrete.InitialStiffness;
				var Dsi = IntegrationPoints[i].Reinforcement?.InitialStiffness;

				// Set to stiffness
				Dc.SetSubMatrix(3 * i, 3 * i, Dci);

				if (Dsi is null)
					continue;

				Ds.SetSubMatrix(3 * i, 3 * i, Dsi);
			}

			return
				(Dc, Ds);
		}

		#endregion
	}
}