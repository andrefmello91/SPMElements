using System.Collections.Generic;
using System.Linq;
using andrefmello91.Extensions;
using andrefmello91.FEMAnalysis;
using andrefmello91.Material.Concrete;
using andrefmello91.Material.Reinforcement;
using andrefmello91.OnPlaneComponents;
using andrefmello91.ReinforcedConcreteMembrane;
using andrefmello91.SPMElements.PanelProperties;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;
using UnitsNet;
using UnitsNet.Units;
#nullable enable

namespace andrefmello91.SPMElements
{
	/// <summary>
	///     Nonlinear panel class.
	/// </summary>
	internal class NLPanel : Panel
	{

		#region Fields

		// Auxiliary fields
		private readonly Matrix<double> _baMatrix;

		#endregion

		#region Properties

		/// <inheritdoc />
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

		/// <inheritdoc />
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

		/// <inheritdoc />
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

		/// <inheritdoc />
		public override Length CrackOpening => Membrane.CrackOpening(Reinforcement, ConcretePrincipalStrains);

		/// <summary>
		///     Get <see cref="Membrane" /> integration points.
		/// </summary>
		private Membrane[] IntegrationPoints { get; }

		private Vector<double> ReinforcementStresses { get; set; }

		/// <summary>
		///     Get panel strain <see cref="Vector" />.
		/// </summary>
		private Vector<double> StrainVector => 
			_baMatrix * (Vector<double>) (Displacements.Unit == LengthUnit.Millimeter
				? Displacements 
				: Displacements.Convert(LengthUnit.Millimeter));

		/// <summary>
		///     Get panel stress <see cref="Vector" />.
		/// </summary>
		/// <remarks>
		///     Components in <see cref="PressureUnit.Megapascal" />.
		/// </remarks>
		private Vector<double> Stresses => ConcreteStresses + ReinforcementStresses;

		/// <summary>
		///		Check if concrete is cracked in this panel.
		/// </summary>
		/// <inheritdoc cref="Material.Concrete.Concrete.Cracked"/>
		public bool ConcreteCracked => IntegrationPoints.Any(m => m.Concrete.Cracked);

		#endregion

		#region Constructors

		/// <summary>
		///     Nonlinear panel object.
		/// </summary>
		/// <inheritdoc />
		internal NLPanel(Node grip1, Node grip2, Node grip3, Node grip4, PanelGeometry geometry, IConcreteParameters concreteParameters, ConstitutiveModel model = ConstitutiveModel.MCFT, WebReinforcement? reinforcement = null)
			: base(grip1, grip2, grip3, grip4, geometry)
		{
			Concrete = BiaxialConcrete.From(concreteParameters, model);

			Reinforcement = reinforcement;

			if (Reinforcement is not null)
				Reinforcement.Width = Geometry.Width;

			IntegrationPoints = IntPoints(concreteParameters, reinforcement, geometry.Width, model).ToArray();
			_baMatrix         = CalculateBa(Geometry);

			InitiateStiffness();
		}

		#endregion

		#region Methods

		/// <summary>
		///     Calculate BA matrix.
		/// </summary>
		/// <param name="geometry">The <see cref="PanelGeometry" />.</param>
		private static Matrix<double> CalculateBa(PanelGeometry geometry)
		{
			var (a, b, c, d) = geometry.DimensionsInMillimeters();

			// Calculate t1, t2 and t3
			double
				t1 = a * b - c * d,
				t2 = 0.5 * (a * a - c * c) + b * b - d * d,
				t3 = 0.5 * (b * b - d * d) + a * a - c * c;

			// Calculate the components of A matrix
			double
				a_t1  = a / t1,
				b_t1  = b / t1,
				c_t1  = c / t1,
				d_t1  = d / t1,
				a_t2  = a / t2,
				b_t3  = b / t3,
				a_2t1 = a_t1 / 2,
				b_2t1 = b_t1 / 2,
				c_2t1 = c_t1 / 2,
				d_2t1 = d_t1 / 2;

			// Create A matrix
			var A = new[,]
			{
				{ d_t1, 0, b_t1, 0, -d_t1, 0, -b_t1, 0 },
				{ 0, -a_t1, 0, -c_t1, 0, a_t1, 0, c_t1 },
				{ -a_2t1, d_2t1, -c_2t1, b_2t1, a_2t1, -d_2t1, c_2t1, -b_2t1 },
				{ -a_t2, 0, a_t2, 0, -a_t2, 0, a_t2, 0 },
				{ 0, b_t3, 0, -b_t3, 0, b_t3, 0, -b_t3 }
			}.ToMatrix();

			// Calculate the components of B matrix
			double
				c_a = c / a,
				d_b = d / b;

			// Create B matrix
			var B = new[,]
			{
				{ 1, 0, 0, -c_a, 0 },
				{ 0, 1, 0, 0, -1 },
				{ 0, 0, 2, 0, 0 },
				{ 1, 0, 0, 1, 0 },
				{ 0, 1, 0, 0, d_b },
				{ 0, 0, 2, 0, 0 },
				{ 1, 0, 0, c_a, 0 },
				{ 0, 1, 0, 0, 1 },
				{ 0, 0, 2, 0, 0 },
				{ 1, 0, 0, -1, 0 },
				{ 0, 1, 0, 0, -d_b },
				{ 0, 0, 2, 0, 0 }
			}.ToMatrix();

			return
				B * A;
		}

		/// <summary>
		///     Calculate P matrices for concrete and steel
		/// </summary>
		/// <inheritdoc cref="CalculateBa" />
		private static (Matrix<double> Pc, Matrix<double> Ps) CalculateP(PanelGeometry geometry)
		{
			// Get dimensions
			double[]
				x = geometry.Vertices.XCoordinates.Select(cx => cx.Millimeters).ToArray(),
				y = geometry.Vertices.YCoordinates.Select(cy => cy.Millimeters).ToArray(),
				c = geometry.StringerDimensions.Select(s => s.Millimeters).ToArray();

			var t = geometry.Width.Millimeters;

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

			Pc[6, 9]  = t * (y[0] - y[3] + c[0] + c[2]);
			Pc[6, 11] = Pc[7, 10] = t * (x[3] - x[0]);
			Pc[7, 11] = t * (y[0] - y[3]);

			// Calculate the components of Ps
			Ps[0, 0] = Pc[0, 0];
			Ps[1, 1] = t * (x[0] - x[1]);

			Ps[2, 3] = t * (y[2] - y[1]);
			Ps[3, 4] = Pc[3, 4];

			Ps[4, 6] = Pc[4, 6];
			Ps[5, 7] = t * (x[2] - x[3]);

			Ps[6, 9]  = t * (y[0] - y[3]);
			Ps[7, 10] = Pc[7, 10];

			return
				(Pc, Ps);
		}

		/// <summary>
		///     Calculate Q matrix.
		/// </summary>
		/// <inheritdoc cref="CalculateBa" />
		private static Matrix<double> CalculateQ(PanelGeometry geometry)
		{
			// Get dimensions
			var (a, b, c, d) = geometry.DimensionsInMillimeters();

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
					{ a2, bc, bdMt4, -ab, -a2, -bc, MbdMt4, ab },
					{ 0, Tt4, 0, 0, 0, 0, 0, 0 },
					{ 0, 0, Tt4, 0, 0, 0, 0, 0 },
					{ -ab, acMt4, ad, b2, ab, MacMt4, -ad, -b2 },
					{ -a2, -bc, MbdMt4, ab, a2, bc, bdMt4, -ab },
					{ 0, 0, 0, 0, 0, Tt4, 0, 0 },
					{ 0, 0, 0, 0, 0, 0, Tt4, 0 },
					{ ab, MacMt4, -ad, -b2, -ab, acMt4, ad, b2 }
				}.ToMatrix();
		}

		/// <summary>
		///     Calculate initial material stiffness matrices.
		/// </summary>
		/// <param name="integrationPoints">The <see cref="Membrane" /> integration points of the panel.</param>
		private static (Matrix<double> Dc, Matrix<double> Ds) InitialMaterialStiffness(IReadOnlyList<Membrane> integrationPoints)
		{
			var Dc = Matrix<double>.Build.Dense(12, 12);
			var Ds = Matrix<double>.Build.Dense(12, 12);

			for (var i = 0; i < 4; i++)
			{
				// Get the stiffness
				var Dci = integrationPoints[i].Concrete.InitialStiffness;
				var Dsi = integrationPoints[i].Reinforcement?.InitialStiffness;

				// Set to stiffness
				Dc.SetSubMatrix(3 * i, 3 * i, Dci);

				if (Dsi is null)
					continue;

				Ds.SetSubMatrix(3 * i, 3 * i, Dsi);
			}

			return
				(Dc, Ds);
		}

		/// <summary>
		///     Initiate <see cref="Membrane" /> integration points.
		/// </summary>
		private static IEnumerable<Membrane> IntPoints(IConcreteParameters concreteParameters, WebReinforcement? reinforcement, Length width, ConstitutiveModel model)
		{
			for (var i = 0; i < 4; i++)
				yield return Membrane.From(concreteParameters, reinforcement?.Clone(), width, model);
		}

		/// <inheritdoc />
		public override void CalculateForces()
		{
			// Calculate stresses, forces and update stiffness
			CalculateStresses();
			CalculateGripForces();
		}

		/// <inheritdoc />
		public override void UpdateDisplacements() =>
			Displacements = this.GetDisplacementsFromGrips();

		/// <summary>
		///     Calculate and set global force vector.
		///     <para>See: <see cref="Panel.Forces" />.</para>
		/// </summary>
		private void CalculateGripForces()
		{
			double t0, t1, t2, t3, t4;

			// Get dimensions
			double[]
				x = Geometry.Vertices.XCoordinates.Select(cx => cx.Millimeters).ToArray(),
				y = Geometry.Vertices.YCoordinates.Select(cy => cy.Millimeters).ToArray();

			var (a, b, c, d) = Geometry.DimensionsInMillimeters();
			var s = Geometry.StringerDimensions.Select(st => st.Millimeters).ToArray();
			;
			var t = Geometry.Width.Millimeters;

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
				f1 = (sig1[0] * t1 - sig1[2] * t2) * t,
				f2 = (-sigC1[1] * t3 - sigS1[1] * t2 + sig1[2] * t1) * t;

			t1 = y[2] - y[1];
			t2 = x[2] - x[1];
			t3 = CheckT3(t1 - s[2] - s[0]);

			double
				f3 = (sigC2[0] * t3 + sigS2[0] * t1 - sig2[2] * t2) * t,
				f4 = (-sig2[1] * t2 + sig2[2] * t1) * t;

			t1 = y[2] - y[3];
			t2 = x[2] - x[3];
			t3 = CheckT3(t2 - s[1] - s[3]);

			double
				f5 = (-sig3[0] * t1 + sig3[2] * t2) * t,
				f6 = (sigC3[1] * t3 + sigS3[1] * t2 - sig3[2] * t1) * t;

			t1 = y[3] - y[0];
			t2 = x[3] - x[0];
			t3 = CheckT3(t1 - s[0] - s[2]);

			double
				f7 = (-sigC4[0] * t3 - sigS4[0] * t1 - sig4[2] * t2) * t,
				f8 = (sig4[1] * t2 - sig4[2] * t1) * t;

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

			Forces = new ForceVector(new[]
				{
					f1, f2, f3, f4, f5, f6, f7, f8
				});

			// Check value of t3
			static double CheckT3(double value) => value < 0 ? 0 : value;
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
			ConcreteStresses      = Vector<double>.Build.Dense(12);
			ReinforcementStresses = Vector<double>.Build.Dense(12);

			for (var i = 0; i < 4; i++)
			{
				// Get the stiffness
				var sigC = IntegrationPoints[i].Concrete.Stresses.ToHorizontal();
				var sigS = IntegrationPoints[i].Reinforcement?.Stresses.ToHorizontal() ?? StressState.Zero;

				// Set to stiffness
				ConcreteStresses.SetSubVector(3 * i, 3, sigC.AsVector());
				ReinforcementStresses.SetSubVector(3 * i, 3, sigS.AsVector());
			}
		}

		/// <summary>
		///     Calculate initial stiffness matrix.
		/// </summary>
		private void InitiateStiffness()
		{
			var (Dc, Ds) = InitialMaterialStiffness(IntegrationPoints);
			var (Pc, Ps) = CalculateP(Geometry);

			var Q = CalculateQ(Geometry);

			var QPs = Q * Ps;
			var QPc = Q * Pc;

			var kc = QPc * Dc * _baMatrix;
			var ks = QPs * Ds * _baMatrix;

			Stiffness = new StiffnessMatrix(kc + ks);
		}

		#endregion

	}
}