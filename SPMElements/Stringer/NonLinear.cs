using System;
using Autodesk.AutoCAD.DatabaseServices;
using Material.Concrete;
using Material.Reinforcement;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;
using UnitsNet;
using UnitsNet.Units;

namespace SPMElements
{
	public partial class NonLinearStringer : Stringer
	{
		// Auxiliary fields
		private Matrix<double> _FMatrix;
		private (double e1, double e3) _genStrains,  _iterationGenStrains;
		private (double N1, double N3) _genStresses, _iterationGenStresses;

		private readonly Matrix<double> _BMatrix = 
			Matrix<double>.Build.DenseOfArray(new double[,]
			{
				{ -1,  1, 0},
				{  0, -1, 1}
			});

        /// <summary>
        /// Get integration points.
        /// </summary>
        private IntegrationPoint[] IntPoints { get; }

        /// <summary>
        /// Get <see cref="StressStrainRelations"/>.
        /// </summary>
        private StressStrainRelations Relations { get; }

		/// <summary>
        /// Get <see cref="Steel"/> of <see cref="Stringer.Reinforcement"/>.
        /// </summary>
		private Steel Steel => Reinforcement?.Steel;

		/// <inheritdoc/>
		public override Matrix<double> LocalStiffness => _BMatrix.Transpose() * _FMatrix.Inverse() * _BMatrix;

		/// <inheritdoc/>
		public override Vector<double> LocalForces
		{
			get
			{
				var (N1, N3) = _genStresses;

				return
					Vector<double>.Build.DenseOfArray(new[]
					{
						-N1, N1 - N3, N3
					});
			}
		}

        /// <summary>
        /// Get local force <see cref="Vector"/> calculated at each iteration.
        /// </summary>
        public Vector<double> IterationLocalForces
		{
			get
			{
				var (N1, N3) = _iterationGenStresses;

				return
					Vector<double>.Build.DenseOfArray(new[]
					{
						-N1, N1 - N3, N3
					});
			}
		}

        /// <summary>
        /// Get global force <see cref="Vector"/> calculated at each iteration.
        /// </summary>
		public Vector<double> IterationGlobalForces => TransformationMatrix.Transpose() * IterationLocalForces;

        /// <summary>
        /// Get total plastic generalized strain.
        /// </summary>
        public (double ep1, double ep3) PlasticGenStrains
		{
			get
			{
				// Get generalized strains
				var (e1, e3) = _genStrains;

				double
					ep1 = PlasticStrain(e1),
					ep3 = PlasticStrain(e3);

				return (ep1, ep3);
			}
		}

		// Calculate the maximum plastic strain in a Stringer for tension and compression
		public (double eput, double epuc) MaxPlasticStrain
		{
			get
			{
				// Calculate the maximum plastic strain for tension
				double
					ec   = Concrete.ec,
					ecu  = Concrete.ecu,
					ey   = Steel?.YieldStrain ?? ec,
					esu  = Steel?.UltimateStrain ?? 0.01,
					eput = 0.3 * esu * Geometry.Length;

				// Calculate the maximum plastic strain for compression
				double et = Math.Max(ec, -ey);
				double a = Math.Min(Geometry.Width, Geometry.Height);
				double epuc = (ecu - et) * a;

				// Return a tuple in order Tension || Compression
				return (eput, epuc);
			}
		}

		/// <summary>
		/// Nonlinear stringer object
		/// </summary>
		/// <inheritdoc/>
		public NonLinearStringer(ObjectId objectId, int number, Node initialNode, Node centerNode, Node endNode, double width, double height, Parameters concreteParameters, Constitutive concreteConstitutive, UniaxialReinforcement reinforcement = null, LengthUnit geometryUnit = LengthUnit.Millimeter)
			: base(objectId, number, initialNode, centerNode, endNode, width, height, concreteParameters, concreteConstitutive, reinforcement, geometryUnit)
		{
			// Initiate F matrix
			_FMatrix = InitialFMatrix();

			// Initiate integration points
			IntPoints = new[]
			{
				new IntegrationPoint(Concrete.ecr, Steel?.YieldStrain ?? 0),
				new IntegrationPoint(Concrete.ecr, Steel?.YieldStrain ?? 0),
				new IntegrationPoint(Concrete.ecr, Steel?.YieldStrain ?? 0),
				new IntegrationPoint(Concrete.ecr, Steel?.YieldStrain ?? 0)
			};

			// Get the relations
			Relations = StressStrainRelations.GetRelations(Concrete, Reinforcement);
		}

        /// <summary>
        /// Calculate the initial flexibility matrix.
        /// </summary>
        public Matrix<double> InitialFMatrix()
		{
			double
				t1 = Concrete.Stiffness + (Reinforcement?.Stiffness ?? 0), 
				de = 1 / t1;

			// Calculate the flexibility matrix elements
			double
				de11 = de * Geometry.Length / 3,
				de12 = de11 / 2,
				de22 = de11;

			// Get the flexibility matrix
			return Matrix<double>.Build.DenseOfArray(new [,]
			{
				{ de11, de12},
				{ de12, de22}
			});
		}

		/// <inheritdoc/>
		public override void Analysis(Vector<double> globalDisplacements = null, int numStrainSteps = 5)
		{
			// Set displacements
			if (globalDisplacements != null)
				SetDisplacements(globalDisplacements);

			// Get the initial forces (from previous load step)
			var (N1, N3) = _genStresses;

			// Get initial generalized strains (from previous load step)
			var (e1i, e3i) = _genStrains;

			// Get local displacements
			var ul = LocalDisplacements;

			// Calculate current generalized strains
			double
				e1 = ul[1] - ul[0],
				e3 = ul[2] - ul[1];

			// Calculate strain increments
			double
				de1 = (e1 - e1i) / numStrainSteps,
				de3 = (e3 - e3i) / numStrainSteps;

			// Initiate flexibility matrix
			Matrix<double> F;

			// Calculate generalized strains and F matrix for N1 and N3
			((e1, e3), F) = StringerGenStrains((N1, N3));

			// Incremental process to find forces
			for (int i = 1; i <= numStrainSteps ; i++ )
			{
				// Calculate F determinant
				double d = F.Determinant();

				// Calculate increments
				double
					dN1 = ( F[1, 1] * de1 - F[0, 1] * de3) / d,
					dN3 = (-F[0, 1] * de1 + F[0, 0] * de3) / d;

				// Increment forces
				N1 += dN1;
				N3 += dN3;

				// Recalculate generalized strains and F matrix for N1 and N3
				((e1, e3), F) = StringerGenStrains((N1, N3));
			}

			// Verify the values of N1 and N3
			N1 = PlasticForce(N1);
			N3 = PlasticForce(N3);

			// Set values
			_FMatrix              = F;
			_iterationGenStresses = (N1, N3); 
			_iterationGenStrains  = (e1, e3);
		}

		/// <summary>
		/// Calculate the plastic force.
		/// </summary>
		/// <param name="N">Force to verify, in N.</param>
		private double PlasticForce(double N)
		{
			double
				Nt  = Relations.MaxCompressiveForce,
				Nyr = Reinforcement?.YieldForce ?? 0;

			// Check the value of N
			if (N < Nt)
				return Nt;

			if (N > Nyr)
				return Nyr;

			return N;
		}

		/// <summary>
		/// Calculate the stringer flexibility matrix and generalized strains.
		/// </summary>
		/// <param name="genStresses">Current generalized stresses.</param>
		/// <returns></returns>
		public ((double e1, double e3) genStrains, Matrix<double> F) StringerGenStrains((double N1, double N3) genStresses)
		{
			var (N1, N3) = genStresses;
			var N = new[]
			{
				N1, (2 * N1 + N3) / 3, (N1  + 2 * N3) / 3, N3
			};

			var e  = new double[4];
			var de = new double[4];

			for (int i = 0; i < N.Length; i++)
			{
				(e[i], de[i]) = Relations.StringerStrain(N[i], IntPoints[i]);
			}

			// Calculate approximated generalized strains
			double
				e1 = Geometry.Length * (3 * e[0] + 6 * e[1] + 3 * e[2]) / 24,
				e3 = Geometry.Length * (3 * e[1] + 6 * e[2] + 3 * e[3]) / 24;

			// Calculate the flexibility matrix elements
			double
				de11 = Geometry.Length * (3 * de[0] + 4 * de[1] + de[2]) / 24,
				de12 = Geometry.Length * (de[1] + de[2]) / 12,
				de22 = Geometry.Length * (de[1] + 4 * de[2] + 3 * de[3]) / 24;

			// Get the flexibility matrix
			var F = Matrix<double>.Build.DenseOfArray(new [,]
			{
				{ de11, de12},
				{ de12, de22}
			});

			return ((e1, e3), F);
		}

		/// <summary>
		/// Set Stringer results after reaching convergence.
		/// </summary>
		public void Results()
		{
			// Set the final values
			_genStresses = _iterationGenStresses;
			_genStrains  = _iterationGenStrains;
		}

		/// <summary>
		/// Calculate plastic strains
		/// </summary>
		/// <param name="strain">Current strain</param>
		private double PlasticStrain(double strain)
		{
			// Initialize the plastic strain
			double
				ep = 0,
				ey = Steel?.YieldStrain ?? 0,
				ec = Concrete.ec;

			// Case of tension
			if (strain > ey)
				ep = Geometry.Length / 8 * (strain - ey);

			// Case of compression
			if (strain < ec)
				ep = Geometry.Length / 8 * (strain - ec);

			return ep;
		}
	}
}

