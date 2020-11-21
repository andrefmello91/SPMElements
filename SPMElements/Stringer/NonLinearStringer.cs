﻿using System;
using System.Collections.Generic;
using System.Linq;
using Autodesk.AutoCAD.DatabaseServices;
using Autodesk.AutoCAD.Geometry;
using Extensions.Number;
using Material.Concrete;
using Material.Reinforcement;
using Material.Reinforcement.Uniaxial;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;
using SPM.Elements.StringerProperties;
using UnitsNet;
using UnitsNet.Units;

namespace SPM.Elements
{
	/// <summary>
	/// Nonlinear stringer class.
	/// </summary>
	public class NonLinearStringer : Stringer
	{
		// Auxiliary fields
		private Matrix<double> _FMatrix;
		private (double e1, double e3) _lastGenStrains,  _curGenStrains;
		private (double N1, double N3) _lastGenStresses, _curGenStresses;
		private (double T, double C)? _maxPlasticStrain;

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
        /// Get <see cref="Steel"/> of <see cref="Stringer.Reinforcement"/>.
        /// </summary>
		private Steel Steel => Reinforcement?.Steel;

		/// <inheritdoc/>
		public override Matrix<double> LocalStiffness => _BMatrix.Transpose() * (_FMatrix ?? InitialFMatrix()).Inverse() * _BMatrix;

		/// <inheritdoc/>
		protected override Vector<double> LocalForces
		{
			get
			{
				var (N1, N3) = _lastGenStresses;

				return
					Vector<double>.Build.DenseOfArray(new[]
					{
						-N1, N1 - N3, N3
					});
			}
		}

		/// <summary>
		/// Get local force <see cref="Vector"/> calculated at the current iteration.
		/// </summary>
		private Vector<double> CurrentLocalForces
		{
			get
			{
				var (N1, N3) = _curGenStresses;

				return
					Vector<double>.Build.DenseOfArray(new[]
					{
						-N1, N1 - N3, N3
					});
			}
		}

        /// <summary>
        /// Get global force <see cref="Vector"/> calculated at the current iteration.
        /// </summary>
		public Vector<double> CurrentGlobalForces => TransformationMatrix.Transpose() * CurrentLocalForces;

        /// <summary>
        /// Get total plastic generalized strain.
        /// </summary>
        private (double ep1, double ep3) PlasticGenStrains
		{
			get
			{
				// Get generalized strains
				var (e1, e3) = _lastGenStrains;

				double
					ep1 = PlasticStrain(e1),
					ep3 = PlasticStrain(e3);

				return (ep1, ep3);
			}
		}

        /// <summary>
        /// Get the maximum plastic strain in a Stringer for tension (T) and compression (C).
        /// </summary>
		private (double T, double C) MaxPlasticStrain => _maxPlasticStrain ?? CalculateMaxPlasticStrain();

		/// <summary>
		/// Nonlinear stringer object
		/// </summary>
		/// <inheritdoc/>
		public NonLinearStringer(ObjectId objectId, int number, Node grip1, Node grip2, Node grip3, double width, double height, Parameters concreteParameters, ConstitutiveModel concreteConstitutive, UniaxialReinforcement reinforcement = null, LengthUnit unit = LengthUnit.Millimeter)
			: this(objectId, number, grip1, grip2, grip3, Length.From(width, unit), Length.From(height, unit), concreteParameters, concreteConstitutive, reinforcement)
		{
		}

		/// <summary>
		/// Nonlinear stringer object
		/// </summary>
		/// <inheritdoc/>
		public NonLinearStringer(ObjectId objectId, int number, Node grip1, Node grip2, Node grip3, Length width, Length height, Parameters concreteParameters, ConstitutiveModel concreteConstitutive, UniaxialReinforcement reinforcement = null)
			: base(objectId, number, grip1, grip2, grip3, width, height, concreteParameters, concreteConstitutive, reinforcement)
		{
			// Initiate integration points
			IntPoints = GetIntPoints().ToArray();
		}

		/// <summary>
		/// Linear stringer object.
		/// </summary>
		/// <inheritdoc/>
		public NonLinearStringer(ObjectId objectId, int number, IEnumerable<Node> nodes, Point3d grip1Position, Point3d grip3Position, double width, double height, Parameters concreteParameters, ConstitutiveModel concreteConstitutive, UniaxialReinforcement reinforcement = null, LengthUnit unit = LengthUnit.Millimeter)
			: this(objectId, number, nodes, grip1Position, grip3Position, Length.From(width, unit), Length.From(height, unit), concreteParameters, concreteConstitutive, reinforcement)
		{
		}

		/// <summary>
		/// Linear stringer object.
		/// </summary>
		/// <inheritdoc/>
		public NonLinearStringer(ObjectId objectId, int number, IEnumerable<Node> nodes, Point3d grip1Position, Point3d grip3Position, Length width, Length height, Parameters concreteParameters, ConstitutiveModel concreteConstitutive, UniaxialReinforcement reinforcement = null)
			: base(objectId, number, nodes, grip1Position, grip3Position, width, height, concreteParameters, concreteConstitutive, reinforcement)
		{
			// Initiate integration points
			IntPoints = GetIntPoints().ToArray();
		}

		/// <summary>
        /// Get <see cref="IntegrationPoint"/>'s.
        /// </summary>
		private IEnumerable<IntegrationPoint> GetIntPoints()
		{
			for (int i = 0; i < 4; i++)
				yield return IntegrationPoint.Read(Concrete, Reinforcement);
		}

		/// <summary>
		/// Calculate the initial flexibility <see cref="Matrix"/>.
		/// </summary>
		private Matrix<double> InitialFMatrix()
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

		/// <summary>
		/// Returns the initial local stiffness <see cref="Matrix"/>.
		/// </summary>
        private Matrix<double> InitialLocalStiffness() => _BMatrix.Transpose() * InitialFMatrix().Inverse() * _BMatrix;

		/// <summary>
		/// Returns the initial stiffness <see cref="Matrix"/>.
		/// </summary>
        public Matrix<double> InitialStiffness() => TransformationMatrix.Transpose() * InitialLocalStiffness() * TransformationMatrix;

		/// <inheritdoc/>
		public override void Analysis(Vector<double> globalDisplacements = null, int numStrainSteps = 5)
		{
			// Set displacements
			if (globalDisplacements != null)
				SetDisplacements(globalDisplacements);

			// Get the initial forces (from previous load step)
			var (N1, N3) = _lastGenStresses;

			// Get initial generalized strains (from previous load step)
			var (e1i, e3i) = _lastGenStrains;

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

			// Calculate generalized strains and F matrix for N1 and N3
			(e1, e3) = StringerGenStrains((N1, N3), out var F);

			// Incremental process to find forces
			for (int i = 1; i <= numStrainSteps ; i++ )
			{
				//if (e1.IsNaN() || e3.IsNaN())
				//	break;

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
				(e1, e3) = StringerGenStrains((N1, N3), out F);
			}

			// Verify the values of N1 and N3
			N1 = PlasticForce(N1);
			N3 = PlasticForce(N3);

			// Set values
			_FMatrix = F;
			_curGenStresses = (N1, N3);
			_curGenStrains = (e1, e3);
		}

		/// <summary>
		/// Calculate the generalized strains and stringer flexibility matrix .
		/// </summary>
		/// <param name="genStresses">Current generalized stresses.</param>
		/// <param name="F">The flexibility <see cref="Matrix"/>.</param>
		private (double e1, double e3) StringerGenStrains((double N1, double N3) genStresses, out Matrix<double> F)
		{
			var (N1, N3) = genStresses;

			var N = new[] { N1, (2 * N1 + N3) / 3, (N1  + 2 * N3) / 3, N3 };

			var e  = new double[4];
			var de = new double[4];

			for (int i = 0; i < N.Length; i++)
				(e[i], de[i]) = IntPoints[i].CalculateStrain(N[i]);

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
			F = Matrix<double>.Build.DenseOfArray(new [,]
			{
				{ de11, de12},
				{ de12, de22}
			});

			return (e1, e3);
		}

		/// <summary>
		/// Set Stringer results after reaching convergence.
		/// </summary>
		public void Results()
		{
			// Set the final values
			_lastGenStresses = _curGenStresses;
			_lastGenStrains  = _curGenStrains;
		}

		/// <summary>
		/// Calculate the plastic force.
		/// </summary>
		/// <param name="force">Force to verify, in N.</param>
		private double PlasticForce(double force)
		{
			double
				Nt  = IntPoints[0].MaxCompressiveForce,
				Nyr = Reinforcement?.YieldForce ?? 0;

			// Check the value of N
			return
				force < Nt 
					? Nt 
					: force > Nyr 
						? Nyr 
						: force;
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

        /// <summary>
        /// Calculate the maximum plastic strain in a Stringer for tension (T) and compression (C).
        /// </summary>
        private (double T, double C) CalculateMaxPlasticStrain()
        {
	        // Calculate the maximum plastic strain for tension
	        double
		        ec = Concrete.ec,
		        ecu = Concrete.ecu,
		        ey = Steel?.YieldStrain ?? ec,
		        esu = Steel?.UltimateStrain ?? 0.01,
		        eput = 0.3 * esu * Geometry.Length;

	        // Calculate the maximum plastic strain for compression
	        double et = Math.Max(ec, -ey);
	        double a = Math.Min(Geometry.Width, Geometry.Height);
	        double epuc = (ecu - et) * a;

	        // Return a tuple in order Tension || Compression
	        _maxPlasticStrain = (eput, epuc);

	        return (eput, epuc);
        }
	}
}

