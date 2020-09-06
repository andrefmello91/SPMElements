using System;
using Autodesk.AutoCAD.DatabaseServices;
using Material.Concrete;
using Material.Reinforcement;
using MathNet.Numerics.LinearAlgebra;
using UnitsNet.Units;
using Concrete = Material.Concrete.UniaxialConcrete;

namespace SPMElements
{
	/// <summary>
	/// Stringer linear class.
	/// </summary>
	public class LinearStringer : Stringer
	{
		/// <inheritdoc/>
		public override Matrix<double> LocalStiffness
		{
			get
			{
				// Calculate the constant factor of stiffness
				double EcA_L = Concrete.Stiffness / Geometry.Length;

				// Calculate the local stiffness matrix
				return
					EcA_L * Matrix<double>.Build.DenseOfArray(new double[,]
					{
						{  4, -6,  2 },
						{ -6, 12, -6 },
						{  2, -6,  4 }
					});
			}
		}

        /// <summary>
        /// Linear stringer object.
        /// </summary>
        /// <inheritdoc/>
        public LinearStringer(ObjectId objectId, int number, Node grip1, Node grip2, Node grip3, double width, double height, Parameters concreteParameters, Constitutive concreteConstitutive, UniaxialReinforcement reinforcement = null, LengthUnit geometryUnit = LengthUnit.Millimeter) : base(objectId, number, grip1, grip2, grip3, width, height, concreteParameters, concreteConstitutive, reinforcement, geometryUnit)
        {
        }

        /// <summary>
        /// Calculate Stringer forces
        /// </summary>
        public Vector<double> CalculateForces()
		{
			// Get the parameters
			var Kl = LocalStiffness;
			var ul = LocalDisplacements;

			// Calculate the vector of normal forces
			var fl = Kl * ul;

			// Approximate small values to zero
			fl.CoerceZero(0.001);

			return fl;
		}

		/// <inheritdoc/>
		public override void Analysis(Vector<double> globalDisplacements = null, int numStrainSteps = 5)
		{
			// Set displacements
			if (globalDisplacements != null)
				SetDisplacements(globalDisplacements);

			LocalForces = CalculateForces();
		}
	}
}

