using System;
using Autodesk.AutoCAD.DatabaseServices;
using Material.Concrete;
using Material.Reinforcement;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;
using UnitsNet;
using UnitsNet.Units;
using Concrete = Material.Concrete.UniaxialConcrete;

namespace SPMElements
{
	/// <summary>
	/// Stringer linear class.
	/// </summary>
	public class LinearStringer : Stringer
	{
		// Auxiliary fields
		private Matrix<double> _localStiffness;

		/// <inheritdoc/>
		public override Matrix<double> LocalStiffness => _localStiffness ?? CalculateStiffness();

		/// <summary>
        /// Linear stringer object.
        /// </summary>
        /// <inheritdoc/>
		public LinearStringer(ObjectId objectId, int number, Node grip1, Node grip2, Node grip3, double width, double height, Parameters concreteParameters, Constitutive concreteConstitutive, UniaxialReinforcement reinforcement = null, LengthUnit unit = LengthUnit.Millimeter) : base(objectId, number, grip1, grip2, grip3, width, height, concreteParameters, concreteConstitutive, reinforcement, unit)
		{
		}

		/// <summary>
		/// Linear stringer object.
		/// </summary>
		public LinearStringer(ObjectId objectId, int number, Node grip1, Node grip2, Node grip3, Length width, Length height, Parameters concreteParameters, Constitutive concreteConstitutive, UniaxialReinforcement reinforcement = null) : base(objectId, number, grip1, grip2, grip3, width, height, concreteParameters, concreteConstitutive, reinforcement)
		{
		}

        /// <summary>
        /// Calculate local stiffness <see cref="Matrix"/>.
        /// </summary>
        /// <returns></returns>
        private Matrix<double> CalculateStiffness()
        {
	        // Calculate the constant factor of stiffness
	        double EcA_L = Concrete.Stiffness / Geometry.Length;

	        // Calculate the local stiffness matrix
	        _localStiffness = 
		        EcA_L * Matrix<double>.Build.DenseOfArray(new double[,]
		        {
			        {  4, -6,  2 },
			        { -6, 12, -6 },
			        {  2, -6,  4 }
		        });

	        return _localStiffness;
        }


        /// <summary>
        /// Calculate Stringer forces
        /// </summary>
        private Vector<double> CalculateForces()
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

