using System;
using System.Collections.Generic;
using Autodesk.AutoCAD.DatabaseServices;
using Autodesk.AutoCAD.Geometry;
using Extensions.Number;
using Material.Concrete;
using Material.Reinforcement;
using MathNet.Numerics;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;
using OnPlaneComponents;
using SPM.Elements.PanelProperties;
using UnitsNet;
using UnitsNet.Units;
using Reinforcement      = Material.Reinforcement.WebReinforcement;

namespace SPM.Elements
{
	/// <summary>
    /// Linear panel class.
    /// </summary>
	public class LinearPanel : Panel
    {
		// Auxiliary fields
		private Matrix<double> _transMatrix, _localStiffness;
		private Vector<double> _localForces;

		/// <summary>
		/// Get transformation <see cref="Matrix"/>.
		/// </summary>
		private Matrix<double> TransformationMatrix => _transMatrix ?? CalculateTransformationMatrix();

		/// <inheritdoc/>
		public override Matrix<double> LocalStiffness => _localStiffness ?? CalculateStiffness();

		/// <inheritdoc/>
	    public override Matrix<double> GlobalStiffness => TransformationMatrix.Transpose() * LocalStiffness * TransformationMatrix;

		/// <inheritdoc/>
		public override Vector<double> Forces => TransformationMatrix.Transpose() * _localForces;
		
        /// <inheritdoc/>
        public override StressState AverageStresses
	    {
		    get
		    {
			    // Get the dimensions as a vector
			    var lsV = Vector<double>.Build.DenseOfArray(Geometry.EdgeLengths);

			    // Calculate the shear stresses
			    var tau = _localForces / (lsV * Geometry.Width);

			    // Calculate the average stress
			    double tauAvg = (-tau[0] + tau[1] - tau[2] + tau[3]) / 4;

			    return new StressState(0, 0, tauAvg);
		    }
	    }

	    /// <inheritdoc/>
	    public override PrincipalStressState ConcretePrincipalStresses
	    {
		    get
		    {
			    double sig2;

			    // Get shear stress
			    double tau = AverageStresses.TauXY;

			    // Get steel strengths
			    double
				    fyx = Reinforcement?.DirectionX?.Steel.YieldStress ?? 0,
				    fyy = Reinforcement?.DirectionY?.Steel.YieldStress ?? 0;

				if (fyx == fyy)
				    sig2 = -2 * Math.Abs(tau);

			    else
			    {
				    // Get relation of steel strengths
				    double rLambda = Math.Sqrt(fyx / fyy);
				    sig2 = -Math.Abs(tau) * (rLambda + 1 / rLambda);
			    }

			    // Calculate theta
			    double theta1;

			    if (tau >= 0)
				    theta1 = Constants.PiOver4;

			    else
				    theta1 = -Constants.PiOver4;

			    return new PrincipalStressState(0, sig2, theta1);
		    }
	    }

		/// <summary>
		/// Linear panel object.
		/// </summary>
		/// <inheritdoc/>
        public LinearPanel(ObjectId objectId, int number, Node grip1, Node grip2, Node grip3, Node grip4, Vertices vertices, double width, Parameters concreteParameters, Constitutive concreteConstitutive, Reinforcement reinforcement = null, LengthUnit unit = LengthUnit.Millimeter)
			: base(objectId, number, grip1, grip2, grip3, grip4, vertices, width, concreteParameters, concreteConstitutive, reinforcement, unit)
	    {
	    }

		/// <summary>
		/// Linear panel object.
		/// </summary>
		/// <inheritdoc/>
		public LinearPanel(ObjectId objectId, int number, Node grip1, Node grip2, Node grip3, Node grip4, Vertices vertices, Length width, Parameters concreteParameters, Constitutive concreteConstitutive, Reinforcement reinforcement = null)
			: base(objectId, number, grip1, grip2, grip3, grip4, vertices, width, concreteParameters, concreteConstitutive, reinforcement)
		{
		}

		/// <summary>
		/// Linear panel object.
		/// </summary>
		/// <inheritdoc/>
		public LinearPanel(ObjectId objectId, int number, Node grip1, Node grip2, Node grip3, Node grip4, IEnumerable<Point3d> vertices, Length width, Parameters concreteParameters, Constitutive concreteConstitutive, Reinforcement reinforcement = null)
			: base(objectId, number, grip1, grip2, grip3, grip4, vertices, width, concreteParameters, concreteConstitutive, reinforcement)
		{
		}

        /// <summary>
        /// Linear panel object.
        /// </summary>
        /// <inheritdoc/>
        public LinearPanel(ObjectId objectId, int number, Node grip1, Node grip2, Node grip3, Node grip4, IEnumerable<Point3d> vertices, double width, Parameters concreteParameters, Constitutive concreteConstitutive, Reinforcement reinforcement = null, LengthUnit unit = LengthUnit.Millimeter)
	        : base(objectId, number, grip1, grip2, grip3, grip4, vertices, width, concreteParameters, concreteConstitutive, reinforcement, unit)
	    {
	    }

        /// <summary>
        /// Linear panel object.
        /// </summary>
        /// <inheritdoc/>
        public LinearPanel(ObjectId objectId, int number, IEnumerable<Node> nodes, Vertices vertices, Length width, Parameters concreteParameters, Constitutive concreteConstitutive, WebReinforcement reinforcement = null)
            : base(objectId, number, nodes, vertices, width, concreteParameters, concreteConstitutive, reinforcement)
        {
        }

        /// <summary>
        /// Linear panel object.
        /// </summary>
        /// <inheritdoc/>
        public LinearPanel(ObjectId objectId, int number, IEnumerable<Node> nodes, Vertices vertices, double width, Parameters concreteParameters, Constitutive concreteConstitutive, WebReinforcement reinforcement = null, LengthUnit unit = LengthUnit.Millimeter)
            : this (objectId, number, nodes, vertices, Length.From(width, unit), concreteParameters, concreteConstitutive, reinforcement)
        {
        }

        /// <summary>
        /// Linear panel object.
        /// </summary>
        /// <inheritdoc/>
        public LinearPanel(ObjectId objectId, int number, IEnumerable<Node> nodes, IEnumerable<Point3d> vertices, Length width, Parameters concreteParameters, Constitutive concreteConstitutive, WebReinforcement reinforcement = null)
            : this(objectId, number, nodes, new Vertices(vertices, width.Unit), width, concreteParameters, concreteConstitutive, reinforcement)
        {
        }

        /// <summary>
        /// Linear panel object.
        /// </summary>
        /// <inheritdoc/>
        public LinearPanel(ObjectId objectId, int number, IEnumerable<Node> nodes, IEnumerable<Point3d> vertices, double width, Parameters concreteParameters, Constitutive concreteConstitutive, WebReinforcement reinforcement = null, LengthUnit unit = LengthUnit.Millimeter)
            : this(objectId, number, nodes, new Vertices(vertices, unit), Length.From(width, unit), concreteParameters, concreteConstitutive, reinforcement)
        {
        }
		
        /// <summary>
        /// Calculate transformation matrix
        /// </summary>
        private Matrix<double> CalculateTransformationMatrix()
	    {
		    // Get the transformation matrix
		    // Direction cosines
		    var (m1, n1) = Geometry.Edge1.Angle.DirectionCosines();
		    var (m2, n2) = Geometry.Edge2.Angle.DirectionCosines();
            var (m3, n3) = Geometry.Edge3.Angle.DirectionCosines();
            var (m4, n4) = Geometry.Edge4.Angle.DirectionCosines();

            // T matrix
            _transMatrix =  Matrix<double>.Build.DenseOfArray(new [,]
		    {
			    {m1, n1,  0,  0,  0,  0,  0,  0},
			    { 0,  0, m2, n2,  0,  0,  0,  0},
			    { 0,  0,  0,  0, m3, n3,  0,  0},
			    { 0,  0,  0,  0,  0,  0, m4, n4}
		    });

            return _transMatrix;
	    }

        /// <summary>
        /// Calculate panel stiffness
        /// </summary>
        private Matrix<double> CalculateStiffness()
        {
	        // If the panel is rectangular
	        _localStiffness = Geometry.Rectangular ? RectangularPanelStiffness() : NonRectangularPanelStiffness();

	        return _localStiffness;
        }

        /// <summary>
        /// Calculate panel stiffness if panel is rectangular.
        /// <para>See: <seealso cref="PanelGeometry.Rectangular"/></para>
        /// </summary>
	    private Matrix<double> RectangularPanelStiffness()
	    {
		    // Get the dimensions
		    double
			    w = Geometry.Width,
			    a = Geometry.Dimensions.a,
			    b = Geometry.Dimensions.b;

		    // Calculate the parameters of the stiffness matrix
		    double
			    a_b = a / b,
			    b_a = b / a;

		    // Calculate Gc
		    double Gc = Concrete.Ec / 2.4;

		    // Calculate the stiffness matrix
		    return 
			    Gc * w * Matrix<double>.Build.DenseOfArray(new[,]
			    {
				    {a_b,  -1, a_b,  -1},
				    { -1, b_a,  -1, b_a},
				    {a_b,  -1, a_b,  -1},
				    { -1, b_a,  -1, b_a}
			    });
	    }

        /// <summary>
        /// Calculate panel stiffness if panel is not rectangular.
        /// <para>See: <seealso cref="PanelGeometry.Rectangular"/></para>
        /// </summary>
	    private Matrix<double> NonRectangularPanelStiffness()
	    {
		    // Get the dimensions
		    double[]
			    x = Geometry.Vertices.XCoordinates,
			    y = Geometry.Vertices.YCoordinates;

		    var (a, b, c, d) = Geometry.Dimensions;
		    double
			    w  = Geometry.Width,
			    l1 = Geometry.Edge1.Length,
			    l2 = Geometry.Edge2.Length,
			    l3 = Geometry.Edge3.Length,
			    l4 = Geometry.Edge4.Length;

		    // Calculate Gc
		    double Gc = Concrete.Ec / 2.4;

		    // Equilibrium parameters
		    double
			    c1 = x[1] - x[0],
			    c2 = x[2] - x[1],
			    c3 = x[3] - x[2],
			    c4 = x[0] - x[3],
			    s1 = y[1] - y[0],
			    s2 = y[2] - y[1],
			    s3 = y[3] - y[2],
			    s4 = y[0] - y[3],
			    r1 = x[0] * y[1] - x[1] * y[0],
			    r2 = x[1] * y[2] - x[2] * y[1],
			    r3 = x[2] * y[3] - x[3] * y[2],
			    r4 = x[3] * y[0] - x[0] * y[3];

		    // Kinematic parameters
		    double
			    t1 = -b * c1 - c * s1,
			    t2 = a * s2 + d * c2,
			    t3 = b * c3 + c * s3,
			    t4 = -a * s4 - d * c4;

		    // Matrices to calculate the determinants
		    var km1 = Matrix<double>.Build.DenseOfArray(new [,]
		    {
			    {c2, c3, c4},
			    {s2, s3, s4},
			    {r2, r3, r4},
		    });

		    var km2 = Matrix<double>.Build.DenseOfArray(new [,]
		    {
			    {c1, c3, c4}, 
			    {s1, s3, s4},
			    {r1, r3, r4},
		    });

		    var km3 = Matrix<double>.Build.DenseOfArray(new [,]
		    {
			    {c1, c2, c4},
			    {s1, s2, s4},
			    {r1, r2, r4},
		    });

		    var km4 = Matrix<double>.Build.DenseOfArray(new [,]
		    {
			    {c1, c2, c3},
			    {s1, s2, s3},
			    {r1, r2, r3},
		    });

		    // Calculate the determinants
		    double
			    k1 = km1.Determinant(),
			    k2 = km2.Determinant(),
			    k3 = km3.Determinant(),
			    k4 = km4.Determinant();

		    // Calculate kf and ku
		    double
			    kf =  k1 + k2 + k3 + k4,
			    ku = -t1 * k1 + t2 * k2 - t3 * k3 + t4 * k4;

		    // Calculate D
		    double D = 16 * Gc * w / (kf * ku);

		    // Get the vector B
		    var B = Vector<double>.Build.DenseOfArray(new []
		    {
			    -k1 * l1, k2 * l2, -k3 * l3, k4 * l4
		    });

		    // Get the stiffness matrix
		    return 
			    B.ToColumnMatrix() * D * B.ToRowMatrix();
	    }

		/// <summary>
        /// Calculate panel local forces.
        /// </summary>
	    private Vector<double> CalculateForces()
	    {
		    // Get the parameters
		    var up = Displacements;
		    var T  = TransformationMatrix;
		    var Kl = LocalStiffness;

		    // Get the displacements in the direction of the edges
		    var ul = T * up;

		    // Calculate the vector of forces
		    var fl = Kl * ul;

		    // Aproximate small values to zero
		    fl.CoerceZero(0.000001);

		    return fl;
	    }

		/// <summary>
        /// Set displacements and calculate forces.
        /// </summary>
        /// <param name="globalDisplacements">The global displacement <see cref="Vector"/>.</param>
		public override void Analysis(Vector<double> globalDisplacements = null)
	    {
		    // Set displacements
		    if (globalDisplacements != null)
			    SetDisplacements(globalDisplacements);

			// Calculate local forces
		    _localForces = CalculateForces();
	    }
    }
}
