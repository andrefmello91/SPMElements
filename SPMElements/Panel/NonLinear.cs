using System;
using System.Collections.Generic;
using System.Linq;
using Autodesk.AutoCAD.DatabaseServices;
using Autodesk.AutoCAD.Geometry;
using Material.Concrete;
using Material.Reinforcement;
using Material.Reinforcement.Biaxial;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;
using OnPlaneComponents;
using RCMembrane;
using SPM.Elements.PanelProperties;
using UnitsNet;
using UnitsNet.Units;

namespace SPM.Elements
{
    public class NonLinearPanel : Panel
    {
		// Auxiliary fields
		private Matrix<double> _BA, _Q, _Pc, _Ps, _Dc, _Ds;

        /// <summary>
        /// Get <see cref="Membrane"/> integration points.
        /// </summary>
        public Membrane[] IntegrationPoints  { get; }

        /// <summary>
        /// Get panel strain <see cref="Vector"/>.
        /// </summary>
        public Vector<double> StrainVector => (_BA ?? CalculateBA()) * Displacements;

        /// <summary>
        /// Get/set panel concrete stress <see cref="Vector"/>.
        /// </summary>
	    public Vector<double> ConcreteStresses { get; private set; }

        /// <summary>
        /// Get/set panel reinforcement stress <see cref="Vector"/>.
        /// </summary>
	    public Vector<double> ReinforcementStresses { get; private set; }

        /// <summary>
        /// Get panel stress <see cref="Vector"/>.
        /// </summary>
	    public Vector<double> Stresses => ConcreteStresses + ReinforcementStresses;

	    /// <inheritdoc/>
	    public override Matrix<double> GlobalStiffness
	    {
		    get
		    {
				if (_Dc is null || _Ds is null)
					(_Dc, _Ds) = InitialMaterialStiffness();

                Matrix<double>
				    BA = _BA ?? CalculateBA(),
				    Pc = _Pc ?? CalculateP().Pc,
				    Ps = _Ps ?? CalculateP().Ps,
				    Q  = _Q  ?? CalculateQ();

			    var QPs = Q * Ps;
			    var QPc = Q * Pc;

			    var kc = QPc * _Dc * BA;
			    var ks = QPs * _Ds * BA;

			    return kc + ks;
		    }
	    }

	    /// <inheritdoc/>
	    public override PrincipalStressState ConcretePrincipalStresses
	    {
		    get
		    {
			    var sigma = StressState.Zero;

			    for (int i = 0; i < 4; i++)
				    sigma += IntegrationPoints[i].Concrete.Stresses;

				// Calculate average
				sigma = 0.25 * sigma;

				// Return principal
			    return PrincipalStressState.FromStress(sigma);
		    }
	    }

	    /// <inheritdoc/>
	    public override StressState AverageStresses
	    {
		    get
		    {
			    // Get stress vector
			    var sigma = Stresses;

			    // Calculate average stresses
			    double
				    sigxm = (sigma[0] + sigma[3] + sigma[6] + sigma[9]) / 4,
				    sigym = (sigma[1] + sigma[4] + sigma[7] + sigma[10]) / 4,
				    sigxym = (sigma[2] + sigma[5] + sigma[8] + sigma[11]) / 4;

			    return new StressState(sigxm, sigym, sigxym);
		    }
	    }

	    /// <summary>
	    /// Nonlinear panel object.
	    /// </summary>
	    /// <inheritdoc/>
	    public NonLinearPanel(ObjectId objectId, int number, Node grip1, Node grip2, Node grip3, Node grip4, Vertices vertices, Length width, Parameters concreteParameters, ConstitutiveModel model, WebReinforcement reinforcement = null)
		    : base(objectId, number, grip1, grip2, grip3, grip4, vertices, width, concreteParameters, model, reinforcement)
	    {
		    IntegrationPoints = IntPoints(concreteParameters, model).ToArray();
	    }

        /// <summary>
        /// Nonlinear panel object.
        /// </summary>
        /// <inheritdoc/>
        public NonLinearPanel(ObjectId objectId, int number, Node grip1, Node grip2, Node grip3, Node grip4, Vertices vertices, double width, Parameters concreteParameters, ConstitutiveModel model, WebReinforcement reinforcement = null, LengthUnit unit = LengthUnit.Millimeter)
            : this(objectId, number, grip1, grip2, grip3, grip4, vertices, Length.From(width, unit), concreteParameters, model, reinforcement)
        {
        }

        /// <summary>
        /// Nonlinear panel object.
        /// </summary>
        /// <inheritdoc/>
        public NonLinearPanel(ObjectId objectId, int number, Node grip1, Node grip2, Node grip3, Node grip4, IEnumerable<Point3d> vertices, Length width, Parameters concreteParameters, ConstitutiveModel model, WebReinforcement reinforcement = null)
	        : this(objectId, number, grip1, grip2, grip3, grip4, new Vertices(vertices, width.Unit), width, concreteParameters, model, reinforcement)
        {
        }

        /// <summary>
        /// Nonlinear panel object.
        /// </summary>
        /// <inheritdoc/>
        public NonLinearPanel(ObjectId objectId, int number, Node grip1, Node grip2, Node grip3, Node grip4, IEnumerable<Point3d> vertices, double width, Parameters concreteParameters, ConstitutiveModel model, WebReinforcement reinforcement = null, LengthUnit unit = LengthUnit.Millimeter)
            : this(objectId, number, grip1, grip2, grip3, grip4, new Vertices(vertices, unit), Length.From(width, unit), concreteParameters, model, reinforcement)
        {
        }

        /// <summary>
        /// Nonlinear panel object.
        /// </summary>
        /// <inheritdoc/>
        public NonLinearPanel(ObjectId objectId, int number, IEnumerable<Node> nodes, Vertices vertices, Length width, Parameters concreteParameters, ConstitutiveModel model, WebReinforcement reinforcement = null)
            : base(objectId, number, nodes, vertices, width, concreteParameters, model, reinforcement)
        {
	        IntegrationPoints = IntPoints(concreteParameters, model).ToArray();
        }

        /// <summary>
        /// Nonlinear panel object.
        /// </summary>
        /// <inheritdoc/>
        public NonLinearPanel(ObjectId objectId, int number, IEnumerable<Node> nodes, Vertices vertices, double width, Parameters concreteParameters, ConstitutiveModel model, WebReinforcement reinforcement = null, LengthUnit unit = LengthUnit.Millimeter)
            : this(objectId, number, nodes, vertices, Length.From(width, unit), concreteParameters, model, reinforcement)
        {
        }

        /// <summary>
        /// Nonlinear panel object.
        /// </summary>
        /// <inheritdoc/>
        public NonLinearPanel(ObjectId objectId, int number, IEnumerable<Node> nodes, IEnumerable<Point3d> vertices, Length width, Parameters concreteParameters, ConstitutiveModel model, WebReinforcement reinforcement = null)
            : this(objectId, number, nodes, new Vertices(vertices, width.Unit), width, concreteParameters, model, reinforcement)
        {
        }

        /// <summary>
        /// Nonlinear panel object.
        /// </summary>
        /// <inheritdoc/>
        public NonLinearPanel(ObjectId objectId, int number, IEnumerable<Node> nodes, IEnumerable<Point3d> vertices, double width, Parameters concreteParameters, ConstitutiveModel model, WebReinforcement reinforcement = null, LengthUnit unit = LengthUnit.Millimeter)
            : this(objectId, number, nodes, new Vertices(vertices, unit), Length.From(width, unit), concreteParameters, model, reinforcement)
        {
        }

        /// <summary>
        /// Initiate <see cref="Membrane"/> integration points.
        /// </summary>
        /// <param name="concreteParameters">Concrete <see cref="Parameters"/> object.</param>
        /// <param name="model">Concrete <see cref="ConstitutiveModel"/> object.</param>
        /// <returns></returns>
        private IEnumerable<Membrane> IntPoints(Parameters concreteParameters, ConstitutiveModel model)
	    {
		    for (int i = 0; i < 4; i++)
			    yield return Membrane.Read(concreteParameters, Reinforcement, Geometry.Width, model);
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

	        // Get the vector strains and stresses
	        var ev = StrainVector;

	        // Calculate the material matrix of each int. point by MCFT
	        for (int i = 0; i < 4; i++)
	        {
		        // Get the strains and stresses
		        var e = StrainState.FromVector(ev.SubVector(3 * i, 3));

		        // Calculate stresses by MCFT
		        IntegrationPoints[i].Calculate(e);
	        }

	        // Calculate stresses, forces and update stiffness
	        CalculateStresses();
	        CalculateForces();
	        UpdateStiffness();
        }

        /// <summary>
        /// Calculate BA matrix.
        /// </summary>
        private Matrix<double> CalculateBA()
	    {
		    var (a, b, c, d) = Geometry.Dimensions;

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
		    var A = Matrix<double>.Build.DenseOfArray(new[,]
		    {
			    {   d_t1,     0,   b_t1,     0, -d_t1,      0, -b_t1,      0 },
			    {      0, -a_t1,      0, -c_t1,     0,   a_t1,     0,   c_t1 },
			    { -a_2t1, d_2t1, -c_2t1, b_2t1, a_2t1, -d_2t1, c_2t1, -b_2t1 },
			    { - a_t2,     0,   a_t2,     0, -a_t2,      0,  a_t2,      0 },
			    {      0,  b_t3,      0, -b_t3,     0,   b_t3,     0,  -b_t3 }
		    });

		    // Calculate the components of B matrix
		    double
			    c_a = c / a,
			    d_b = d / b,
			    a2_b = 2 * a / b,
			    b2_a = 2 * b / a,
			    c2_b = 2 * c / b,
			    d2_a = 2 * d / a;

            // Create B matrix
            //var B = Matrix<double>.Build.DenseOfArray(new[,]
            //{
            // {1, 0, 0,  -c_a,     0 },
            // {0, 1, 0,     0,    -1 },
            // {0, 0, 2,  b2_a,  c2_b },
            // {1, 0, 0,     1,     0 },
            // {0, 1, 0,     0,   d_b },
            // {0, 0, 2, -d2_a, -a2_b },
            // {1, 0, 0,   c_a,     0 },
            // {0, 1, 0,     0,     1 },
            // {0, 0, 2, -b2_a, -c2_b },
            // {1, 0, 0,    -1,     0 },
            // {0, 1, 0,     0,  -d_b },
            // {0, 0, 2,  d2_a,  a2_b }
            //});

            var B = Matrix<double>.Build.DenseOfArray(new[,]
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
            });

            _BA = B * A;

		    return _BA;
	    }

        /// <summary>
        /// Calculate Q matrix.
        /// </summary>
	    private Matrix<double> CalculateQ()
	    {
		    // Get dimensions
		    var (a, b, c, d) = Geometry.Dimensions;

		    // Calculate t4
		    double t4 = a * a + b * b;

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
		    _Q = 1 / Tt4 * Matrix<double>.Build.DenseOfArray(new[,]
			    {
				    {  a2,     bc,  bdMt4, -ab, -a2,    -bc, MbdMt4,  ab },
				    {   0,    Tt4,      0,   0,   0,      0,      0,   0 },
				    {   0,      0,    Tt4,   0,   0,      0,      0,   0 },
				    { -ab,  acMt4,     ad,  b2,  ab, MacMt4,    -ad, -b2 },
				    { -a2,    -bc, MbdMt4,  ab,  a2,     bc,  bdMt4, -ab },
				    {   0,      0,      0,   0,   0,    Tt4,      0,   0 },
				    {   0,      0,      0,   0,   0,      0,    Tt4,   0 },
				    {  ab, MacMt4,    -ad, -b2, -ab,  acMt4,     ad,  b2 }
			    });

		    return _Q;
	    }

        /// <summary>
        /// Calculate P matrices for concrete and steel
        /// </summary>
        private (Matrix<double> Pc, Matrix<double> Ps) CalculateP()
	    {
		    // Get dimensions
		    double[]
			    x = Geometry.Vertices.XCoordinates,
			    y = Geometry.Vertices.YCoordinates;

		    double t = Geometry.Width;
		    var c = Geometry.StringerDimensions;

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

			// Set values
			_Pc = Pc;
			_Ps = Ps;

		    return
			    (Pc, Ps);
	    }

		/// <summary>
        /// Calculate and set stress vectors.
        /// <para>See: <see cref="ConcreteStresses"/>, <see cref="ReinforcementStresses"/>.</para>
        /// </summary>
	    private void CalculateStresses()
	    {
		    ConcreteStresses = Vector<double>.Build.Dense(12);
		    ReinforcementStresses = Vector<double>.Build.Dense(12);

		    for (int i = 0; i < 4; i++)
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
		/// Calculate and set global force vector.
		/// <para>See: <see cref="Panel.Forces"/>.</para>
		/// </summary>
	    private void CalculateForces()
	    {
		    double t0, t1, t2, t3, t4;

            // Get dimensions
            double[]
	            x = Geometry.Vertices.XCoordinates,
	            y = Geometry.Vertices.YCoordinates;

		    var (a, b, c, d) = Geometry.Dimensions;
		    var s            = Geometry.StringerDimensions;
		    var t            = Geometry.Width;

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

		    Forces =
			    Vector<double>.Build.DenseOfArray(new []
			    {
				    f1, f2, f3, f4, f5, f6, f7, f8
			    });

		    // Check value of t3
		    double CheckT3(double value) => value < 0 ? 0 : value;
	    }

		/// <summary>
		/// Update stiffness.
		/// </summary>
		private void UpdateStiffness()
		{
			_Dc = Matrix<double>.Build.Dense(12, 12);
			_Ds = Matrix<double>.Build.Dense(12, 12);

			for (int i = 0; i < 4; i++)
			{
				// Get the stiffness
				var Dci = IntegrationPoints[i].Concrete.Stiffness;
				var Dsi = IntegrationPoints[i].Reinforcement.Stiffness;

				// Set to stiffness
				_Dc.SetSubMatrix(3 * i, 3 * i, Dci);
				_Ds.SetSubMatrix(3 * i, 3 * i, Dsi);
			}
		}

        /// <summary>
        /// Calculate initial material stiffness matrices.
        /// </summary>
        public (Matrix<double> Dc, Matrix<double> Ds) InitialMaterialStiffness()
	    {
		    var Dc = Matrix<double>.Build.Dense(12, 12);
		    var Ds = Matrix<double>.Build.Dense(12, 12);

		    for (int i = 0; i < 4; i++)
		    {
			    // Get the stiffness
			    var Dci = IntegrationPoints[i].Concrete.InitialStiffness;
			    var Dsi = IntegrationPoints[i].Concrete.InitialStiffness;

			    // Set to stiffness
			    Dc.SetSubMatrix(3 * i, 3 * i, Dci);
			    Ds.SetSubMatrix(3 * i, 3 * i, Dsi);
		    }

		    return
			    (Dc, Ds);
	    }

        /// <summary>
        /// Calculate the vector of strains (alternate method).
        /// </summary>
        private Vector<double> CalculateStrains()
        {
	        // Get dimensions and displacements
	        var (a, b, c, d) = Geometry.Dimensions;
	        var u = Displacements;

	        // Calculate constants
	        double
		        t1 = a * b - c * d,
		        t2 = a * a - c * c,
		        t3 = b * b - d * d,
		        t4 = 0.5 * t2 + t3,
		        t5 = 0.5 * t3 + t2;

	        // Calculate generalized strains
	        double
		        e1 = (d * u[0] + b * u[2] - d * u[4] - b * u[6]) / t1,
		        e2 = (-a * u[1] - c * u[3] + a * u[5] + c * u[7]) / t1,
		        e3 = (-a * u[0] + d * u[1] - c * u[2] + b * u[3] +
		              a * u[4] - d * u[5] + c * u[6] - b * u[7]) * 0.5 / t1,
		        e4 = (-u[0] + u[2] - u[4] + u[6]) * a / t4,
		        e5 = (u[1] - u[3] + u[5] - u[7]) * b / t5;

	        return
		        Vector<double>.Build.DenseOfArray(new[]
		        {
			        e1 - c / a * e4,
			        e2 - e5,
			        2 * (e3 + b / a * e4 + c / b * e5),
			        e1 + e4,
			        e2 + d / b * e5,
			        2 * (e3 - d / a * e4 - a / b * e5),
			        e1 + c / a * e4,
			        e2 + e5,
			        2 * (e3 - b / a * e4 - c / b * e5),
			        e1 - e4,
			        e2 - d / b * e5,
			        2 * (e3 + d / a * e4 * a / b * e5)
		        });
        }

        /// <summary>
        /// Calculate tangent stiffness.
        /// </summary>
        private Matrix<double> TangentStiffness()
		{
			// Get displacements
			var u = Displacements;

			// Set step size
			double d = 2E-10;

			// Calculate elements of matrix
			var K = Matrix<double>.Build.Dense(8, 8);
			for (int i = 0; i < 8; i++)
			{
				// Get row update vector
				var ud = CreateVector.Dense<double>(8);
				ud[i] = d;

				// Set displacements and do analysis
				SetDisplacements(u + ud);
				Analysis();

				// Get updated panel forces
				var fd1 = Forces;

				// Set displacements and do analysis
				SetDisplacements(u - ud);
				Analysis();

				// Get updated panel forces
				var fd2 = Forces;

				// Calculate ith column
				var km = 0.5 / d * (fd1 - fd2);

				// Set column
				K.SetColumn(i, km);
			}

			// Set displacements again
			SetDisplacements(u);

			return K;
		}

        /// <summary>
        /// Calculate initial stiffness matrix (alternate).
        /// </summary>
        private Matrix<double> InitialStiffness()
	    {
		    var (a, b, _, _) = Geometry.Dimensions;

		    // Calculate constants
		    double
			    a2  = a * a,
			    b2  = b * b,
			    ab  = a * b,
			    a_b = a / b,
			    b_a = b / a,
			    nu  = Concrete.nu,
			    t0  = Concrete.Ec * Geometry.Width / (1 - nu * nu),
			    t1  = 2 * ab / (a2 + 2 * b2),
			    t2  = 2 * ab / (2 * a2 + b2),
			    t3  =  b_a * (3 * a2 + 2 * b2) / (a2 + 2 * b2),
			    t4  =  a_b * (2 * a2 + 3 * b2) / (2 * a2 + b2),
			    t5  =  b_a * (a2 - 2 * b2)     / (a2 + 2 * b2),
			    t6  = -a_b * (2 * a2 - b2)     / (2 * a2 + b2);

		    return
			    t0 * Matrix<double>.Build.DenseOfArray(new [,]
			    {
				    {  t1,   0, -t1,   0,  t1,   0, -t1,   0 },
				    {   0,  t4, -nu, -t2,   0,  t6,  nu, -t2 },
				    { -t1, -nu,  t3,   0, -t1,  nu,  t5,   0 },
				    {   0, -t2,   0,  t2,   0, -t2,   0,  t2 },
				    {  t1,   0, -t1,   0,  t1,   0, -t1,   0 },
				    {   0,  t6,  nu, -t2,   0,  t4, -nu, -t2 },
				    { -t1,  nu,  t5,   0, -t1, -nu,  t3,   0 },
				    {   0, -t2,   0,  t2,   0, -t2,   0,  t2 }
			    });
	    }
    }
}
