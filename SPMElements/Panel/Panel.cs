using System;
using System.Collections.Generic;
using System.Linq;
using Autodesk.AutoCAD.DatabaseServices;
using Autodesk.AutoCAD.Geometry;
using Extensions.LinearAlgebra;
using Extensions.Number;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;
using Material.Concrete;
using Material.Concrete.Biaxial;
using Material.Reinforcement.Biaxial;
using MathNet.Numerics;
using OnPlaneComponents;
using RCMembrane;
using SPM.Elements.PanelProperties;
using UnitsNet;
using UnitsNet.Units;

namespace SPM.Elements
{
	/// <summary>
    /// Base panel class with linear properties.
    /// </summary>
	public class Panel : SPMElement, IEquatable<Panel>
	{
		// Auxiliary fields
		private Matrix<double> _transMatrix, _localStiffness;
		private Vector<double> _localForces;
		protected Vector<double> GlobalForces;

        /// <summary>
        /// Get <see cref="PanelGeometry"/> of this.
        /// </summary>
        public PanelGeometry Geometry { get; }

		/// <summary>
		/// Get <see cref="BiaxialConcrete"/> of this.
		/// </summary>
		public BiaxialConcrete Concrete { get; }

        /// <summary>
        /// Get <see cref="WebReinforcement"/> of this.
        /// </summary>
        public WebReinforcement Reinforcement { get; }

        /// <summary>
        /// Get the center <see cref="Node"/> of bottom edge.
        /// </summary>
        public Node Grip1 { get; }

        /// <summary>
        /// Get the center <see cref="Node"/> of right edge.
        /// </summary>
        public Node Grip2 { get; }

        /// <summary>
        /// Get the center <see cref="Node"/> of top edge.
        /// </summary>
        public Node Grip3 { get; }

        /// <summary>
        /// Get the center <see cref="Node"/> of left edge.
        /// </summary>
        public Node Grip4 { get; }

        /// <summary>
        /// Get transformation <see cref="Matrix"/>.
        /// </summary>
        private Matrix<double> TransformationMatrix => _transMatrix ?? CalculateTransformationMatrix();

        /// <summary>
        /// Get/set local stiffness <see cref="Matrix"/>.
        /// </summary>
        public virtual Matrix<double> LocalStiffness
        {
	        get => _localStiffness ?? CalculateStiffness();
	        set => _localStiffness = value;
        }

        /// <summary>
		/// Get global stiffness <see cref="Matrix"/>.
		/// </summary>
        public virtual Matrix<double> GlobalStiffness => TransformationMatrix.Transpose() * LocalStiffness * TransformationMatrix;

        /// <summary>
        /// Get/set global displacement <see cref="Vector"/>.
        /// </summary>
        public Vector<double> Displacements { get; protected set; }

        /// <summary>
        /// Get local force <see cref="Vector"/>.
        /// </summary>
        private Vector<double> LocalForces => _localForces ?? CalculateLocalForces();

        /// <summary>
        /// Get global force <see cref="Vector"/>.
        /// </summary>
        public virtual Vector<double> Forces => GlobalForces ?? CalculateGlobalForces();

        /// <summary>
        /// Get average <see cref="StressState"/>.
        /// </summary>
        public virtual StressState AverageStresses
        {
	        get
	        {
		        // Get the dimensions as a vector
		        var lsV = Geometry.EdgeLengths.ToVector();

		        // Calculate the shear stresses
		        var tau = LocalForces / (lsV * Geometry.Width);

		        // Calculate the average stress
		        double tauAvg = (-tau[0] + tau[1] - tau[2] + tau[3]) / 4;

		        return new StressState(0, 0, tauAvg);
	        }
        }

        /// <summary>
        /// Get average concrete <see cref="PrincipalStressState"/>.
        /// </summary>
        public virtual PrincipalStressState ConcretePrincipalStresses
        {
	        get
	        {
		        double sig2;

		        // Get shear stress
		        var tau = AverageStresses.TauXY;

		        // Get steel strengths
		        double
			        fyx = Reinforcement?.DirectionX?.Steel?.YieldStress ?? 0,
			        fyy = Reinforcement?.DirectionY?.Steel?.YieldStress ?? 0;

		        if (fyx.Approx(fyy))
			        sig2 = -2 * tau.Abs();

		        else
		        {
			        // Get relation of steel strengths
			        var rLambda = Math.Sqrt(fyx / fyy);
			        sig2 = -tau.Abs() * (rLambda + 1 / rLambda);
		        }

		        var theta1 = tau >= 0 ? Constants.PiOver4 : -Constants.PiOver4;

		        return new PrincipalStressState(0, sig2, theta1);
	        }
        }

        /// <summary>
        /// Get average <see cref="PrincipalStressState"/>.
        /// </summary>
        public PrincipalStressState AveragePrincipalStresses => PrincipalStressState.FromStress(AverageStresses);

        /// <summary>
        /// Get average concrete <see cref="PrincipalStrainState"/>.
        /// </summary>
        public virtual PrincipalStrainState ConcretePrincipalStrains { get; }

        /// <summary>
        /// Get the average crack opening in concrete, in mm.
        /// </summary>
        public virtual double CrackOpening { get; }

        /// <summary>
        /// Get the grip numbers of this.
        /// </summary>
        public int[] Grips => new[] { Grip1.Number, Grip2.Number, Grip3.Number, Grip4.Number };

        /// <summary>
        /// Get the DoF index of panel <see cref="Grips"/>.
        /// </summary>
        public override int[] DoFIndex => Indexes ?? GlobalIndexes(Grips);

		/// <summary>
        /// Get absolute maximum panel force.
        /// </summary>
        public double MaxForce => Forces.AbsoluteMaximum();

        /// <summary>
        /// Base panel object.
        /// </summary>
        /// <param name="objectId">The panel <see cref="ObjectId"/>.</param>
        /// <param name="number">The panel number.</param>
        /// <param name="grip1">The center <see cref="Node"/> of bottom edge</param>
        /// <param name="grip2">The center <see cref="Node"/> of right edge</param>
        /// <param name="grip3">The center <see cref="Node"/> of top edge</param>
        /// <param name="grip4">The center <see cref="Node"/> of left edge</param>
        /// <param name="vertices">Panel <see cref="Vertices"/> object.</param>
        /// <param name="width">Panel width.</param>
        /// <param name="concreteParameters">The concrete parameters <see cref="Parameters"/>.</param>
        /// <param name="model">The concrete <see cref="ConstitutiveModel"/>.</param>
        /// <param name="reinforcement">The <see cref="WebReinforcement"/>.</param>
        public Panel(ObjectId objectId, int number, Node grip1, Node grip2, Node grip3, Node grip4, Vertices vertices, Length width, Parameters concreteParameters, ConstitutiveModel model, WebReinforcement reinforcement = null)
			: base(objectId, number)
		{
			Grip1 = grip1;
			Grip2 = grip2;
			Grip3 = grip3;
			Grip4 = grip4;

			Geometry = new PanelGeometry(vertices, width);

			Concrete = new BiaxialConcrete(concreteParameters, model);

			Reinforcement = reinforcement;
		}

        /// <summary>
        /// Base panel object.
        /// </summary>
        /// <param name="objectId">The panel <see cref="ObjectId"/>.</param>
        /// <param name="number">The panel number.</param>
        /// <param name="grip1">The center <see cref="Node"/> of bottom edge</param>
        /// <param name="grip2">The center <see cref="Node"/> of right edge</param>
        /// <param name="grip3">The center <see cref="Node"/> of top edge</param>
        /// <param name="grip4">The center <see cref="Node"/> of left edge</param>
        /// <param name="vertices">Panel <see cref="Vertices"/> object.</param>
        /// <param name="width">Panel width, in <paramref name="unit"/>.</param>
        /// <param name="concreteParameters">The concrete parameters <see cref="Parameters"/>.</param>
        /// <param name="model">The concrete <see cref="ConstitutiveModel"/>.</param>
        /// <param name="reinforcement">The <see cref="WebReinforcement"/>.</param>
        /// <param name="unit">The <see cref="LengthUnit"/> of <paramref name="width"/> and <paramref name="vertices"/>' coordinates.</param>
        public Panel(ObjectId objectId, int number, Node grip1, Node grip2, Node grip3, Node grip4, Vertices vertices, double width, Parameters concreteParameters, ConstitutiveModel model, WebReinforcement reinforcement = null, LengthUnit unit = LengthUnit.Millimeter)
			: this (objectId, number, grip1, grip2, grip3, grip4, vertices, Length.From(width, unit), concreteParameters, model, reinforcement)
        {
        }

        /// <summary>
        /// Base panel object.
        /// </summary>
        /// <param name="objectId">The panel <see cref="ObjectId"/>.</param>
        /// <param name="number">The panel number.</param>
        /// <param name="grip1">The center <see cref="Node"/> of bottom edge</param>
        /// <param name="grip2">The center <see cref="Node"/> of right edge</param>
        /// <param name="grip3">The center <see cref="Node"/> of top edge</param>
        /// <param name="grip4">The center <see cref="Node"/> of left edge</param>
        /// <param name="vertices">The collection of <see cref="Point3d"/> panel vertices.</param>
        /// <param name="width">Panel width.</param>
        /// <param name="concreteParameters">The concrete parameters <see cref="Parameters"/>.</param>
        /// <param name="model">The concrete <see cref="ConstitutiveModel"/>.</param>
        /// <param name="reinforcement">The <see cref="WebReinforcement"/>.</param>
        public Panel(ObjectId objectId, int number, Node grip1, Node grip2, Node grip3, Node grip4, IEnumerable<Point3d> vertices, Length width, Parameters concreteParameters, ConstitutiveModel model, WebReinforcement reinforcement = null)
	        : this(objectId, number, grip1, grip2, grip3, grip4, new Vertices(vertices, width.Unit), width, concreteParameters, model, reinforcement)
        {
        }

        /// <summary>
        /// Base panel object.
        /// </summary>
        /// <param name="objectId">The panel <see cref="ObjectId"/>.</param>
        /// <param name="number">The panel number.</param>
        /// <param name="grip1">The center <see cref="Node"/> of bottom edge</param>
        /// <param name="grip2">The center <see cref="Node"/> of right edge</param>
        /// <param name="grip3">The center <see cref="Node"/> of top edge</param>
        /// <param name="grip4">The center <see cref="Node"/> of left edge</param>
        /// <param name="vertices">The collection of <see cref="Point3d"/> panel vertices.</param>
        /// <param name="width">Panel width, in <paramref name="unit"/>.</param>
        /// <param name="concreteParameters">The concrete parameters <see cref="Parameters"/>.</param>
        /// <param name="model">The concrete <see cref="ConstitutiveModel"/>.</param>
        /// <param name="reinforcement">The <see cref="WebReinforcement"/>.</param>
        /// <param name="unit">The <see cref="LengthUnit"/> of <paramref name="width"/> and <paramref name="vertices"/>' coordinates.</param>
        public Panel(ObjectId objectId, int number, Node grip1, Node grip2, Node grip3, Node grip4, IEnumerable<Point3d> vertices, double width, Parameters concreteParameters, ConstitutiveModel model, WebReinforcement reinforcement = null, LengthUnit unit = LengthUnit.Millimeter) 
	        : this(objectId, number, grip1, grip2, grip3, grip4, new Vertices(vertices, unit), Length.From(width, unit), concreteParameters, model, reinforcement)
        {
        }

        /// <summary>
        /// Base panel object.
        /// </summary>
        /// <param name="objectId">The panel <see cref="ObjectId"/>.</param>
        /// <param name="number">The panel number.</param>
        /// <param name="nodes">The collection containing all <see cref="Node"/>'s of SPM model.</param>
        /// <param name="vertices">Panel <see cref="Vertices"/> object.</param>
        /// <param name="width">Panel width.</param>
        /// <param name="concreteParameters">The concrete parameters <see cref="Parameters"/>.</param>
        /// <param name="model">The concrete <see cref="ConstitutiveModel"/>.</param>
        /// <param name="reinforcement">The <see cref="WebReinforcement"/>.</param>
        public Panel(ObjectId objectId, int number, IEnumerable<Node> nodes, Vertices vertices, Length width, Parameters concreteParameters, ConstitutiveModel model, WebReinforcement reinforcement = null)
	        : base(objectId, number)
        {
	        Geometry = new PanelGeometry(vertices, width);

	        Grip1 = nodes.GetByPosition(Geometry.Edge1.CenterPoint);
	        Grip2 = nodes.GetByPosition(Geometry.Edge2.CenterPoint);
            Grip3 = nodes.GetByPosition(Geometry.Edge3.CenterPoint);
	        Grip4 = nodes.GetByPosition(Geometry.Edge4.CenterPoint);
			
            Concrete = new BiaxialConcrete(concreteParameters, model);

	        Reinforcement = reinforcement;
        }

        /// <summary>
        /// Base panel object.
        /// </summary>
        /// <param name="objectId">The panel <see cref="ObjectId"/>.</param>
        /// <param name="number">The panel number.</param>
        /// <param name="nodes">The collection containing all <see cref="Node"/>'s of SPM model.</param>
        /// <param name="vertices">Panel <see cref="Vertices"/> object.</param>
        /// <param name="width">Panel width, in <paramref name="unit"/>.</param>
        /// <param name="concreteParameters">The concrete parameters <see cref="Parameters"/>.</param>
        /// <param name="model">The concrete <see cref="ConstitutiveModel"/>.</param>
        /// <param name="reinforcement">The <see cref="WebReinforcement"/>.</param>
        public Panel(ObjectId objectId, int number, IEnumerable<Node> nodes, Vertices vertices, double width, Parameters concreteParameters, ConstitutiveModel model, WebReinforcement reinforcement = null, LengthUnit unit = LengthUnit.Millimeter)
	        : this (objectId, number, nodes, vertices, Length.From(width, unit), concreteParameters, model, reinforcement)
        {
        }

        /// <summary>
        /// Base panel object.
        /// </summary>
        /// <param name="objectId">The panel <see cref="ObjectId"/>.</param>
        /// <param name="number">The panel number.</param>
        /// <param name="nodes">The collection containing all <see cref="Node"/>'s of SPM model.</param>
        /// <param name="vertices">The collection of <see cref="Point3d"/> panel vertices.</param>
        /// <param name="width">Panel width.</param>
        /// <param name="concreteParameters">The concrete parameters <see cref="Parameters"/>.</param>
        /// <param name="model">The concrete <see cref="ConstitutiveModel"/>.</param>
        /// <param name="reinforcement">The <see cref="WebReinforcement"/>.</param>
        public Panel(ObjectId objectId, int number, IEnumerable<Node> nodes, IEnumerable<Point3d> vertices, Length width, Parameters concreteParameters, ConstitutiveModel model, WebReinforcement reinforcement = null)
	        : this(objectId, number, nodes, new Vertices(vertices, width.Unit), width, concreteParameters, model, reinforcement) 
        {
        }

        /// <summary>
        /// Base panel object.
        /// </summary>
        /// <param name="objectId">The panel <see cref="ObjectId"/>.</param>
        /// <param name="number">The panel number.</param>
        /// <param name="nodes">The collection containing all <see cref="Node"/>'s of SPM model.</param>
        /// <param name="vertices">The collection of <see cref="Point3d"/> panel vertices.</param>
        /// <param name="width">Panel width, in <paramref name="unit"/>.</param>
        /// <param name="concreteParameters">The concrete parameters <see cref="Parameters"/>.</param>
        /// <param name="model">The concrete <see cref="ConstitutiveModel"/>.</param>
        /// <param name="reinforcement">The <see cref="WebReinforcement"/>.</param>
        public Panel(ObjectId objectId, int number, IEnumerable<Node> nodes, IEnumerable<Point3d> vertices, double width, Parameters concreteParameters, ConstitutiveModel model, WebReinforcement reinforcement = null, LengthUnit unit = LengthUnit.Millimeter)
	        : this (objectId, number, nodes, new Vertices(vertices, unit), Length.From(width, unit), concreteParameters, model, reinforcement)
        {
        }

        /// <summary>
        /// Set panel displacements from global displacement vector.
        /// </summary>
        protected void SetDisplacements(Vector<double> globalDisplacementVector)
        {
	        var u = globalDisplacementVector;
	        var ind = DoFIndex;

	        // Get the displacements
	        var up = Vector<double>.Build.Dense(8);
	        for (var i = 0; i < ind.Length; i++)
	        {
		        // Indexers
		        var j = ind[i];

		        // Set values
		        up[i] = u[j];
	        }

	        // Set
	        Displacements = up;
        }

        /// <summary>
        /// Set stringer dimensions on edges.
        /// <para>See: <see cref="Edge.SetStringerDimension"/></para>
        /// </summary>
        /// <param name="stringers">The collection containing all of the stringers.</param>
        public void SetEdgeStringersDimensions(IEnumerable<Stringer> stringers)
        {
	        if (stringers is null)
		        return;

	        // Initiate the Stringer dimensions
	        var hs = new double[4];

	        // Analyze panel grips
	        for (int i = 0; i < 4; i++)
		        hs[i] = 0.5 * stringers.First(str => Grips[i] == str.Grips[1]).Geometry.Height;

	        // Set on edges
	        Geometry.Edge1.SetStringerDimension(hs[0]);
	        Geometry.Edge2.SetStringerDimension(hs[1]);
	        Geometry.Edge3.SetStringerDimension(hs[2]);
	        Geometry.Edge4.SetStringerDimension(hs[3]);
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
            _transMatrix = new[,]
            {
                {m1, n1,  0,  0,  0,  0,  0,  0},
                { 0,  0, m2, n2,  0,  0,  0,  0},
                { 0,  0,  0,  0, m3, n3,  0,  0},
                { 0,  0,  0,  0,  0,  0, m4, n4}
            }.ToMatrix();

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
                Gc * w * new[,]
                {
                    {a_b,  -1, a_b,  -1},
                    { -1, b_a,  -1, b_a},
                    {a_b,  -1, a_b,  -1},
                    { -1, b_a,  -1, b_a}
                }.ToMatrix();
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
                w = Geometry.Width,
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
            var km1 = new[,]
            {
                {c2, c3, c4},
                {s2, s3, s4},
                {r2, r3, r4},
            }.ToMatrix();

            var km2 = new[,]
            {
                {c1, c3, c4},
                {s1, s3, s4},
                {r1, r3, r4},
            }.ToMatrix();

            var km3 = new[,]
            {
                {c1, c2, c4},
                {s1, s2, s4},
                {r1, r2, r4},
            }.ToMatrix();

            var km4 = new[,]
            {
                {c1, c2, c3},
                {s1, s2, s3},
                {r1, r2, r3},
            }.ToMatrix();

            // Calculate the determinants
            double
                k1 = km1.Determinant(),
                k2 = km2.Determinant(),
                k3 = km3.Determinant(),
                k4 = km4.Determinant();

            // Calculate kf and ku
            double
                kf = k1 + k2 + k3 + k4,
                ku = -t1 * k1 + t2 * k2 - t3 * k3 + t4 * k4;

            // Calculate D
            double D = 16 * Gc * w / (kf * ku);

            // Get the vector B
            var B = new[]
            {
                -k1 * l1, k2 * l2, -k3 * l3, k4 * l4
            }.ToVector();

            // Get the stiffness matrix
            return
                B.ToColumnMatrix() * D * B.ToRowMatrix();
        }

        /// <summary>
        /// Calculate panel local forces.
        /// </summary>
        private Vector<double> CalculateLocalForces()
        {
            // Get the parameters
            var up = Displacements;
            var T = TransformationMatrix;
            var Kl = LocalStiffness;

            // Get the displacements in the direction of the edges
            var ul = T * up;

            // Calculate the vector of forces
            _localForces = Kl * ul;

            // Aproximate small values to zero
            _localForces.CoerceZero(1E-6);

            return _localForces;
        }

        /// <summary>
        /// Calculate panel global forces.
        /// </summary>
        private Vector<double> CalculateGlobalForces()
        {
	        GlobalForces = TransformationMatrix.Transpose() * LocalForces;

	        return GlobalForces;
        }

        /// <summary>
        /// Set displacements and calculate forces.
        /// </summary>
        /// <param name="globalDisplacements">The global displacement <see cref="Vector"/>.</param>
        public virtual void Analysis(Vector<double> globalDisplacements = null)
        {
            // Set displacements
            if (globalDisplacements != null)
                SetDisplacements(globalDisplacements);
        }

        /// <summary>
        /// Return a panel object based on type of analysis.
        /// <para>See: <seealso cref="AnalysisType"/>.</para>
        /// </summary>
        /// <param name="analysisType">The <see cref="AnalysisType"/>.</param>
        /// <param name="objectId">The panel <see cref="ObjectId"/>.</param>
        /// <param name="number">The panel number.</param>
        /// <param name="grip1">The center <see cref="Node"/> of bottom edge</param>
        /// <param name="grip2">The center <see cref="Node"/> of right edge</param>
        /// <param name="grip3">The center <see cref="Node"/> of top edge</param>
        /// <param name="grip4">The center <see cref="Node"/> of left edge</param>
        /// <param name="vertices">Panel <see cref="Vertices"/> object.</param>
        /// <param name="width">Panel width, in <paramref name="unit"/>.</param>
        /// <param name="concreteParameters">The concrete parameters <see cref="Parameters"/>.</param>
        /// <param name="model">The concrete <see cref="ConstitutiveModel"/>.</param>
        /// <param name="reinforcement">The <see cref="WebReinforcement"/>.</param>
        /// <param name="unit">The <see cref="LengthUnit"/> of <paramref name="width"/> and <paramref name="vertices"/>' coordinates.</param>
        public static Panel Read(AnalysisType analysisType, ObjectId objectId, int number, Node grip1, Node grip2, Node grip3, Node grip4, Vertices vertices, double width, Parameters concreteParameters, ConstitutiveModel model, WebReinforcement reinforcement = null, LengthUnit unit = LengthUnit.Millimeter) => 
	        analysisType is AnalysisType.Linear 
		        ? new Panel(objectId, number, grip1, grip2, grip3, grip4, vertices, width, concreteParameters, model, reinforcement, unit) 
		        : new NLPanel(objectId, number, grip1, grip2, grip3, grip4, vertices, width, concreteParameters, model, reinforcement, unit);

        /// <summary>
        /// Return a panel object based on type of analysis.
        /// <para>See: <seealso cref="AnalysisType"/>.</para>
        /// </summary>
        /// <param name="analysisType">The <see cref="AnalysisType"/>.</param>
        /// <param name="objectId">The panel <see cref="ObjectId"/>.</param>
        /// <param name="number">The panel number.</param>
        /// <param name="grip1">The center <see cref="Node"/> of bottom edge</param>
        /// <param name="grip2">The center <see cref="Node"/> of right edge</param>
        /// <param name="grip3">The center <see cref="Node"/> of top edge</param>
        /// <param name="grip4">The center <see cref="Node"/> of left edge</param>
        /// <param name="vertices">The collection of <see cref="Point3d"/> panel vertices.</param>
        /// <param name="width">Panel width, in <paramref name="unit"/>.</param>
        /// <param name="concreteParameters">The concrete parameters <see cref="Parameters"/>.</param>
        /// <param name="model">The concrete <see cref="ConstitutiveModel"/>.</param>
        /// <param name="reinforcement">The <see cref="WebReinforcement"/>.</param>
        /// <param name="unit">The <see cref="LengthUnit"/> of <paramref name="width"/> and <paramref name="vertices"/>' coordinates.</param>
        public static Panel Read(AnalysisType analysisType, ObjectId objectId, int number, Node grip1, Node grip2, Node grip3, Node grip4, IEnumerable<Point3d> vertices, double width, Parameters concreteParameters, ConstitutiveModel model, WebReinforcement reinforcement = null, LengthUnit unit = LengthUnit.Millimeter) => 
	        analysisType is AnalysisType.Linear 
		        ? new Panel(objectId, number, grip1, grip2, grip3, grip4, vertices, width, concreteParameters, model, reinforcement, unit) 
		        : new NLPanel(objectId, number, grip1, grip2, grip3, grip4, vertices, width, concreteParameters, model, reinforcement, unit);

        /// <summary>
        /// Return a panel object based on type of analysis.
        /// <para>See: <seealso cref="AnalysisType"/>.</para>
        /// </summary>
        /// <param name="analysisType">The <see cref="AnalysisType"/>.</param>
        /// <param name="objectId">The panel <see cref="ObjectId"/>.</param>
        /// <param name="number">The panel number.</param>
        /// <param name="nodes">The collection containing all <see cref="Node"/>'s of SPM model.</param>
        /// <param name="vertices">The collection of <see cref="Point3d"/> panel vertices.</param>
        /// <param name="width">Panel width.</param>
        /// <param name="concreteParameters">The concrete parameters <see cref="Parameters"/>.</param>
        /// <param name="model">The concrete <see cref="ConstitutiveModel"/>.</param>
        /// <param name="reinforcement">The <see cref="WebReinforcement"/>.</param>
        /// <param name="unit">The <see cref="LengthUnit"/> of <paramref name="width"/> and <paramref name="vertices"/>' coordinates.</param>
        public static Panel Read(AnalysisType analysisType, ObjectId objectId, int number, IEnumerable<Node> nodes, IEnumerable<Point3d> vertices, double width, Parameters concreteParameters, ConstitutiveModel model, WebReinforcement reinforcement = null, LengthUnit unit = LengthUnit.Millimeter) => 
	        analysisType is AnalysisType.Linear 
		        ? new Panel(objectId, number, nodes, vertices, width, concreteParameters, model, reinforcement, unit) 
		        : new NLPanel(objectId, number, nodes, vertices, width, concreteParameters, model, reinforcement, unit);

        /// <summary>
        /// Returns true if <paramref name="other"/>'s <see cref="Geometry"/> is equal.
        /// </summary>
        /// <param name="other">The other <see cref="Panel"/> object to compare.</param>
		public bool Equals(Panel other) => other != null && Geometry == other.Geometry;

		/// <summary>
		/// Returns true if <paramref name="obj"/> is <see cref="Panel"/> and <see cref="Geometry"/> is equal.
		/// </summary>
		/// <param name="obj">The other <see cref="object"/> to compare.</param>
		public override bool Equals(object obj) => obj is Panel other && Equals(other);

		public override int GetHashCode() => Geometry.GetHashCode();

		public override string ToString()
		{
			var msgstr =
				$"Panel {Number}\n\n" +
				$"Grips: ({Grips[0]} - {Grips[1]} - {Grips[2]} - {Grips[3]})\n" +
				$"{Geometry}";

			if (!(Reinforcement is null))
				msgstr += $"\n\n{Reinforcement}";

			return msgstr;
		}

		/// <summary>
		/// Returns true if arguments are equal.
		/// <para>See:<seealso cref="Equals(Panel)"/>.</para>
		/// </summary>
		public static bool operator == (Panel left, Panel right) => left != null && left.Equals(right);

        /// <summary>
        /// Returns true if arguments are different.
        /// <para>See:<seealso cref="Equals(Panel)"/>.</para>
        /// </summary>
        public static bool operator != (Panel left, Panel right) => left != null && !left.Equals(right);

    }
}