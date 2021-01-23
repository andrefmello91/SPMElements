using System;
using System.Collections.Generic;
using Autodesk.AutoCAD.DatabaseServices;
using Autodesk.AutoCAD.Geometry;
using Extensions;
using Extensions.LinearAlgebra;
using Extensions.Number;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;
using Material.Reinforcement.Uniaxial;
using Material.Concrete.Uniaxial;
using SPM.Elements.StringerProperties;
using UnitsNet;
using UnitsNet.Units;
using ConstitutiveModel = Material.Concrete.ConstitutiveModel;
using Parameters = Material.Concrete.Parameters;

namespace SPM.Elements
{
	/// <summary>
    /// Stringer base class with linear properties.
    /// </summary>
	public class Stringer : SPMElement, IEquatable<Stringer>
    {
		/// <summary>
        /// Type of forces that stringer can be loaded.
        /// </summary>
		public enum ForceState
		{
			Unloaded,
			PureTension,
			PureCompression,
			Combined
		}

		// Auxiliary fields
		protected Matrix<double> TransMatrix, LocStiffness;
		private UniaxialReinforcement _reinforcement;

        /// <summary>
        /// Event to run when <see cref="Geometry"/> width or height changes.
        /// </summary>
        public EventHandler<ParameterChangedEventArgs<double>> GeometryChanged;

        /// <summary>
        /// Event to run when <see cref="Reinforcement"/> changes.
        /// </summary>
        public EventHandler<ParameterChangedEventArgs<UniaxialReinforcement>> ReinforcementChanged;

        /// <summary>
        /// Get the initial <see cref="Node"/> of this.
        /// </summary>
        public Node Grip1 { get; }

		/// <summary>
		/// Get the center <see cref="Node"/> of this.
		/// </summary>
		public Node Grip2 { get; }

		/// <summary>
		/// Get the end <see cref="Node"/> of this.
		/// </summary>
		public Node Grip3 { get; }

        /// <summary>
        /// Get/set the <see cref="Geometry"/> of this.
        /// </summary>
        public StringerGeometry Geometry { get; }

        /// <summary>
        /// Get/set the <see cref="UniaxialConcrete"/> of this.
        /// </summary>
        public UniaxialConcrete Concrete { get; }

        /// <summary>
        /// Get the <see cref="UniaxialReinforcement"/> of this.
        /// </summary>
        public UniaxialReinforcement Reinforcement
        {
	        get => _reinforcement;
	        set
	        {
		        var old = _reinforcement.Copy();

		        _reinforcement = value;

                if (old != value)
                    ReinforcementChanged?.Invoke(this, new ParameterChangedEventArgs<UniaxialReinforcement>(old, value));
	        }
        }

        /// <summary>
        /// Get local stiffness <see cref="Matrix"/>.
        /// </summary>
        public virtual Matrix<double> LocalStiffness => LocStiffness ?? CalculateStiffness();

        /// <summary>
        /// Get/set local force <see cref="Vector"/>.
        /// </summary>
        protected virtual Vector<double> LocalForces { get; set; }

        /// <summary>
        /// Get/set global displacement <see cref="Vector"/>.
        /// </summary>
        public Vector<double> Displacements { get; protected set; }

        /// <summary>
        /// Get the grip numbers of this.
        /// </summary>
        public int[] Grips => new[] { Grip1.Number, Grip2.Number, Grip3.Number };

        /// <summary>
        /// Get the transformation <see cref="Matrix"/>.
        /// </summary>
        public Matrix<double> TransformationMatrix => TransMatrix ?? CalculateTransformationMatrix();

        /// <summary>
        /// Get the DoF index of stringer <see cref="Grips"/>.
        /// </summary>
        public override int[] DoFIndex => Indexes ?? GetIndexes(Grips);

        /// <summary>
        /// Get global stiffness <see cref="Matrix"/>.
        /// </summary>
		public Matrix<double> GlobalStiffness => TransformationMatrix.Transpose() * LocalStiffness * TransformationMatrix;

		/// <summary>
		/// Get local displacement <see cref="Vector"/>.
		/// </summary>
		public Vector<double> LocalDisplacements => TransformationMatrix * Displacements;

		/// <summary>
        /// Get normal forces acting in the stringer, in N.
        /// </summary>
		public (double N1, double N3) NormalForces => (LocalForces[0], -LocalForces[2]);

		/// <summary>
		/// Get the state of forces acting on the stringer.
		/// </summary>
		public ForceState State
		{
			get
			{
				var (N1, N3) = NormalForces;

				if (N1 == 0 && N3 == 0)
					return ForceState.Unloaded;

				if (N1 > 0 && N3 > 0)
					return ForceState.PureTension;

				if (N1 < 0 && N3 < 0)
					return ForceState.PureCompression;

				return ForceState.Combined;
			}
		}

		/// <summary>
        /// Get stringer global force <see cref="Vector"/>.
        /// </summary>
        public Vector<double> Forces => TransformationMatrix.Transpose() * LocalForces;

        /// <summary>
        /// Get absolute maximum stringer force.
        /// </summary>
        public double MaxForce => LocalForces.AbsoluteMaximum();

        /// <summary>
        /// Get crack openings in start, mid and end nodes.
        /// </summary>
        public virtual double[] CrackOpenings { get; }

        /// <summary>
        /// Stringer object.
        /// </summary>
        /// <param name="objectId">The stringer <see cref="ObjectId"/>.</param>
        /// <param name="number">The stringer number.</param>
        /// <param name="grip1">The initial <see cref="Node"/> of the <see cref="Stringer"/>.</param>
        /// <param name="grip2">The center <see cref="Node"/> of the <see cref="Stringer"/>.</param>
        /// <param name="grip3">The final <see cref="Node"/> of the <see cref="Stringer"/>.</param>
        /// <param name="width">The stringer width, in <paramref name="unit"/> considered.</param>
        /// <param name="height">The stringer height, in <paramref name="unit"/> considered.</param>
        /// <param name="concreteParameters">The concrete parameters <see cref="Parameters"/>.</param>
        /// <param name="model">The concrete <see cref="ConstitutiveModel"/>.</param>
        /// <param name="reinforcement">The <see cref="UniaxialReinforcement"/> of this stringer.</param>
        /// <param name="unit">The <see cref="LengthUnit"/> of <paramref name="width"/> and <paramref name="height"/>.
        /// <para>Default: <seealso cref="LengthUnit.Millimeter"/>.</para></param>
        public Stringer(ObjectId objectId, int number, Node grip1, Node grip2, Node grip3, double width, double height, Parameters concreteParameters, ConstitutiveModel model, UniaxialReinforcement reinforcement = null, LengthUnit unit = LengthUnit.Millimeter)
			: this (objectId, number, grip1, grip2, grip3, Length.From(width, unit), Length.From(height, unit), concreteParameters, model, reinforcement)
        {
        }

        /// <summary>
        /// Stringer object.
        /// </summary>
        /// <param name="objectId">The stringer <see cref="ObjectId"/>.</param>
        /// <param name="number">The stringer number.</param>
        /// <param name="grip1">The initial <see cref="Node"/> of the <see cref="Stringer"/>.</param>
        /// <param name="grip2">The center <see cref="Node"/> of the <see cref="Stringer"/>.</param>
        /// <param name="grip3">The final <see cref="Node"/> of the <see cref="Stringer"/>.</param>
        /// <param name="width">The stringer width.</param>
        /// <param name="height">The stringer height.</param>
        /// <param name="concreteParameters">The concrete parameters <see cref="Parameters"/>.</param>
        /// <param name="model">The concrete <see cref="ConstitutiveModel"/>.</param>
        /// <param name="reinforcement">The <see cref="UniaxialReinforcement"/> of this stringer.</param>
        public Stringer(ObjectId objectId, int number, Node grip1, Node grip2, Node grip3, Length width, Length height, Parameters concreteParameters, ConstitutiveModel model, UniaxialReinforcement reinforcement = null) : base(objectId, number)
        {
	        Geometry       = GetGeometry(grip1.Position, grip3.Position, width, height);
	        Grip1          = grip1;
	        Grip2          = grip2;
	        Grip3          = grip3;
	        _reinforcement = reinforcement;
	        Concrete       = new UniaxialConcrete(concreteParameters, Geometry.Area, model);
        }

        /// <summary>
        /// Stringer object.
        /// </summary>
        /// <param name="objectId">The stringer <see cref="ObjectId"/>.</param>
        /// <param name="number">The stringer number.</param>
        /// <param name="nodes">The collection containing all <see cref="Node"/>'s of SPM model.</param>
        /// <param name="grip1Position">The position of initial <see cref="Node"/> of the <see cref="Stringer"/>.</param>
        /// <param name="grip3Position">The position of final <see cref="Node"/> of the <see cref="Stringer"/>.</param>
        /// <param name="width">The stringer width, in mm.</param>
        /// <param name="height">The stringer height, in mm.</param>
        /// <param name="concreteParameters">The concrete parameters <see cref="Parameters"/>.</param>
        /// <param name="model">The concrete <see cref="ConstitutiveModel"/>.</param>
        /// <param name="reinforcement">The <see cref="UniaxialReinforcement"/> of this stringer.</param>
        public Stringer(ObjectId objectId, int number, IEnumerable<Node> nodes, Point3d grip1Position, Point3d grip3Position, double width, double height, Parameters concreteParameters, ConstitutiveModel model, UniaxialReinforcement reinforcement = null, LengthUnit unit = LengthUnit.Millimeter)
	        : this(objectId, number, nodes, grip1Position, grip3Position, Length.From(width, unit), Length.From(height, unit), concreteParameters, model, reinforcement)
        {
        }

        /// <summary>
        /// Stringer object.
        /// </summary>
        /// <param name="objectId">The stringer <see cref="ObjectId"/>.</param>
        /// <param name="number">The stringer number.</param>
        /// <param name="nodes">The collection containing all <see cref="Node"/>'s of SPM model.</param>
        /// <param name="grip1Position">The position of initial <see cref="Node"/> of the <see cref="Stringer"/>.</param>
        /// <param name="grip3Position">The position of final <see cref="Node"/> of the <see cref="Stringer"/>.</param>
        /// <param name="width">The stringer width.</param>
        /// <param name="height">The stringer height.</param>
        /// <param name="concreteParameters">The concrete parameters <see cref="Parameters"/>.</param>
        /// <param name="model">The concrete <see cref="ConstitutiveModel"/>.</param>
        /// <param name="reinforcement">The <see cref="UniaxialReinforcement"/> of this stringer.</param>
        public Stringer(ObjectId objectId, int number, IEnumerable<Node> nodes, Point3d grip1Position, Point3d grip3Position, Length width, Length height, Parameters concreteParameters, ConstitutiveModel model, UniaxialReinforcement reinforcement = null) : base(objectId, number)
        {
	        Geometry       = GetGeometry(grip1Position, grip3Position, width, height);
            Grip1          = nodes.GetByPosition(grip1Position);
	        Grip2          = nodes.GetByPosition(Geometry.CenterPoint);
	        Grip3          = nodes.GetByPosition(grip3Position);
	        _reinforcement = reinforcement;
	        Concrete       = new UniaxialConcrete(concreteParameters, Geometry.Area, model);
        }

        /// <summary>
        /// Get the initial <see cref="Geometry"/>.
        /// </summary>
        private StringerGeometry GetGeometry(Point3d initialPoint, Point3d endPoint, Length width, Length height)
        {
	        var geometry = new StringerGeometry(initialPoint, endPoint, width, height);
	        geometry.GeometryChanged += On_GeometryChange;
	        return geometry;
        }

        /// <summary>
        /// Calculate the transformation matrix.
        /// </summary>
        private Matrix<double> CalculateTransformationMatrix()
        {
	        // Get the direction cosines
	        var (l, m) = Geometry.Angle.DirectionCosines();

	        // Obtain the transformation matrix
	        TransMatrix = new [,]
	        {
		        {l, m, 0, 0, 0, 0 },
		        {0, 0, l, m, 0, 0 },
		        {0, 0, 0, 0, l, m }
	        }.ToMatrix();

	        return TransMatrix;
        }

        /// <summary>
        /// Set Stringer displacements from global displacement vector.
        /// </summary>
        public void SetDisplacements(Vector<double> globalDisplacementVector)
        {
	        var u = globalDisplacementVector;
	        int[] ind = DoFIndex;

	        // Get the displacements
	        var us = Vector<double>.Build.Dense(6);
	        for (int i = 0; i < ind.Length; i++)
	        {
		        // Global index
		        int j = ind[i];

		        // Set values
		        us[i] = u[j];
	        }

	        // Set
	        Displacements = us;
        }

        /// <summary>
        /// Calculate local stiffness <see cref="Matrix"/>.
        /// </summary>
        /// <returns></returns>
        protected Matrix<double> CalculateStiffness()
        {
	        // Calculate the constant factor of stiffness
	        var k = Concrete.Stiffness / (3 * Geometry.Length);

	        // Calculate the local stiffness matrix
	        LocStiffness =
		        k * new double[,]
		        {
			        {  7, -8,  1 },
			        { -8, 16, -8 },
			        {  1, -8,  7 }
		        }.ToMatrix();

	        return LocStiffness;
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

        /// <summary>
        /// Do analysis of stringer.
        /// </summary>
        /// <param name="globalDisplacements">The global displacement vector.</param>
        /// <param name="numStrainSteps">The number of strain increments (for nonlinear analysis) (default: 5).</param>
        public virtual void Analysis(Vector<double> globalDisplacements = null, int numStrainSteps = 5)
		{
			// Set displacements
			if (globalDisplacements != null)
				SetDisplacements(globalDisplacements);

			LocalForces = CalculateForces();
		}

        /// <summary>
        /// Read the stringer based on <see cref="AnalysisType"/>.
        /// </summary>
        /// <param name="analysisType">Type of analysis to perform (<see cref="AnalysisType"/>).</param>
        /// <param name="objectId">The stringer <see cref="ObjectId"/>.</param>
        /// <param name="number">The stringer number.</param>
        /// <param name="nodes">The collection containing all <see cref="Node"/>'s of SPM model.</param>
        /// <param name="grip1Position">The position of initial <see cref="Node"/> of the <see cref="Stringer"/>.</param>
        /// <param name="grip3Position">The position of final <see cref="Node"/> of the <see cref="Stringer"/>.</param>
        /// <param name="width">The stringer width.</param>
        /// <param name="height">The stringer height.</param>
        /// <param name="concreteParameters">The concrete parameters <see cref="Parameters"/>.</param>
        /// <param name="model">The concrete <see cref="ConstitutiveModel"/>.</param>
        /// <param name="reinforcement">The <see cref="UniaxialReinforcement"/>.</param>
        /// <param name="geometryUnit">The <see cref="LengthUnit"/> of <paramref name="width"/> and <paramref name="height"/>.<para>Default: <seealso cref="LengthUnit.Millimeter"/>.</para></param>
		public static Stringer Read(AnalysisType analysisType, ObjectId objectId, int number, IEnumerable<Node> nodes, Point3d grip1Position, Point3d grip3Position, double width, double height, Parameters concreteParameters, ConstitutiveModel model, UniaxialReinforcement reinforcement = null, LengthUnit geometryUnit = LengthUnit.Millimeter) => 
	        analysisType is AnalysisType.Linear 
		        ? new Stringer(objectId, number, nodes, grip1Position, grip3Position, width, height, concreteParameters, model, reinforcement, geometryUnit) 
		        : new NLStringer(objectId, number, nodes, grip1Position, grip3Position, width, height, concreteParameters, model, reinforcement, geometryUnit);

        /// <summary>
        /// Read the stringer based on <see cref="AnalysisType"/>.
        /// </summary>
        /// <param name="analysisType">Type of analysis to perform (<see cref="AnalysisType"/>).</param>
        /// <param name="objectId">The stringer <see cref="ObjectId"/>.</param>
        /// <param name="number">The stringer number.</param>
        /// <param name="initialNode">The initial <see cref="Node"/> of the <see cref="Stringer"/>.</param>
        /// <param name="centerNode">The center <see cref="Node"/> of the <see cref="Stringer"/>.</param>
        /// <param name="finalNode">The final <see cref="Node"/> of the <see cref="Stringer"/>.</param>
        /// <param name="width">The stringer width.</param>
        /// <param name="height">The stringer height.</param>
        /// <param name="concreteParameters">The concrete parameters <see cref="Parameters"/>.</param>
        /// <param name="model">The concrete <see cref="ConstitutiveModel"/>.</param>
        /// <param name="reinforcement">The <see cref="UniaxialReinforcement"/>.</param>
        /// <param name="geometryUnit">The <see cref="LengthUnit"/> of <paramref name="width"/> and <paramref name="height"/>.<para>Default: <seealso cref="LengthUnit.Millimeter"/>.</para></param>
		public static Stringer Read(AnalysisType analysisType, ObjectId objectId, int number, Node initialNode, Node centerNode, Node finalNode, double width, double height, Parameters concreteParameters, ConstitutiveModel model, UniaxialReinforcement reinforcement = null, LengthUnit geometryUnit = LengthUnit.Millimeter) => 
	        analysisType is AnalysisType.Linear 
		        ? new Stringer(objectId, number, initialNode, centerNode, finalNode, width, height, concreteParameters, model, reinforcement, geometryUnit) 
		        : new NLStringer(objectId, number, initialNode, centerNode, finalNode, width, height, concreteParameters, model, reinforcement, geometryUnit);

        /// <summary>
        /// Returns true if <see cref="Geometry"/> of <paramref name="other"/> is equal to this.
        /// </summary>
        /// <param name="other"></param>
		public bool Equals(Stringer other) => !(other is null) && Geometry == other.Geometry;

        /// <inheritdoc/>
        public override bool Equals(SPMElement other) => other is Stringer stringer && Equals(stringer);

        /// <summary>
        /// Returns true if <paramref name="obj"/> is <see cref="Stringer"/> and <see cref="Geometry"/> of <paramref name="obj"/> is equal to this.
        /// </summary>
        public override bool Equals(object obj) => obj is Stringer other && Equals(other);

		public override int GetHashCode() => Geometry.GetHashCode();

		public override string ToString()
		{
			string msgstr =
				$"Stringer {Number}\n\n" +
				$"Grips: ({Grips[0]} - {Grips[1]} - {Grips[2]})\n" +
				$"{Geometry}";

			if (Reinforcement != null)
			{
				msgstr += $"\n\n{Reinforcement}";
			}

			return msgstr;
		}

        /// <summary>
        /// Execute <see cref="GeometryChanged"/> event.
        /// </summary>
		private void On_GeometryChange(object sender, ParameterChangedEventArgs<double> e)
		{
            GeometryChanged?.Invoke(this, new ParameterChangedEventArgs<double>(e.OldValue, e.NewValue));
		}

		/// <summary>
        /// Returns true if arguments are equal.
        /// <para>See:<seealso cref="Equals(Stringer)"/>.</para>
        /// </summary>
		public static bool operator == (Stringer left, Stringer right) => left != null && left.Equals(right);

		/// <summary>
		/// Returns true if arguments are different.
		/// <para>See:<seealso cref="Equals(Stringer)"/>.</para>
		/// </summary>
		public static bool operator != (Stringer left, Stringer right) => left != null && !left.Equals(right);
    }
}

