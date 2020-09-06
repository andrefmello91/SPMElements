using System;
using Autodesk.AutoCAD.DatabaseServices;
using Autodesk.AutoCAD.Geometry;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;
using Material.Concrete;
using Material.Reinforcement;
using OnPlaneComponents;
using UnitsNet;
using UnitsNet.Units;
using SPMElements.PanelProperties;

namespace SPMElements
{
	/// <summary>
    /// Base panel class.
    /// </summary>
	public class Panel : SPMElement, IEquatable<Panel>
	{
		/// <summary>
        /// Get <see cref="PanelGeometry"/> of this.
        /// </summary>
		public PanelGeometry Geometry { get; }

		/// <summary>
		/// Get <see cref="BiaxialConcrete"/> of this.
		/// </summary>
		public BiaxialConcrete Concrete { get; }

        /// <summary>
        /// Get <see cref="BiaxialReinforcement"/> of this.
        /// </summary>
        public BiaxialReinforcement Reinforcement { get; }

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
        /// Get/set local stiffness <see cref="Matrix"/>.
        /// </summary>
		public virtual Matrix<double> LocalStiffness { get; set; }

		/// <summary>
		/// Get global stiffness <see cref="Matrix"/>.
		/// </summary>
		public virtual Matrix<double> GlobalStiffness { get; }

		/// <summary>
		/// Get/set global displacement <see cref="Vector"/>.
		/// </summary>
		public Vector<double> Displacements { get; set; }

		/// <summary>
		/// Get/set global force <see cref="Vector"/>.
		/// </summary>
		public Vector<double> Forces { get; set; }

		/// <summary>
		/// Get average <see cref="StressState"/>.
		/// </summary>
		public virtual StressState AverageStresses { get; }

		/// <summary>
		/// Get average concrete <see cref="PrincipalStressState"/>.
		/// </summary>
		public virtual PrincipalStressState ConcretePrincipalStresses { get; }

		/// <summary>
		/// Get average <see cref="PrincipalStressState"/>.
		/// </summary>
		public PrincipalStressState AveragePrincipalStresses => PrincipalStressState.FromStress(AverageStresses);

        /// <summary>
        /// Get the grip numbers of this.
        /// </summary>
        public int[] Grips => new[] { Grip1.Number, Grip2.Number, Grip3.Number, Grip4.Number };

        /// <summary>
        /// Get the DoF index of panel <see cref="Grips"/>.
        /// </summary>
        public override int[] DoFIndex => GlobalIndexes(Grips);

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
        /// <param name="geometry">The <see cref="PanelGeometry"/>.</param>
        /// <param name="concreteParameters">The concrete parameters <see cref="Parameters"/>.</param>
        /// <param name="concreteConstitutive">The concrete constitutive <see cref="Constitutive"/>.</param>
        /// <param name="reinforcement">The <see cref="BiaxialReinforcement"/>.</param>
        public Panel(ObjectId objectId, int number, Node grip1, Node grip2, Node grip3, Node grip4, PanelGeometry geometry, Parameters concreteParameters, Constitutive concreteConstitutive, BiaxialReinforcement reinforcement = null) : base(objectId, number)
        {
	        Grip1 = grip1;
	        Grip2 = grip2;
	        Grip3 = grip3;
	        Grip4 = grip4;

	        Geometry = geometry;

			Concrete = new BiaxialConcrete(concreteParameters, concreteConstitutive);

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
        /// <param name="width">Panel width, in <paramref name="geometryUnit"/>.</param>
        /// <param name="concreteParameters">The concrete parameters <see cref="Parameters"/>.</param>
        /// <param name="concreteConstitutive">The concrete constitutive <see cref="Constitutive"/>.</param>
        /// <param name="reinforcement">The <see cref="BiaxialReinforcement"/>.</param>
        /// <param name="geometryUnit">The <see cref="LengthUnit"/> of <paramref name="width"/> and <paramref name="vertices"/>' coordinates.</param>
        public Panel(ObjectId objectId, int number, Node grip1, Node grip2, Node grip3, Node grip4, Vertices vertices, double width, Parameters concreteParameters, Constitutive concreteConstitutive, BiaxialReinforcement reinforcement = null, LengthUnit geometryUnit = LengthUnit.Millimeter) 
	        : this(objectId, number, grip1, grip2, grip3, grip4, new PanelGeometry(vertices, width, geometryUnit), concreteParameters, concreteConstitutive, reinforcement)
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
        /// <param name="vertices">The array of <see cref="Point3d"/> panel vertices.</param>
        /// <param name="width">Panel width, in <paramref name="geometryUnit"/>.</param>
        /// <param name="concreteParameters">The concrete parameters <see cref="Parameters"/>.</param>
        /// <param name="concreteConstitutive">The concrete constitutive <see cref="Constitutive"/>.</param>
        /// <param name="reinforcement">The <see cref="BiaxialReinforcement"/>.</param>
        /// <param name="geometryUnit">The <see cref="LengthUnit"/> of <paramref name="width"/> and <paramref name="vertices"/>' coordinates.</param>
        public Panel(ObjectId objectId, int number, Node grip1, Node grip2, Node grip3, Node grip4, Point3d[] vertices, double width, Parameters concreteParameters, Constitutive concreteConstitutive, BiaxialReinforcement reinforcement = null, LengthUnit geometryUnit = LengthUnit.Millimeter) 
	        : this(objectId, number, grip1, grip2, grip3, grip4, new PanelGeometry(vertices, width, geometryUnit), concreteParameters, concreteConstitutive, reinforcement)
        {
        }

		public static Panel ReadPanel(AnalysisType analysisType, ObjectId panelObject, Units units,
			Parameters concreteParameters = null, Constitutive concreteConstitutive = null, Stringer[] stringers = null)
		{
			if (analysisType == AnalysisType.Linear)
				return new LinearPanel(panelObject, units, concreteParameters, concreteConstitutive);

			return new NonLinearPanel(panelObject, units, concreteParameters, concreteConstitutive, stringers);
		}


		/// <summary>
		/// Set panel displacements from global displacement vector.
		/// </summary>
		public void SetDisplacements(Vector<double> globalDisplacementVector)
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
		/// Do analysis of panel.
		/// </summary>
		/// <param name="globalDisplacements">The global displacement vector.</param>
		public virtual void Analysis(Vector<double> globalDisplacements = null)
		{
		}

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
				$"Panel {Number} + \n\n" +
				$"Grips: ({Grips[0]} - {Grips[1]} - {Grips[2]} - {Grips[3]})\n" +
				Geometry;

			if (!(Reinforcement is null))
				msgstr += "\n\n" + Reinforcement;

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