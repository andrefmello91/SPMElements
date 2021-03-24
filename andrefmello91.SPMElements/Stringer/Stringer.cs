using System;
using System.Collections.Generic;
using System.Diagnostics.CodeAnalysis;
using System.Linq;
using andrefmello91.FEMAnalysis;
using andrefmello91.Material.Concrete;
using andrefmello91.Material.Reinforcement.Uniaxial;
using andrefmello91.OnPlaneComponents;
using andrefmello91.OnPlaneComponents.Force;
using andrefmello91.SPMElements.StringerProperties;
using Extensions;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;
using UnitsNet;
using UnitsNet.Units;
using static andrefmello91.FEMAnalysis.Extensions;

#nullable enable

namespace andrefmello91.SPMElements
{
	/// <summary>
	///     Stringer base class with linear properties.
	/// </summary>
	public class Stringer : IFiniteElement, IEquatable<Stringer>, IComparable<Stringer>
	{
		/// <summary>
		///     Type of forces that stringer can be loaded.
		/// </summary>
		public enum ForceState
		{
			/// <summary>
			///		Stringer is not loaded.
			/// </summary>
			Unloaded,
			
			/// <summary>
			///		Stringer is fully tensioned.
			/// </summary>
			PureTension,
			
			/// <summary>
			///		Stringer is fully compressed.
			/// </summary>
			PureCompression,
			
			/// <summary>
			///		Stringer is tensioned at an end and compressed at the other end.
			/// </summary>
			Combined
		}

		#region Fields

		// Auxiliary fields
		private Lazy<Matrix<double>> _transMatrix, _localStiffness;

		#endregion

		#region Properties

		/// <inheritdoc />
		public int Number { get; set; }

		/// <inheritdoc />
		public int[] DoFIndex => GlobalIndexes(Grips).ToArray();

		/// <summary>
		///		Get the grip numbers of this element.
		/// </summary>
		public int[] GripNumbers => new[] { Grip1.Number, Grip2.Number, Grip3.Number };

		/// <inheritdoc />
		public IGrip[] Grips => new IGrip[] { Grip1, Grip2, Grip3 };

		/// <summary>
		///     Get the displacement <see cref="Vector" />, in local coordinate system.
		/// </summary>
		/// <remarks>
		///     Components in <see cref="LengthUnit.Millimeter" />.
		/// </remarks>
		public Vector<double> LocalDisplacements => TransformationMatrix * Displacements;

		/// <inheritdoc />
		public Vector<double> Displacements => this.GetDisplacementsFromGrips();

		/// <summary>
		///     Get the force <see cref="Vector" />, in local coordinate system.
		/// </summary>
		/// <remarks>
		///     Components in <see cref="ForceUnit.Newton" />.
		/// </remarks>
		public virtual Vector<double> LocalForces { get; protected set; }

		/// <inheritdoc />
		public Vector<double> Forces => TransformationMatrix.Transpose() * LocalForces;

		/// <summary>
		///		Get the maximum local force at this stringer.
		/// </summary>
		public Force MaxForce => Force.FromNewtons(LocalForces.AbsoluteMaximum());

		/// <summary>
		///		Get the transformation matrix to transform from local to global coordinate systems.
		/// </summary>
		public Matrix<double> TransformationMatrix => _transMatrix.Value;

		/// <summary>
		///		Get the local stiffness <see cref="Matrix"/>.
		/// </summary>
		public virtual Matrix<double> LocalStiffness => _localStiffness.Value;

		/// <inheritdoc />
		public Matrix<double> Stiffness => TransformationMatrix.Transpose() * LocalStiffness * TransformationMatrix;

		/// <summary>
		///     Get/set the <see cref="UniaxialConcrete" /> of this.
		/// </summary>
		public UniaxialConcrete Concrete { get; protected set; }

		/// <summary>
		///     Get concrete area.
		/// </summary>
		protected Area ConcreteArea => this is NLStringer
			? Geometry.CrossSection.Area - (Reinforcement?.Area ?? Area.Zero)
			: Geometry.CrossSection.Area;

		/// <summary>
		///     Get crack openings in start, mid and end nodes.
		/// </summary>
		public virtual Length[] CrackOpenings { get; }

		/// <summary>
		///     Get/set the <see cref="Geometry" /> of this.
		/// </summary>
		public StringerGeometry Geometry { get; }

		/// <summary>
		///     Get the initial <see cref="Node" /> of this stringer.
		/// </summary>
		public Node Grip1 { get; }

		/// <summary>
		///     Get the center <see cref="Node" /> of this stringer.
		/// </summary>
		public Node Grip2 { get; }

		/// <summary>
		///     Get the end <see cref="Node" /> of this stringer.
		/// </summary>
		public Node Grip3 { get; }

		/// <summary>
		///     Get normal forces acting in the stringer.
		/// </summary>
		public (Force N1, Force N3) NormalForces => (Force.FromNewtons(LocalForces[0]), Force.FromNewtons(-LocalForces[2]));

		/// <summary>
		///     Get the <see cref="UniaxialReinforcement" /> of this element.
		/// </summary>
		public UniaxialReinforcement? Reinforcement { get; }

		/// <summary>
		///     Get the state of forces acting at this stringer.
		/// </summary>
		public ForceState State
		{
			get
			{
				var (N1, N3) = NormalForces;

				if (N1.ApproxZero(PlaneForce.Tolerance) && N3.ApproxZero(PlaneForce.Tolerance))
					return ForceState.Unloaded;

				if (N1 >= Force.Zero && N3 >= Force.Zero)
					return ForceState.PureTension;

				if (N1 <= Force.Zero && N3 <= Force.Zero)
					return ForceState.PureCompression;

				return ForceState.Combined;
			}
		}

		#endregion

		#region Constructors

		/// <summary>
		///     Stringer object.
		/// </summary>
		/// <param name="grip1">The initial <see cref="SPMElements" /> of the <see cref="Stringer" />.</param>
		/// <param name="grip2">The center <see cref="SPMElements" /> of the <see cref="Stringer" />.</param>
		/// <param name="grip3">The final <see cref="SPMElements" /> of the <see cref="Stringer" />.</param>
		/// <param name="crossSection">The stringer cross-section.</param>
		/// <param name="concreteParameters">The concrete <see cref="IParameters" />.</param>
		/// <param name="model">The concrete <see cref="ConstitutiveModel" />.</param>
		/// <param name="reinforcement">The <see cref="UniaxialReinforcement" /> of this stringer.</param>
		public Stringer(Node grip1, Node grip2, Node grip3, CrossSection crossSection, IParameters concreteParameters, ConstitutiveModel model = ConstitutiveModel.MCFT, UniaxialReinforcement? reinforcement = null)
		{
			Geometry       = new StringerGeometry(grip1.Position, grip3.Position, crossSection);
			Grip1          = grip1;
			Grip2          = grip2;
			Grip3          = grip3;
			Reinforcement  = reinforcement;
			Concrete       = new UniaxialConcrete(concreteParameters, ConcreteArea, model);

			if (!(Reinforcement is null))
				Reinforcement.ConcreteArea = Geometry.CrossSection.Area;
			
			// Initiate lazy members
			_transMatrix    = new Lazy<Matrix<double>>(CalculateTransformationMatrix);
			_localStiffness = new Lazy<Matrix<double>>(CalculateStiffness);
		}

		#endregion

		#region  Methods
		
		/// <inheritdoc cref="Stringer(Node, Node, Node, CrossSection, IParameters, ConstitutiveModel, UniaxialReinforcement)"/>
		/// <summary>
		///		Create a <see cref="Stringer"/> from a collection of <paramref name="nodes"/> and known positions of initial and final grips.
		/// </summary>
		/// <param name="nodes">The collection containing all <see cref="SPMElements" />'s of SPM model.</param>
		/// <param name="grip1Position">The position of initial <see cref="SPMElements" /> of the <see cref="Stringer" />.</param>
		/// <param name="grip3Position">The position of final <see cref="SPMElements" /> of the <see cref="Stringer" />.</param>
		public static Stringer FromNodes([NotNull] IEnumerable<Node> nodes, Point grip1Position, Point grip3Position, CrossSection crossSection, IParameters concreteParameters, ConstitutiveModel model = ConstitutiveModel.MCFT, UniaxialReinforcement? reinforcement = null) =>
			new Stringer(nodes.GetByPosition(grip1Position), nodes.GetByPosition(grip1Position.MidPoint(grip3Position)), nodes.GetByPosition(grip3Position), crossSection, concreteParameters, model, reinforcement);

		/// <summary>
		///     Read the stringer based on <see cref="AnalysisType" />.
		/// </summary>
		/// <param name="analysisType">Type of analysis to perform (<see cref="AnalysisType" />).</param>
		/// <param name="number">The stringer number.</param>
		/// <inheritdoc cref="Stringer" />
		public static Stringer Read(AnalysisType analysisType, int number, IEnumerable<SPMElements.Node> nodes, StringerGeometry geometry, IParameters concreteParameters, ConstitutiveModel model, UniaxialReinforcement reinforcement = null) =>
			analysisType is AnalysisType.Linear
				? new   Stringer(nodes, geometry, concreteParameters, model, reinforcement) { Number = number }
				: new NLStringer(nodes, geometry, concreteParameters, model, reinforcement) { Number = number };

		/// <inheritdoc />
		public virtual void CalculateForces() => LocalForces = CalculateLocalForces();

		/// <summary>
		///     Calculate the transformation matrix.
		/// </summary>
		private Matrix<double> CalculateTransformationMatrix()
		{
			// Get the direction cosines
			var (l, m) = Geometry.Angle.DirectionCosines();

			// Obtain the transformation matrix
			return new [,]
			{
				{l, m, 0, 0, 0, 0 },
				{0, 0, l, m, 0, 0 },
				{0, 0, 0, 0, l, m }
			}.ToMatrix();
		}


		/// <summary>
		///     Calculate local stiffness <see cref="Matrix" />.
		/// </summary>
		/// <returns></returns>
		protected Matrix<double> CalculateStiffness()
		{
			// Calculate the constant factor of stiffness
			var k = Concrete.Stiffness.Newtons / (3 * Geometry.Length.Millimeters);

			// Calculate the local stiffness matrix
			return 
				k * new double[,]
				{
					{  7, -8,  1 },
					{ -8, 16, -8 },
					{  1, -8,  7 }
				}.ToMatrix();
		}

		/// <summary>
		///     Calculate local stringer forces.
		/// </summary>
		private Vector<double> CalculateLocalForces()
		{
			// Calculate the vector of normal forces
			var fl = LocalStiffness * LocalDisplacements;

			// Approximate small values to zero
			fl.CoerceZero(0.001);

			return fl;
		}

		/// <inheritdoc />
		public int CompareTo(Stringer? other) => other is null
			? 1
			: Geometry.CompareTo(other.Geometry);

		/// <summary>
		///     Returns true if <see cref="Geometry" /> of <paramref name="other" /> is equal to this.
		/// </summary>
		/// <param name="other"></param>
		public bool Equals(Stringer? other) => !(other is null) && Geometry == other.Geometry;

		/// <inheritdoc />
		public bool Equals(IFiniteElement? other) => other is Stringer stringer && Equals(stringer);

		/// <inheritdoc />
		public int CompareTo(IFiniteElement? other) => other is Stringer stringer
			? CompareTo(stringer)
			: 0;

		/// <summary>
		///     Returns true if <paramref name="obj" /> is <see cref="Stringer" /> and <see cref="Geometry" /> of
		///     <paramref name="obj" /> is equal to this.
		/// </summary>
		public override bool Equals(object? obj) => obj is Stringer other && Equals(other);

		/// <inheritdoc />
		public override int GetHashCode() => Geometry.GetHashCode();

		/// <inheritdoc />
		public override string ToString()
		{
			var msgstr =
				$"Stringer {Number}\n\n" +
				$"Grips: ({GripNumbers[0]} - {GripNumbers[1]} - {GripNumbers[2]})\n" +
				$"DoFIndex: {DoFIndex.Select(i => i.ToString()).Aggregate((i, f) => $"{i} - {f}")}\n" +
				$"{Geometry}";

			if (!(Reinforcement is null)) 
				msgstr += $"\n\n{Reinforcement}";

			return msgstr;
		}

		#endregion

		#region Operators

		/// <summary>
		///     Returns true if arguments are equal.
		///     <para>See:<seealso cref="Equals(Stringer)" />.</para>
		/// </summary>
		public static bool operator == (Stringer? left, Stringer? right) => !(left is null) && left.Equals(right);

		/// <summary>
		///     Returns true if arguments are different.
		///     <para>See:<seealso cref="Equals(Stringer)" />.</para>
		/// </summary>
		public static bool operator != (Stringer? left, Stringer? right) => !(left is null) && !left.Equals(right);

		#endregion
	}
}