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
#nullable enable

namespace andrefmello91.SPMElements
{
	/// <summary>
	///     Stringer base class with linear properties.
	/// </summary>
	public class Stringer : SPMElement, IEquatable<Stringer>, IComparable<Stringer>
	{
		#region Properties

		/// <summary>
		///     Get the <see cref="UniaxialConcrete" /> of this stringer.
		/// </summary>
		public UniaxialConcrete Concrete { get; }

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


		/// <inheritdoc />
		public override IGrip[] Grips => new IGrip[] { Grip1, Grip2, Grip3 };


		/// <inheritdoc />
		public override Force MaxForce => Force.FromNewtons(LocalForces.AbsoluteMaximum());

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
		public StringerForceState State
		{
			get
			{
				var (N1, N3) = NormalForces;

				if (N1.ApproxZero(PlaneForce.Tolerance) && N3.ApproxZero(PlaneForce.Tolerance))
					return StringerForceState.Unloaded;

				if (N1 >= Force.Zero && N3 >= Force.Zero)
					return StringerForceState.PureTension;

				if (N1 <= Force.Zero && N3 <= Force.Zero)
					return StringerForceState.PureCompression;

				return StringerForceState.Combined;
			}
		}

		/// <inheritdoc />
		protected override Vector<double> LocalForces { get; set; }

		#endregion
		#region Constructors

		/// <summary>
		///     Elastic stringer object.
		/// </summary>
		/// <param name="grip1">The initial <see cref="Node" /> of the <see cref="Stringer" />.</param>
		/// <param name="grip2">The center <see cref="Node" /> of the <see cref="Stringer" />.</param>
		/// <param name="grip3">The final <see cref="Node" /> of the <see cref="Stringer" />.</param>
		/// <param name="crossSection">The stringer cross-section.</param>
		/// <param name="concreteParameters">The concrete <see cref="IParameters" />.</param>
		/// <param name="model">The concrete <see cref="ConstitutiveModel" />.</param>
		/// <param name="reinforcement">The <see cref="UniaxialReinforcement" /> of this stringer.</param>
		public Stringer(Node grip1, Node grip2, Node grip3, CrossSection crossSection, IParameters concreteParameters, ConstitutiveModel model = ConstitutiveModel.MCFT, UniaxialReinforcement? reinforcement = null)
		{
			Geometry      = new StringerGeometry(grip1.Position, grip3.Position, crossSection);
			Grip1         = grip1;
			Grip2         = grip2;
			Grip3         = grip3;
			Reinforcement = reinforcement;
			Concrete      = new UniaxialConcrete(concreteParameters, GetConcreteArea(this), model);

			if (!(Reinforcement is null))
				Reinforcement.ConcreteArea = Concrete.Area;

			// Initiate lazy members
			TransMatrix  = new Lazy<Matrix<double>>(() => CalculateTransformationMatrix(Geometry.Angle));
			LocStiffness = new Lazy<Matrix<double>>(() => CalculateStiffness(Concrete.Stiffness, Geometry.Length));
		}

		#endregion
		#region Methods

		/// <inheritdoc cref="Stringer(Node, Node, Node, CrossSection, IParameters, ConstitutiveModel, UniaxialReinforcement)" />
		/// <summary>
		///     Create a <see cref="Stringer" /> from a collection of <paramref name="nodes" /> and known positions of initial and
		///     final grips.
		/// </summary>
		/// <param name="nodes">The collection containing all <see cref="Node" />'s of SPM model.</param>
		/// <param name="grip1Position">The position of initial <see cref="Node" /> of the <see cref="Stringer" />.</param>
		/// <param name="grip3Position">The position of final <see cref="Node" /> of the <see cref="Stringer" />.</param>
		public static Stringer FromNodes([NotNull] IEnumerable<Node> nodes, Point grip1Position, Point grip3Position, CrossSection crossSection, IParameters concreteParameters, ConstitutiveModel model = ConstitutiveModel.MCFT, UniaxialReinforcement? reinforcement = null, ElementModel elementModel = ElementModel.Elastic)
		{
			var stringer = new Stringer(nodes.GetByPosition(grip1Position), nodes.GetByPosition(grip1Position.MidPoint(grip3Position)), nodes.GetByPosition(grip3Position), crossSection, concreteParameters, model, reinforcement);

			return elementModel is ElementModel.Elastic
				? stringer
				: stringer.ToNonlinear();
		}

		/// <summary>
		///     Calculate local stiffness <see cref="Matrix" />.
		/// </summary>
		/// <param name="concreteStiffness">The stiffness of concrete cross-section.</param>
		/// <param name="stringerLength">The length of the stringer.</param>
		private static Matrix<double> CalculateStiffness(Force concreteStiffness, Length stringerLength)
		{
			// Calculate the constant factor of stiffness
			var k = concreteStiffness.Newtons / (3 * stringerLength.Millimeters);

			// Calculate the local stiffness matrix
			return
				k * new double[,]
				{
					{ 7, -8, 1 },
					{ -8, 16, -8 },
					{ 1, -8, 7 }
				}.ToMatrix();
		}

		/// <summary>
		///     Calculate the transformation matrix based on the <paramref name="angle" /> of a stringer.
		/// </summary>
		/// <param name="angle">The angle of the stringer, related to horizontal axis.</param>
		private static Matrix<double> CalculateTransformationMatrix(double angle)
		{
			// Get the direction cosines
			var (l, m) = angle.DirectionCosines();

			// Obtain the transformation matrix
			return new[,]
			{
				{ l, m, 0, 0, 0, 0 },
				{ 0, 0, l, m, 0, 0 },
				{ 0, 0, 0, 0, l, m }
			}.ToMatrix();
		}

		/// <summary>
		///     Get the proper concrete area for a <paramref name="stringer" />.
		/// </summary>
		private static Area GetConcreteArea(Stringer stringer) =>
			stringer switch
			{
				NLStringer => stringer.Geometry.CrossSection.Area - (stringer.Reinforcement?.Area ?? Area.Zero),
				_          => stringer.Geometry.CrossSection.Area
			};

		/// <inheritdoc />
		public override int CompareTo(IFiniteElement? other) => other is Stringer stringer
			? CompareTo(stringer)
			: 0;

		/// <summary>
		///     Returns true if <paramref name="obj" /> is <see cref="Stringer" /> and <see cref="Geometry" /> of
		///     <paramref name="obj" /> is equal to this.
		/// </summary>
		public override bool Equals(object? obj) => obj is Stringer other && Equals(other);

		/// <inheritdoc />
		public override bool Equals(IFiniteElement? other) => other is Stringer stringer && Equals(stringer);

		/// <summary>
		///     Create a <see cref="NLStringer" /> object based in this stringer.
		/// </summary>
		/// <returns>
		///     <see cref="NLStringer" />
		/// </returns>
		public NLStringer ToNonlinear() => new(Grip1, Grip2, Grip3, Geometry.CrossSection, Concrete.Parameters, Concrete.Model, Reinforcement);

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
	}
}