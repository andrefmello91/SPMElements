using System;
using System.Collections.Generic;
using System.Diagnostics.CodeAnalysis;
using System.Linq;
using andrefmello91.Extensions;
using andrefmello91.Material.Concrete;
using andrefmello91.Material.Reinforcement;
using andrefmello91.OnPlaneComponents;
using andrefmello91.SPMElements.StringerProperties;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;
using UnitsNet;
#nullable enable

namespace andrefmello91.SPMElements
{
	/// <summary>
	///     Stringer base class with linear properties.
	/// </summary>
	public class Stringer : SPMElement<StringerGeometry>
	{

		#region Properties

		/// <summary>
		///     Get the <see cref="UniaxialConcrete" /> of this stringer.
		/// </summary>
		public UniaxialConcrete Concrete { get; protected set; }

		/// <summary>
		///     Get crack openings in start, mid and end nodes.
		/// </summary>
		public virtual Length[] CrackOpenings { get; }

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
		public override Node[] Grips => new[] { Grip1, Grip2, Grip3 };

		/// <inheritdoc />
		public override Force MaxForce => Force.FromNewtons(LocalForces.AbsoluteMaximum());

		/// <summary>
		///     Get normal forces acting in the stringer.
		/// </summary>
		public (Force N1, Force N3) NormalForces => (Force.FromNewtons(-LocalForces[0]), Force.FromNewtons(LocalForces[2]));

		/// <summary>
		///     Get the <see cref="UniaxialReinforcement" /> of this element.
		/// </summary>
		public UniaxialReinforcement? Reinforcement { get; protected set; }

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

		#endregion

		#region Constructors

		/// <summary>
		///     Elastic stringer object.
		/// </summary>
		/// <inheritdoc cref="From" />
		protected Stringer(Node grip1, Node grip2, Node grip3, CrossSection crossSection)
			:base(new StringerGeometry(grip1.Position, grip3.Position, crossSection))
		{
			Grip1    = grip1;
			Grip2    = grip2;
			Grip3    = grip3;
		}


		/// <inheritdoc cref="Stringer(Node, Node, Node, CrossSection)" />
		/// <inheritdoc cref="From" />
		protected Stringer(Node grip1, Node grip2, Node grip3, CrossSection crossSection, IParameters concreteParameters, ConstitutiveModel model = ConstitutiveModel.MCFT, UniaxialReinforcement? reinforcement = null)
			: this(grip1, grip2, grip3, crossSection)
		{
			Reinforcement = reinforcement;
			Concrete      = new UniaxialConcrete(concreteParameters, GetConcreteArea(this), model);

			if (Reinforcement is not null)
				Reinforcement.ConcreteArea = Concrete.Area;

			InitiateStiffness();
		}

		#endregion

		#region Methods

		/// <summary>
		///     Create a <see cref="Stringer" /> based on element model.
		/// </summary>
		/// <param name="grip1">The initial <see cref="Node" /> of the <see cref="Stringer" />.</param>
		/// <param name="grip2">The center <see cref="Node" /> of the <see cref="Stringer" />.</param>
		/// <param name="grip3">The final <see cref="Node" /> of the <see cref="Stringer" />.</param>
		/// <param name="crossSection">The stringer cross-section.</param>
		/// <param name="concreteParameters">The concrete <see cref="IParameters" />.</param>
		/// <param name="model">The concrete <see cref="ConstitutiveModel" />.</param>
		/// <param name="reinforcement">The <see cref="UniaxialReinforcement" /> of this stringer.</param>
		/// <inheritdoc cref="As" />
		public static Stringer From(Node grip1, Node grip2, Node grip3, CrossSection crossSection, IParameters concreteParameters, ConstitutiveModel model = ConstitutiveModel.MCFT, UniaxialReinforcement? reinforcement = null, ElementModel elementModel = ElementModel.Elastic) =>
			elementModel switch
			{
				ElementModel.Elastic => new Stringer(grip1, grip2, grip3, crossSection, concreteParameters, model, reinforcement),
				_                    => new NLStringer(grip1, grip2, grip3, crossSection, concreteParameters, model, reinforcement)
			};

		/// <summary>
		///     Create a <see cref="Stringer" /> from a collection of <paramref name="nodes" /> and known positions of initial and
		///     final grips.
		/// </summary>
		/// <param name="nodes">The collection containing all <see cref="Node" />'s of SPM model.</param>
		/// <param name="grip1Position">The position of initial <see cref="Node" /> of the <see cref="Stringer" />.</param>
		/// <param name="grip3Position">The position of final <see cref="Node" /> of the <see cref="Stringer" />.</param>
		/// <inheritdoc cref="From" />
		public static Stringer FromNodes([NotNull] IEnumerable<Node> nodes, Point grip1Position, Point grip3Position, CrossSection crossSection, IParameters concreteParameters, ConstitutiveModel model = ConstitutiveModel.MCFT, UniaxialReinforcement? reinforcement = null, ElementModel elementModel = ElementModel.Elastic) =>
			From(nodes.GetByPosition(grip1Position), nodes.GetByPosition(grip1Position.MidPoint(grip3Position)), nodes.GetByPosition(grip3Position), crossSection, concreteParameters, model, reinforcement, elementModel);

		/// <summary>
		///     Get the proper concrete area for a <paramref name="stringer" />.
		/// </summary>
		internal static Area GetConcreteArea(Stringer stringer) =>
			stringer switch
			{
				NLStringer => stringer.Geometry.CrossSection.Area - (stringer.Reinforcement?.Area ?? Area.Zero),
				_          => stringer.Geometry.CrossSection.Area
			};

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
		///     Create a <see cref="Stringer" /> object based in this nonlinear stringer.
		/// </summary>
		/// <param name="elementModel">The <see cref="ElementModel" />.</param>
		/// <returns>
		///     <see cref="Stringer" />
		/// </returns>
		public Stringer As(ElementModel elementModel) =>
			elementModel switch
			{
				ElementModel.Elastic when this is NLStringer       => new Stringer(Grip1, Grip2, Grip3, Geometry.CrossSection, Concrete.Parameters, Concrete.Model, Reinforcement?.Clone()),
				ElementModel.Nonlinear when this is not NLStringer => new NLStringer(Grip1, Grip2, Grip3, Geometry.CrossSection, Concrete.Parameters, Concrete.Model, Reinforcement?.Clone()),
				_                                                  => this
			};

		/// <inheritdoc />
		public override void UpdateStiffness()
		{
			// Not needed in linear element.
		}

		/// <summary>
		///     Calculate initial stiffness elements.
		/// </summary>
		protected void InitiateStiffness()
		{
			// Calculate matrices
			TransformationMatrix = CalculateTransformationMatrix(Geometry.Angle);
			LocalStiffness       = CalculateStiffness(Concrete.Stiffness, Geometry.Length);
			Stiffness            = TransformationMatrix.Transpose() * LocalStiffness * TransformationMatrix;
		}

		/// <inheritdoc />
		public override string ToString()
		{
			var msgstr =
				$"Stringer {Number}\n\n" +
				$"Grips: ({GripNumbers[0]} - {GripNumbers[1]} - {GripNumbers[2]})\n" +
				$"DoFIndex: {DoFIndex.Select(i => i.ToString()).Aggregate((i, f) => $"{i} - {f}")}\n" +
				$"{Geometry}";

			if (Reinforcement is not null)
				msgstr += $"\n\n{Reinforcement}";

			return msgstr;
		}

		#endregion

	}
}