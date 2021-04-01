using System;
using System.Collections.Generic;
using System.Diagnostics.CodeAnalysis;
using System.Linq;
using andrefmello91.Extensions;
using andrefmello91.FEMAnalysis;
using andrefmello91.Material.Concrete;
using andrefmello91.Material.Reinforcement;
using andrefmello91.OnPlaneComponents;
using andrefmello91.SPMElements.PanelProperties;
using MathNet.Numerics;
using MathNet.Numerics.LinearAlgebra;
using UnitsNet;
#nullable enable

namespace andrefmello91.SPMElements
{
	/// <summary>
	///     Base panel class with linear properties.
	/// </summary>
	public class Panel : SPMElement, IEquatable<Panel>, IComparable<Panel>
	{

		#region Properties

		/// <summary>
		///     Get average <see cref="PrincipalStressState" />.
		/// </summary>
		public PrincipalStressState AveragePrincipalStresses => PrincipalStressState.FromStress(AverageStresses);

		/// <summary>
		///     Get average <see cref="StressState" />.
		/// </summary>
		public virtual StressState AverageStresses
		{
			get
			{
				// Get the dimensions as a vector
				var lsV = Geometry.EdgeLengths.Select(l => l.Millimeters).ToVector();

				// Calculate the shear stresses
				var tau = LocalForces / (lsV * Geometry.Width.Millimeters);

				// Calculate the average stress
				var tauAvg = (-tau[0] + tau[1] - tau[2] + tau[3]) / 4;

				return new StressState(0, 0, tauAvg);
			}
		}

		/// <summary>
		///     Get <see cref="BiaxialConcrete" /> of this.
		/// </summary>
		public BiaxialConcrete Concrete { get; }

		/// <summary>
		///     Get average concrete <see cref="PrincipalStrainState" />.
		/// </summary>
		public virtual PrincipalStrainState ConcretePrincipalStrains { get; }

		/// <summary>
		///     Get average concrete <see cref="PrincipalStressState" />.
		/// </summary>
		public virtual PrincipalStressState ConcretePrincipalStresses
		{
			get
			{
				Pressure sig2;

				// Get shear stress
				var tau = AverageStresses.TauXY;

				// Get steel strengths
				Pressure
					fyx = Reinforcement?.DirectionX?.Steel?.YieldStress ?? Pressure.Zero,
					fyy = Reinforcement?.DirectionY?.Steel?.YieldStress ?? Pressure.Zero;

				if (fyx.Approx(fyy, StressState.Tolerance))
				{
					sig2 = -2 * tau.Abs();
				}

				else
				{
					// Get relation of steel strengths
					var rLambda = Math.Sqrt(fyx / fyy);
					sig2 = -tau.Abs() * (rLambda + 1 / rLambda);
				}

				var theta1 = tau >= Pressure.Zero ? Constants.PiOver4 : -Constants.PiOver4;

				return new PrincipalStressState(Pressure.Zero, sig2, theta1);
			}
		}

		/// <summary>
		///     Get the average crack opening in concrete.
		/// </summary>
		public virtual Length CrackOpening { get; }

		/// <summary>
		///     Get <see cref="PanelGeometry" /> of this element.
		/// </summary>
		public PanelGeometry Geometry { get; }

		/// <summary>
		///     Get the center <see cref="SPMElements.Node" /> of bottom edge.
		/// </summary>
		public Node Grip1 { get; }

		/// <summary>
		///     Get the center <see cref="SPMElements.Node" /> of right edge.
		/// </summary>
		public Node Grip2 { get; }

		/// <summary>
		///     Get the center <see cref="SPMElements.Node" /> of top edge.
		/// </summary>
		public Node Grip3 { get; }

		/// <summary>
		///     Get the center <see cref="SPMElements.Node" /> of left edge.
		/// </summary>
		public Node Grip4 { get; }

		/// <inheritdoc />
		public override Node[] Grips => new [] { Grip1, Grip2, Grip3, Grip4 };

		/// <inheritdoc />
		public override Force MaxForce => Force.FromNewtons(Forces.AbsoluteMaximum());

		/// <summary>
		///     Get <see cref="WebReinforcement" /> of this.
		/// </summary>
		public WebReinforcement? Reinforcement { get; }

		/// <inheritdoc />
		protected override Vector<double> LocalForces { get; set; }

		#endregion

		#region Constructors

		/// <inheritdoc cref="Panel(Node, Node, Node, Node, PanelGeometry, IParameters, ConstitutiveModel, WebReinforcement)" />
		/// <param name="vertices">Panel <see cref="Vertices" /> object.</param>
		/// <param name="width">Panel width.</param>
		public Panel(Node grip1, Node grip2, Node grip3, Node grip4, Vertices vertices, Length width, IParameters concreteParameters, ConstitutiveModel model = ConstitutiveModel.MCFT, WebReinforcement? reinforcement = null)
			: this(grip1, grip2, grip3, grip4, new PanelGeometry(vertices, width), concreteParameters, model, reinforcement)
		{
		}

		/// <summary>
		///     Elastic panel object.
		/// </summary>
		/// <param name="grip1">The center <see cref="Node" /> of bottom edge</param>
		/// <param name="grip2">The center <see cref="Node" /> of right edge</param>
		/// <param name="grip3">The center <see cref="Node" /> of top edge</param>
		/// <param name="grip4">The center <see cref="Node" /> of left edge</param>
		/// <param name="geometry">The <seealso cref="PanelGeometry" />.</param>
		/// <param name="concreteParameters">The concrete parameters <see cref="Parameters" />.</param>
		/// <param name="model">The concrete <see cref="ConstitutiveModel" />.</param>
		/// <param name="reinforcement">The <see cref="WebReinforcement" />.</param>
		public Panel(Node grip1, Node grip2, Node grip3, Node grip4, PanelGeometry geometry, IParameters concreteParameters, ConstitutiveModel model = ConstitutiveModel.MCFT, WebReinforcement? reinforcement = null)
		{
			Grip1 = grip1;
			Grip2 = grip2;
			Grip3 = grip3;
			Grip4 = grip4;

			Geometry = geometry;

			Concrete = new BiaxialConcrete(concreteParameters, model);

			Reinforcement = reinforcement;

			if (!(Reinforcement is null))
				Reinforcement.Width = Geometry.Width;

			// Initiate lazy members
			TransMatrix  = new Lazy<Matrix<double>>(() => CalculateTransformationMatrix(Geometry));
			LocStiffness = new Lazy<Matrix<double>>(() => CalculateStiffness(Geometry, Concrete.Parameters.TransverseModule));
		}

		#endregion

		#region Methods

		/// <summary>
		///     Create a <see cref="Panel" /> from a collection of <paramref name="nodes" /> and tha panel geometry.
		/// </summary>
		/// <param name="nodes">The collection containing all <see cref="Node" />'s in SPM model.</param>
		public static Panel FromNodes(IEnumerable<Node> nodes, PanelGeometry geometry, IParameters concreteParameters, ConstitutiveModel model = ConstitutiveModel.MCFT, WebReinforcement? reinforcement = null, ElementModel elementModel = ElementModel.Elastic)
		{
			var nds = geometry.Edges.Select(e => nodes.GetByPosition(e.CenterPoint)).ToArray();

			var panel = new Panel(nds[0], nds[1], nds[2], nds[3], geometry, concreteParameters, model, reinforcement);

			return elementModel is ElementModel.Elastic
				? panel
				: panel.ToNonlinear();
		}

		/// <summary>
		///     Calculate panel stiffness
		/// </summary>
		/// <param name="geometry">The <see cref="PanelGeometry" />.</param>
		/// <param name="concreteTransverseModule">The transverse elastic module of concrete.</param>
		private static Matrix<double> CalculateStiffness(PanelGeometry geometry, Pressure concreteTransverseModule) => geometry.IsRectangular
			? RectangularPanelStiffness(geometry, concreteTransverseModule)
			: NonRectangularPanelStiffness(geometry, concreteTransverseModule);

		/// <summary>
		///     Calculate transformation matrix
		/// </summary>
		private static Matrix<double> CalculateTransformationMatrix(PanelGeometry geometry)
		{
			// Get the transformation matrix
			// Direction cosines
			var (m1, n1) = geometry.Edge1.Angle.DirectionCosines();
			var (m2, n2) = geometry.Edge2.Angle.DirectionCosines();
			var (m3, n3) = geometry.Edge3.Angle.DirectionCosines();
			var (m4, n4) = geometry.Edge4.Angle.DirectionCosines();

			// T matrix
			return new[,]
			{
				{ m1, n1, 0, 0, 0, 0, 0, 0 },
				{ 0, 0, m2, n2, 0, 0, 0, 0 },
				{ 0, 0, 0, 0, m3, n3, 0, 0 },
				{ 0, 0, 0, 0, 0, 0, m4, n4 }
			}.ToMatrix();
		}

		/// <summary>
		///     Calculate panel stiffness if panel is not rectangular.
		///     <para>See: <seealso cref="PanelGeometry.IsRectangular" /></para>
		/// </summary>
		/// <inheritdoc cref="CalculateStiffness" />
		private static Matrix<double> NonRectangularPanelStiffness(PanelGeometry geometry, Pressure concreteTransverseModule)
		{
			// Get the dimensions
			double[]
				x = geometry.Vertices.XCoordinates.Select(cx => cx.Millimeters).ToArray(),
				y = geometry.Vertices.YCoordinates.Select(cy => cy.Millimeters).ToArray();

			var (a, b, c, d) = geometry.DimensionsInMillimeters();

			double
				w  = geometry.Width.Millimeters,
				l1 = geometry.Edge1.Length.Millimeters,
				l2 = geometry.Edge2.Length.Millimeters,
				l3 = geometry.Edge3.Length.Millimeters,
				l4 = geometry.Edge4.Length.Millimeters;

			// Calculate Gc
			var Gc = concreteTransverseModule.Megapascals;

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
				{ c2, c3, c4 },
				{ s2, s3, s4 },
				{ r2, r3, r4 }
			}.ToMatrix();

			var km2 = new[,]
			{
				{ c1, c3, c4 },
				{ s1, s3, s4 },
				{ r1, r3, r4 }
			}.ToMatrix();

			var km3 = new[,]
			{
				{ c1, c2, c4 },
				{ s1, s2, s4 },
				{ r1, r2, r4 }
			}.ToMatrix();

			var km4 = new[,]
			{
				{ c1, c2, c3 },
				{ s1, s2, s3 },
				{ r1, r2, r3 }
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
			var D = 16 * Gc * w / (kf * ku);

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
		///     Calculate panel stiffness if panel is rectangular.
		///     <para>See: <seealso cref="PanelGeometry.IsRectangular" /></para>
		/// </summary>
		/// <inheritdoc cref="CalculateStiffness" />
		private static Matrix<double> RectangularPanelStiffness(PanelGeometry geometry, Pressure concreteTransverseModule)
		{
			// Get the dimensions
			double
				w = geometry.Width.Millimeters,
				a = geometry.Dimensions.a.Millimeters,
				b = geometry.Dimensions.b.Millimeters;

			// Calculate the parameters of the stiffness matrix
			double
				a_b = a / b,
				b_a = b / a;

			// Calculate Gc
			var Gc = concreteTransverseModule.Megapascals;

			// Calculate the stiffness matrix
			return
				Gc * w * new[,]
				{
					{ a_b, -1, a_b, -1 },
					{ -1, b_a, -1, b_a },
					{ a_b, -1, a_b, -1 },
					{ -1, b_a, -1, b_a }
				}.ToMatrix();
		}

		/// <inheritdoc />
		public override int CompareTo(IFiniteElement? other) => other is Panel panel
			? CompareTo(panel)
			: 0;

		/// <inheritdoc />
		public override bool Equals(IFiniteElement? other) => other is Panel panel && Equals(panel);

		/// <inheritdoc />
		public override bool Equals(SPMElement? other) => other is Panel panel && Equals(panel);

		/// <summary>
		///     Returns true if <paramref name="obj" /> is <see cref="Panel" /> and <see cref="Geometry" /> is equal.
		/// </summary>
		/// <param name="obj">The other <see cref="object" /> to compare.</param>
		public override bool Equals(object? obj) => obj is Panel other && Equals(other);

		/// <summary>
		///     Set stringer dimensions on edges.
		///     <para>See: <see cref="Edge.SetStringerDimension" /></para>
		/// </summary>
		/// <param name="stringers">The collection containing all of the stringers.</param>
		public void SetStringersDimensions([NotNull] IEnumerable<Stringer> stringers)
		{
			// Get dimensions
			var hs = Grips
				.Select(g => 0.5 * stringers.First(str => g.Equals(str.Grip2)).Geometry.CrossSection.Height)
				.ToArray();

			// Set on edges
			Geometry.Edge1.SetStringerDimension(hs[0]);
			Geometry.Edge2.SetStringerDimension(hs[1]);
			Geometry.Edge3.SetStringerDimension(hs[2]);
			Geometry.Edge4.SetStringerDimension(hs[3]);
		}

		/// <summary>
		///     Create a <see cref="NLPanel" /> object based in this nonlinear stringer.
		/// </summary>
		/// <returns>
		///     <see cref="NLPanel" />
		/// </returns>
		public Panel ToNonlinear() => new NLPanel(Grip1, Grip2, Grip3, Grip4, Geometry, Concrete.Parameters, Concrete.Model, Reinforcement?.Clone());

		/// <inheritdoc />
		public int CompareTo(Panel? other) => other is null
			? 0
			: Geometry.CompareTo(other.Geometry);

		/// <summary>
		///     Returns true if <paramref name="other" />'s <see cref="Geometry" /> is equal.
		/// </summary>
		/// <param name="other">The other <see cref="Panel" /> object to compare.</param>
		public bool Equals(Panel? other) => !(other is null) && Geometry == other.Geometry;

		/// <inheritdoc />
		public override int GetHashCode() => Geometry.GetHashCode();

		/// <inheritdoc />
		public override string ToString()
		{
			var msgstr =
				$"Panel {Number}\n\n" +
				$"Grips: ({Grips[0]} - {Grips[1]} - {Grips[2]} - {Grips[3]})\n" +
				$"DoFIndex: {DoFIndex.Select(i => i.ToString()).Aggregate((i, f) => $"{i} - {f}")}\n" +
				$"{Geometry}";

			if (!(Reinforcement is null))
				msgstr += $"\n\n{Reinforcement}";

			return msgstr;
		}

		#endregion

	}
}