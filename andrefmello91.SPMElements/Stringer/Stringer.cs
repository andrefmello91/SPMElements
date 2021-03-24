﻿using System;
using System.Collections.Generic;
using System.Linq;
using andrefmello91.SPMElements.StringerProperties;
using Extensions;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;
using UnitsNet;
using UnitsNet.Units;
using static andrefmello91.SPMElements.Extensions;
using Force = UnitsNet.Force;

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
			Unloaded,
			PureTension,
			PureCompression,
			Combined
		}

		#region Fields

		// Auxiliary fields
		protected Matrix<double>? TransMatrix, LocStiffness;

		#endregion

		#region Properties

		/// <inheritdoc />
		public int Number { get; set; }

		/// <inheritdoc />
		public int[] DoFIndex => GlobalIndexes(Grips).ToArray();

		public int[] Grips => new[] { Grip1.Number, Grip2.Number, Grip3.Number };

		public Vector<double> LocalDisplacements => TransformationMatrix * Displacements;

		public Vector<double> Displacements { get; protected set; }

		public virtual Vector<double> LocalForces { get; set; }

		public Vector<double> Forces => TransformationMatrix.Transpose() * LocalForces;

		public Force MaxForce => Force.FromNewtons(LocalForces.AbsoluteMaximum());

		public Matrix<double> TransformationMatrix => TransMatrix ?? CalculateTransformationMatrix();

		public virtual Matrix<double> LocalStiffness => LocStiffness ?? CalculateStiffness();

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
		///     Get the initial <see cref="SPMElements.Node" /> of this.
		/// </summary>
		public SPMElements.Node Grip1 { get; }

		/// <summary>
		///     Get the center <see cref="SPMElements.Node" /> of this.
		/// </summary>
		public SPMElements.Node Grip2 { get; }

		/// <summary>
		///     Get the end <see cref="SPMElements.Node" /> of this.
		/// </summary>
		public SPMElements.Node Grip3 { get; }

		/// <summary>
		///     Get normal forces acting in the stringer.
		/// </summary>
		public (Force N1, Force N3) NormalForces => (Force.FromNewtons(LocalForces[0]), Force.FromNewtons(-LocalForces[2]));

		/// <summary>
		///     Get the <see cref="UniaxialReinforcement" /> of this element.
		/// </summary>
		public UniaxialReinforcement? Reinforcement { get; }

		/// <summary>
		///     Get the state of forces acting on the stringer.
		/// </summary>
		public ForceState State
		{
			get
			{
				var (N1, N3) = NormalForces;

				if (N1.ApproxZero(Tolerance) && N3.ApproxZero(Tolerance))
					return ForceState.Unloaded;

				if (N1 > Force.Zero && N3 > Force.Zero)
					return ForceState.PureTension;

				if (N1 < Force.Zero && N3 < Force.Zero)
					return ForceState.PureCompression;

				return ForceState.Combined;
			}
		}

		#endregion

		#region Constructors

		/// <inheritdoc cref="Stringer" />
		/// <param name="unit">
		///     The <see cref="LengthUnit" /> of <paramref name="width" /> and <paramref name="height" />.
		///     <para>Default: <seealso cref="LengthUnit.Millimeter" />.</para>
		/// </param>
		public Stringer(SPMElements.Node grip1, SPMElements.Node grip2, SPMElements.Node grip3, double width, double height, IParameters concreteParameters, ConstitutiveModel model = ConstitutiveModel.MCFT, UniaxialReinforcement? reinforcement = null, LengthUnit unit = LengthUnit.Millimeter)
			: this (grip1, grip2, grip3, Length.From(width, unit), Length.From(height, unit), concreteParameters, model, reinforcement)
		{
		}

		/// <summary>
		///     Stringer object.
		/// </summary>
		/// <param name="grip1">The initial <see cref="SPMElements" /> of the <see cref="Stringer" />.</param>
		/// <param name="grip2">The center <see cref="SPMElements" /> of the <see cref="Stringer" />.</param>
		/// <param name="grip3">The final <see cref="SPMElements" /> of the <see cref="Stringer" />.</param>
		/// <param name="width">The stringer width.</param>
		/// <param name="height">The stringer height.</param>
		/// <param name="concreteParameters">The concrete <see cref="IParameters" />.</param>
		/// <param name="model">The concrete <see cref="ConstitutiveModel" />.</param>
		/// <param name="reinforcement">The <see cref="UniaxialReinforcement" /> of this stringer.</param>
		public Stringer(SPMElements.Node grip1, SPMElements.Node grip2, SPMElements.Node grip3, Length width, Length height, IParameters concreteParameters, ConstitutiveModel model = ConstitutiveModel.MCFT, UniaxialReinforcement? reinforcement = null)
		{
			Geometry       = new StringerGeometry(grip1.Position, grip3.Position, width, height);
			Grip1          = grip1;
			Grip2          = grip2;
			Grip3          = grip3;
			Reinforcement  = reinforcement;
			Concrete       = new UniaxialConcrete(concreteParameters, ConcreteArea, model);

			if (!(Reinforcement is null))
				Reinforcement.ConcreteArea = Geometry.CrossSection.Area;
		}

		/// <param name="nodes">The collection containing all <see cref="SPMElements" />'s of SPM model.</param>
		/// <param name="grip1Position">The position of initial <see cref="SPMElements" /> of the <see cref="Stringer" />.</param>
		/// <param name="grip3Position">The position of final <see cref="SPMElements" /> of the <see cref="Stringer" />.</param>
		/// <inheritdoc
		///     cref="Stringer" />
		public Stringer(IEnumerable<SPMElements.Node> nodes, Point grip1Position, Point grip3Position, double width, double height, IParameters concreteParameters, ConstitutiveModel model = ConstitutiveModel.MCFT, UniaxialReinforcement? reinforcement = null, LengthUnit unit = LengthUnit.Millimeter)
			: this(nodes, grip1Position, grip3Position, Length.From(width, unit), Length.From(height, unit), concreteParameters, model, reinforcement)
		{
		}

		/// <inheritdoc cref="Stringer" />
		/// <inheritdoc cref="Stringer" />
		public Stringer(IEnumerable<SPMElements.Node> nodes, Point grip1Position, Point grip3Position, Length width, Length height, IParameters concreteParameters, ConstitutiveModel model = ConstitutiveModel.MCFT, UniaxialReinforcement? reinforcement = null)
			: this (nodes, new StringerGeometry(grip1Position, grip3Position, width, height), concreteParameters, model, reinforcement)
		{
		}

		/// <param name="geometry">The <see cref="StringerGeometry"/> of this element.</param>
		/// <inheritdoc cref="Stringer" />
		public Stringer(IEnumerable<SPMElements.Node> nodes, StringerGeometry geometry, IParameters concreteParameters, ConstitutiveModel model = ConstitutiveModel.MCFT, UniaxialReinforcement? reinforcement = null)
		{
			Geometry       = geometry;
			Grip1          = nodes.GetByPosition(Geometry.InitialPoint);
			Grip2          = nodes.GetByPosition(Geometry.CenterPoint);
			Grip3          = nodes.GetByPosition(Geometry.EndPoint);
			Reinforcement  = reinforcement;
			Concrete       = new UniaxialConcrete(concreteParameters, ConcreteArea, model);

			if (!(Reinforcement is null))
				Reinforcement.ConcreteArea = Geometry.CrossSection.Area;
		}

		#endregion

		#region  Methods

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

		public void SetDisplacements(Vector<double> globalDisplacementVector)
		{
			var u = globalDisplacementVector;
			var ind = DoFIndex;

			// Get the displacements
			var us = Vector<double>.Build.Dense(6);
			for (var i = 0; i < ind.Length; i++)
			{
				// Global index
				var j = ind[i];

				// Set values
				us[i] = u[j];
			}

			// Set
			Displacements = us;
		}

		public virtual void Analysis(Vector<double>? globalDisplacements = null)
		{
			// Set displacements
			if (globalDisplacements != null)
				SetDisplacements(globalDisplacements);

			LocalForces = CalculateForces();
		}

		/// <summary>
		///     Calculate the transformation matrix.
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
		///     Calculate local stiffness <see cref="Matrix" />.
		/// </summary>
		/// <returns></returns>
		protected Matrix<double> CalculateStiffness()
		{
			// Calculate the constant factor of stiffness
			var k = Concrete.Stiffness.Newtons / (3 * Geometry.Length.Millimeters);

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
		///     Calculate Stringer forces.
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

		public int CompareTo(Stringer? other) => other is null
			? 1
			: Geometry.CompareTo(other.Geometry);

		/// <summary>
		///     Returns true if <see cref="Geometry" /> of <paramref name="other" /> is equal to this.
		/// </summary>
		/// <param name="other"></param>
		public bool Equals(Stringer? other) => !(other is null) && Geometry == other.Geometry;

		/// <summary>
		///     Returns true if <paramref name="obj" /> is <see cref="Stringer" /> and <see cref="Geometry" /> of
		///     <paramref name="obj" /> is equal to this.
		/// </summary>
		public override bool Equals(object? obj) => obj is Stringer other && Equals(other);

		public override int GetHashCode() => Geometry.GetHashCode();

		public override string ToString()
		{
			var msgstr =
				$"Stringer {Number}\n\n" +
				$"Grips: ({Grips[0]} - {Grips[1]} - {Grips[2]})\n" +
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
		public static bool operator == (Stringer left, Stringer right) => left != null && left.Equals(right);

		/// <summary>
		///     Returns true if arguments are different.
		///     <para>See:<seealso cref="Equals(Stringer)" />.</para>
		/// </summary>
		public static bool operator != (Stringer left, Stringer right) => left != null && !left.Equals(right);

		#endregion
	}
}