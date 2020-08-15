﻿using System;
using MathNet.Numerics.LinearAlgebra;
using Material.Concrete;
using Material.Reinforcement;
using Concrete           = Material.Concrete.UniaxialConcrete;
using Reinforcement      = Material.Reinforcement.UniaxialReinforcement;

namespace SPMElements
{
	/// <summary>
    /// Stringer base class;
    /// </summary>
	public class Stringer : SPMElement
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

        // Stringer properties
		public Units                   Units            { get; }
		public int[]                   Grips            { get; }
		public Point3d[]               PointsConnected  { get; }
		public Length				   DrawingLength    { get; }
		public double                  Length           { get; }
		public double                  Angle            { get; }
		public double                  Width            { get; set; }
		public double                  Height           { get; set; }
		public Concrete                Concrete         { get; }
        public Reinforcement           Reinforcement    { get; set; }
        public Matrix<double>          TransMatrix      { get; }
        public virtual Matrix<double>  LocalStiffness   { get; }
		public virtual Vector<double>  Forces           { get; set; }
		public Vector<double>          Displacements    { get; set; }

        /// <summary>
        /// Stringer base object.
        /// </summary>
        /// <param name="stringerObjectId">The object ID of the stringer from AutoCAD drawing.</param>
        /// <param name="units">Units current in use <see cref="SPMTool.Units"/>.</param>
        /// <param name="concreteParameters">The concrete parameters <see cref="Parameters"/>.</param>
        /// <param name="concreteConstitutive">The concrete constitutive <see cref="Constitutive"/>.</param>
        public Stringer(ObjectId stringerObjectId, Units units, Parameters concreteParameters = null, Constitutive concreteConstitutive = null)
		{
			ObjectId = stringerObjectId;
			Units    = units;

			// Read the object as a line
			Line strLine = Geometry.Stringer.ReadStringer(stringerObjectId);

			// Get the length and angles
			DrawingLength = UnitsNet.Length.From(strLine.Length, Units.Geometry);
			Length        = DrawingLength.Millimeters;
			Angle         = strLine.Angle;

			// Calculate midpoint
			var midPt = GlobalAuxiliary.MidPoint(strLine.StartPoint, strLine.EndPoint);

			// Get the points
			PointsConnected = new[] { strLine.StartPoint, midPt, strLine.EndPoint };

			// Read the XData and get the necessary data
			TypedValue[] data = Auxiliary.ReadXData(strLine);

			// Get the Stringer number
			Number = Convert.ToInt32(data[(int) StringerData.Number].Value);

			// Create the list of grips
			Grips = new []
			{
				Convert.ToInt32(data[(int) StringerData.Grip1].Value),
				Convert.ToInt32(data[(int) StringerData.Grip2].Value),
				Convert.ToInt32(data[(int) StringerData.Grip3].Value)
			};

			// Get geometry
			Width  = Convert.ToDouble(data[(int) StringerData.Width].Value);
			Height = Convert.ToDouble(data[(int) StringerData.Height].Value);

			// Get concrete
			Concrete = new Concrete(concreteParameters, Area, concreteConstitutive);

            // Get reinforcement
            int numOfBars = Convert.ToInt32 (data[(int) StringerData.NumOfBars].Value);
			double phi    = Convert.ToDouble(data[(int) StringerData.BarDiam].Value);

			if (numOfBars > 0 && phi > 0)
			{
				// Get steel data
				double
					fy = Convert.ToDouble(data[(int) StringerData.Steelfy].Value),
					Es = Convert.ToDouble(data[(int) StringerData.SteelEs].Value);

				// Set steel data
				var steel = new Steel(fy, Es);

				// Set reinforcement
				Reinforcement = new Reinforcement(numOfBars, phi, Area, steel);
			}

			// Calculate transformation matrix
			TransMatrix = TransformationMatrix();
		}

        /// <summary>
        /// Read the stringer.
        /// </summary>
        /// <param name="analysisType">Type of analysis to perform (<see cref="AnalysisType"/>).</param>
        /// <param name="stringerObjectId">The object ID of the stringer from AutoCAD drawing.</param>
        /// <param name="units">Units current in use <see cref="SPMTool.Units"/>.</param>
        /// <param name="concreteParameters">The concrete parameters <see cref="Parameters"/>.</param>
        /// <param name="concreteConstitutive">The concrete constitutive <see cref="Constitutive"/>.</param>
        public static Stringer ReadStringer(AnalysisType analysisType, ObjectId stringerObjectId, Units units,
			Parameters concreteParameters = null, Constitutive concreteConstitutive = null)
		{
			if (analysisType == AnalysisType.Linear)
				return new LinearStringer(stringerObjectId, units, concreteParameters, concreteConstitutive);

			return new NonLinearStringer(stringerObjectId, units, concreteParameters, concreteConstitutive);
		}

		// Get points
		public Point3d StartPoint => PointsConnected[0];
		public Point3d MidPoint   => PointsConnected[1];
		public Point3d EndPoint   => PointsConnected[2];

		// Set global indexes from grips
		public override int[] DoFIndex => GlobalAuxiliary.GlobalIndexes(Grips);

		// Calculate direction cosines
		public (double cos, double sin) DirectionCosines => GlobalAuxiliary.DirectionCosines(Angle);

		// Calculate steel area
		public double SteelArea => Reinforcement?.Area ?? 0;

		// Calculate concrete area
		public double Area         => Width * Height;
        public double ConcreteArea => Area - SteelArea;

		// Calculate global stiffness
		public Matrix<double> GlobalStiffness => TransMatrix.Transpose() * LocalStiffness * TransMatrix;

		// Calculate local displacements
		public Vector<double> LocalDisplacements => TransMatrix * Displacements;

		/// <summary>
        /// Get normal forces acting in the stringer, in kN.
        /// </summary>
		public (double N1, double N3) NormalForces => (Forces[0], -Forces[2]);

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

        // Global Stringer forces
        public Vector<double> GlobalForces => TransMatrix.Transpose() * Forces;

        // Maximum Stringer force
        public double MaxForce => Forces.AbsoluteMaximum();

        /// <summary>
        /// Calculate the transformation matrix.
        /// </summary>
        private Matrix<double> TransformationMatrix()
        {
	        // Get the direction cosines
	        var (l, m) = DirectionCosines;

	        // Obtain the transformation matrix
	        return Matrix<double>.Build.DenseOfArray(new [,]
	        {
		        {l, m, 0, 0, 0, 0 },
		        {0, 0, l, m, 0, 0 },
		        {0, 0, 0, 0, l, m }
	        });
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
        /// Do analysis of stringer.
        /// </summary>
        /// <param name="globalDisplacements">The global displacement vector.</param>
        /// <param name="numStrainSteps">The number of strain increments (for nonlinear analysis) (default: 5).</param>
        public virtual void Analysis(Vector<double> globalDisplacements = null, int numStrainSteps = 5)
		{
		}

		public override string ToString()
		{
			// Convert units
			Length
				w = UnitsNet.Length.FromMillimeters(Width).ToUnit(Units.Geometry),
				h = UnitsNet.Length.FromMillimeters(Height).ToUnit(Units.Geometry);

            string msgstr =
				"Stringer " + Number + "\n\n" +
				"Grips: (" + Grips[0] + " - " + Grips[1] + " - " + Grips[2] + ")" + "\n" +
				"Lenght = " + DrawingLength + "\n" +
				"Width = "  + w  + "\n" +
				"Height = " + h;

			if (Reinforcement != null)
			{
				msgstr += "\n\n" + Reinforcement.ToString(Units.Reinforcement, Units.MaterialStrength);
			}

			return msgstr;
		}
    }
}

