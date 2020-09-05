using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Autodesk.AutoCAD.Geometry;
using MathNet.Numerics;
using UnitsNet;
using UnitsNet.Units;

namespace SPMElements.PanelGeometry
{
	/// <summary>
    /// Panel edge struct.
    /// </summary>
    public struct Edge : IEquatable<Edge>
    {
		// Auxiliary fields
		private Length _lenght;
		private double? _angle;

		/// <summary>
        /// Get the initial vertex of this <see cref="Edge"/>.
        /// </summary>
		public Point3d InitialVertex { get; }

		/// <summary>
		/// Get the final vertex of this <see cref="Edge"/>.
		/// </summary>
		public Point3d FinalVertex { get; }

		/// <summary>
		/// Get angle related to horizontal axis, in radians.
		/// </summary>
		public double Angle
		{
			get
			{
				if (!_angle.HasValue)
					CalculateAngle();

				return _angle.Value;
			}
		}

		/// <summary>
        /// Get length, in mm.
        /// </summary>
		public double Length => _lenght.Millimeters;

		/// <summary>
        /// Panel edge constructor.
        /// </summary>
        /// <param name="initialVertex">The initial vertex.</param>
        /// <param name="finalVertex">The final vertex.</param>
        /// <param name="geometryUnit">The <see cref="LengthUnit"/> of vertices coordinates.</param>
		public Edge(Point3d initialVertex, Point3d finalVertex, LengthUnit geometryUnit = LengthUnit.Millimeter)
		{
			InitialVertex = initialVertex;
			FinalVertex   = finalVertex;
			_lenght       = UnitsNet.Length.From(initialVertex.DistanceTo(finalVertex), geometryUnit);
			_angle        = null;
		}

		/// <summary>
        /// Calculate edge angle.
        /// </summary>
		private void CalculateAngle()
		{
			double
				x = FinalVertex.X - InitialVertex.X,
				y = FinalVertex.Y - InitialVertex.Y;

			_angle = Trig.Atan(y / x).CoerceZero(1E-6);
		}

		/// <summary>
        /// Returns true if vertices are equal.
        /// </summary>
        /// <param name="other">The other <see cref="Edge"/> object.</param>
		public bool Equals(Edge other) => InitialVertex == other.InitialVertex && FinalVertex == other.FinalVertex;

		public override bool Equals(object obj) => obj is Edge other && Equals(other);

		public override int GetHashCode() => (int) (InitialVertex.X * FinalVertex.X + InitialVertex.Y * FinalVertex.Y);

		public override string ToString()
		{
			return
				$"Initial vertex: ({InitialVertex.X:0.00}, {InitialVertex.Y:0.00})\n" +
				$"Final vertex: ({FinalVertex.X:0.00}, {FinalVertex.Y:0.00})\n" +
				$"Lenght = {_lenght}\n" + 
				$"Angle = {Angle:0.00} rad";
		}

		/// <summary>
		/// Returns true if arguments are equal.
		/// </summary>
		public static bool operator == (Edge left, Edge right) => left.Equals(right);

		/// <summary>
		/// Returns true if arguments are different.
		/// </summary>
		public static bool operator != (Edge left, Edge right) => !left.Equals(right);
    }
}
