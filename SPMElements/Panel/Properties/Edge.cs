using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Autodesk.AutoCAD.Geometry;
using Extensions;
using Extensions.AutoCAD;
using MathNet.Numerics;
using UnitsNet;
using UnitsNet.Units;

namespace SPMElements.PanelProperties
{
	/// <summary>
    /// Panel edge struct.
    /// </summary>
    public struct Edge : IEquatable<Edge>
    {
		// Auxiliary fields
		private Length _length;

		/// <summary>
		/// Get the <see cref="LengthUnit"/> that this was constructed with.
		/// </summary>
		public LengthUnit Unit => _length.Unit;

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
		public double Angle { get; }

		/// <summary>
        /// Get length, in mm.
        /// </summary>
		public double Length => _length.Millimeters;

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
			_length       = UnitsNet.Length.From(initialVertex.DistanceTo(finalVertex), geometryUnit);
			Angle         = initialVertex.AngleTo(finalVertex);
		}

		/// <summary>
		/// Change the <see cref="LengthUnit"/> of this.
		/// </summary>
		/// <param name="unit">The <see cref="LengthUnit"/> to convert.</param>
		public void ChangeUnit(LengthUnit unit)
		{
			if (Unit == unit)
				return;

			_length = _length.ToUnit(unit);
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
				$"Lenght = {_length}\n" + 
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
