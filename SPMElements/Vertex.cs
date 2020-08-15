using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace SPMElements
{
    /// <summary>
    /// Vertex struct.
    /// </summary>
    public struct Vertex : IEquatable<Vertex>
    {
        /// <summary>
        /// X coordinate, in mm.
        /// </summary>
        public double X { get; }

        /// <summary>
        /// Y coordinate, in mm.
        /// </summary>
        public double Y { get; }

        /// <summary>
        /// Vertex object.
        /// </summary>
        /// <param name="x">X coordinate, in mm.</param>
        /// <param name="y">Y coordinate, in mm.</param>
        public Vertex(double x, double y)
        {
	        X = x;
	        Y = y;
        }

        /// <summary>
        /// Compare with another vertex.
        /// </summary>
        /// <param name="other">The vertex to compare.</param>
        /// <returns></returns>
        public bool Equals(Vertex other) => X == other.X && Y == other.Y;
		
        /// <summary>
        /// Calculate the distance to another vertex, in mm.
        /// </summary>
        /// <param name="other">The other vertex.</param>
        public double DistanceTo(Vertex other)
        {
	        if (Equals(other))
		        return 0;

	        double
		        lx = other.X - X,
		        ly = other.Y - Y;

	        return
		        Math.Sqrt(lx * lx + ly * ly);
        }

        /// <summary>
        /// Calculate the angle to another vertex, in radians.
        /// </summary>
        /// <param name="other">The other vertex.</param>
        public double AngleTo(Vertex other)
        {
	        if (Equals(other))
		        return 0;

            double
                lx = other.X - X,
		        ly = other.Y - Y;

	        return
		        Math.Atan(ly / lx);
        }

        public override bool Equals(object obj)
        {
	        if (obj is Vertex other)
		        return Equals(other);

	        return false;
        }

        public override int GetHashCode()
        {
	        return (int)X ^ (int)Y;
        }

        public static bool operator == (Vertex lhs, Vertex rhs)
        {
	        return lhs.Equals(rhs);
        }

        public static bool operator != (Vertex lhs, Vertex rhs)
        {
	        return !(lhs.Equals(rhs));
        }
    }
}
