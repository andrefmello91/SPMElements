using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Autodesk.AutoCAD.Geometry;

namespace SPMElements
{
	/// <summary>
    /// Extensions class.
    /// </summary>
    public static class Extensions
    {
	    /// <summary>
	    /// Return the <see cref="Node"/> of an array, in given <paramref name="position"/>.
	    /// </summary>
	    /// <param name="nodes">The array of nodes.</param>
	    /// <param name="position">The position wanted.</param>
	    /// <returns></returns>
	    public static Node GetByPosition(this Node[] nodes, Point3d position)
	    {
		    foreach (var node in nodes)
		    {
			    if (position == node.Position)
				    return node;
		    }

		    return null;
	    }

	    /// <summary>
	    /// Return the <see cref="Node"/> of an array, in given <paramref name="number"/>.
	    /// </summary>
	    /// <param name="nodes">The array of nodes.</param>
	    /// <param name="number">The number of the node wanted.</param>
	    /// <returns></returns>
	    public static Node GetByNumber(this Node[] nodes, int number)
	    {
		    foreach (var node in nodes)
		    {
			    if (number == node.Number)
				    return node;
		    }

		    return null;
	    }

    }
}
