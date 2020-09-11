using System;
using System.Linq;
using Autodesk.AutoCAD.Geometry;
using Extensions.AutoCAD;
using SPMElements.PanelProperties;

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
	    public static Node GetByPosition(this Node[] nodes, Point3d position) => nodes.First(node => position.Approx(node.Position));

	    /// <summary>
        /// Return the <see cref="SPMElement"/> of an <see cref="Array"/>, in given <paramref name="number"/>.
        /// </summary>
        /// <param name="elements">The <see cref="Array"/> of <see cref="SPMElement"/>.</param>
        /// <param name="number">The number of the node wanted.</param>
        public static SPMElement GetByNumber(this SPMElement[] elements, int number) => elements.First(element => number == element.Number);

        /// <summary>
        /// Set stringer dimensions on edges of each panel.
        /// <para>See: <see cref="Edge.SetStringerDimension"/></para>
        /// </summary>
        /// <param name="stringers">The array containing all of the stringers.</param>
        public static void SetStringerDimensions(this NonLinearPanel[] panels, Stringer[] stringers)
        {
	        foreach (var panel in panels)
		        panel.SetEdgeStringersDimensions(stringers);
        }
    }
}
