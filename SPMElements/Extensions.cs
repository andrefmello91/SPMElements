using System;
using System.Collections.Generic;
using System.Linq;
using Autodesk.AutoCAD.Geometry;
using Extensions.AutoCAD;
using SPM.Elements.PanelProperties;

namespace SPM.Elements
{
	/// <summary>
    /// Extensions class.
    /// </summary>
    public static class Extensions
    {
	    /// <summary>
	    /// Return the <see cref="Node"/> of an array, in given <paramref name="position"/>.
	    /// </summary>
	    /// <param name="nodes">The collection of <see cref="Node"/>'s.</param>
	    /// <param name="position">The position wanted.</param>
	    public static Node GetByPosition(this IEnumerable<Node> nodes, Point3d position) => nodes.First(node => position.Approx(node.Position));

        /// <summary>
        /// Return the <see cref="SPMElement"/> of an <see cref="Array"/>, in given <paramref name="number"/>.
        /// </summary>
        /// <param name="elements">The collection of <see cref="SPMElement"/>'s.</param>
        /// <param name="number">The number of the element wanted.</param>
        public static SPMElement GetByNumber(this IEnumerable<SPMElement> elements, int number) => elements.First(element => number == element.Number);

        /// <summary>
        /// Set stringer dimensions on edges of each panel.
        /// <para>See: <see cref="Edge.SetStringerDimension"/></para>
        /// </summary>
        /// <param name="stringers">The array containing all of the stringers.</param>
        public static void SetStringerDimensions(this IEnumerable<NLPanel> panels, IEnumerable<Stringer> stringers)
        {
	        foreach (var panel in panels)
		        panel.SetEdgeStringersDimensions(stringers);
        }
    }
}
