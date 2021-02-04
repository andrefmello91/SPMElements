using System;
using System.Collections.Generic;
using System.Linq;
using OnPlaneComponents;
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
	    public static Node GetByPosition(this IEnumerable<Node> nodes, Point position) => nodes.First(node => position == node.Position);

        /// <summary>
        /// Return the <see cref="INumberedElement"/> of an <see cref="Array"/>, in given <paramref name="number"/>.
        /// </summary>
        /// <param name="elements">The collection of <see cref="INumberedElement"/>'s.</param>
        /// <param name="number">The number of the element wanted.</param>
        public static INumberedElement GetByNumber(this IEnumerable<INumberedElement> elements, int number) => elements.First(element => number == element.Number);

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

        /// <summary>
        /// Get global indexes of an element's grips
        /// </summary>
        /// <param name="gripNumbers">The grip numbers of the element.</param>
        public static IEnumerable<int> GlobalIndexes(IEnumerable<int> gripNumbers)
        {
	        // Initialize the array
	        var count = gripNumbers.Count();

	        // Get the indexes
	        for (int i = 0; i < count; i++)
	        {
		        var n = 2 * gripNumbers.ElementAt(i);

		        yield return n - 2;
		        yield return n - 1;
	        }
        }

        /// <summary>
        /// Get global indexes of an element's grips
        /// </summary>
        /// <param name="gripNumber">The grip number of the element.</param>
        public static IEnumerable<int> GlobalIndexes(int gripNumber)
        {
	        var n = 2 * gripNumber;

	        yield return n - 2;
	        yield return n - 1;
        }

    }
}
