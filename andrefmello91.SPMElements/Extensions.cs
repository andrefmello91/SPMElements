using System.Collections.Generic;
using System.Linq;
using andrefmello91.OnPlaneComponents;

namespace andrefmello91.SPMElements
{
	/// <summary>
	///     Extensions class.
	/// </summary>
	public static class Extensions
	{
		#region Methods

		/// <summary>
		///     Return the <see cref="Node" /> of an array, in given <paramref name="position" />.
		/// </summary>
		/// <param name="nodes">The collection of <see cref="Node" />'s.</param>
		/// <param name="position">The position wanted.</param>
		public static Node GetByPosition(this IEnumerable<Node> nodes, Point position) => nodes.First(node => position == node.Position);

		/// <summary>
		///     Set stringer dimensions on edges of each panel.
		///     <para>See: <see cref="Edge.SetStringerDimension" /></para>
		/// </summary>
		/// <param name="stringers">The array containing all of the stringers.</param>
		public static void SetStringerDimensions(this IEnumerable<NLPanel> panels, IEnumerable<Stringer> stringers)
		{
			foreach (var panel in panels)
				panel.SetStringersDimensions(stringers);
		}

		#endregion
	}
}