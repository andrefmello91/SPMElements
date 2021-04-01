using System.Collections.Generic;
using System.Linq;
using andrefmello91.FEMAnalysis;

namespace andrefmello91.SPMElements
{
	/// <summary>
	///		SPMInput class.
	/// </summary>
	public class SPMInput : FEMInput
	{
		/// <summary>
		///		Get the collection of stringers.
		/// </summary>
		public List<Stringer> Stringers { get; }
		
		/// <summary>
		///		Get the collection of panels
		/// </summary>
		public List<Panel> Panels { get; }
		
		/// <summary>
		///		Get the collection of nodes.
		/// </summary>
		public List<Node> Nodes { get; }
		
		/// <inheritdoc cref="SPMInput(IEnumerable{Stringer}, IEnumerable{Panel}, IEnumerable{Node})"/>
		/// <remarks>
		///		Nodes are taken from <paramref name="stringers"/> and <paramref name="panels"/>.
		/// </remarks>
		public SPMInput(IEnumerable<Stringer> stringers, IEnumerable<Panel> panels)
			: this(stringers, panels, stringers
				.SelectMany(s => s.Grips)
				.Concat(panels.SelectMany(p => p.Grips))
				.Distinct())
		{
		}

		/// <summary>
		///		SPMInput constructor.
		/// </summary>
		/// <param name="stringers">The collection of <see cref="Stringer"/>'s.</param>
		/// <param name="panels">The collection of <see cref="Panels"/>'s.</param>
		/// <param name="nodes">The collection of <see cref="Nodes"/>'s.</param>
		public SPMInput(IEnumerable<Stringer> stringers, IEnumerable<Panel> panels, IEnumerable<Node> nodes)
			: base(stringers.Concat<IFiniteElement>(panels))
		{
			Stringers = stringers.ToList();
			Panels    = panels.ToList();
			Nodes     = nodes.ToList();
		}
		
		/// <inheritdoc />
		public override string ToString() =>
			$"Number of nodes: {Nodes.Count}\n" +
			$"Number of stringers: {Stringers.Count}\n" +
			$"Number of panels: {Panels.Count}\n" +
			$"Force vector: \n{ForceVector}\n" +
			$"Constraint Index: {ConstraintIndex.Select(i => i.ToString()).Aggregate((i, f) => $"{i} - {f}")}";
	}
}