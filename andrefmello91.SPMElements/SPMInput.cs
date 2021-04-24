using System.Collections.Generic;
using System.Linq;
using andrefmello91.FEMAnalysis;

namespace andrefmello91.SPMElements
{
	/// <summary>
	///     SPMInput class.
	/// </summary>
	public class SPMInput : FEMInput<SPMElement>, IFEMInput<IFiniteElement>
	{

		#region Fields

		private readonly List<IFiniteElement> _elements;

		#endregion

		#region Properties

		/// <summary>
		///     Get the collection of nodes.
		/// </summary>
		public List<Node> Nodes { get; }

		/// <summary>
		///     Get the collection of panels
		/// </summary>
		public List<Panel> Panels { get; }

		/// <summary>
		///     Get the collection of stringers.
		/// </summary>
		public List<Stringer> Stringers { get; }

		/// <inheritdoc />
		List<IFiniteElement> IFEMInput<IFiniteElement>.Elements => _elements;

		#endregion

		#region Constructors

		/// <inheritdoc cref="SPMInput(IEnumerable{Stringer}, IEnumerable{Panel}, IEnumerable{Node})" />
		/// <remarks>
		///     Nodes are taken from <paramref name="stringers" /> and <paramref name="panels" />.
		/// </remarks>
		public SPMInput(IEnumerable<Stringer> stringers, IEnumerable<Panel> panels)
			: this(stringers, panels, stringers
				.SelectMany(s => s.Grips)
				.Concat(panels.SelectMany(p => p.Grips))
				.Distinct()
				.ToList())
		{
		}

		/// <summary>
		///     SPMInput constructor.
		/// </summary>
		/// <param name="stringers">The collection of <see cref="Stringer" />'s.</param>
		/// <param name="panels">The collection of <see cref="Panels" />'s.</param>
		/// <param name="nodes">The collection of <see cref="Nodes" />'s.</param>
		public SPMInput(IEnumerable<Stringer> stringers, IEnumerable<Panel> panels, IEnumerable<Node> nodes)
			: base(stringers.Concat<SPMElement>(panels).ToList(), nodes)
		{
			Stringers = stringers.ToList();
			Panels    = panels.ToList();
			Nodes     = nodes.ToList();

			// Set stringer dimensions
			Panels.SetStringerDimensions(Stringers);

			_elements = Elements.Cast<IFiniteElement>().ToList();
		}

		#endregion

		#region Methods

		/// <summary>
		///     Create a <see cref="SPMInput" /> from element collections.
		/// </summary>
		/// <inheritdoc cref="SPMInput(IEnumerable{Stringer}, IEnumerable{Panel}, IEnumerable{Node})" />
		/// <param name="analysisType">The <see cref="AnalysisType" /> to perform.</param>
		/// <returns>
		///     <see cref="SPMInput" /> if <paramref name="analysisType" /> is <see cref="AnalysisType.Linear" />,
		///     <see cref="NLSPMInput" /> otherwise.
		/// </returns>
		public static SPMInput From(IEnumerable<Stringer> stringers, IEnumerable<Panel> panels, IEnumerable<Node> nodes, AnalysisType analysisType = AnalysisType.Linear) =>
			analysisType switch
			{
				AnalysisType.Linear => new SPMInput(stringers, panels, nodes),
				_                   => new SPMInput(stringers.Select(s => s.ToNonlinear()).ToList(), panels.Select(p => p.ToNonlinear()).ToList(), nodes)
			};

		/// <inheritdoc cref="From(IEnumerable{Stringer},IEnumerable{Panel},IEnumerable{Node},AnalysisType)" />
		public static SPMInput From(IEnumerable<Stringer> stringers, IEnumerable<Panel> panels, AnalysisType analysisType = AnalysisType.Linear) =>
			analysisType switch
			{
				AnalysisType.Linear => new SPMInput(stringers, panels),
				_                   => new SPMInput(stringers.Select(s => s.ToNonlinear()).ToList(), panels.Select(p => p.ToNonlinear()).ToList())
			};

		/// <inheritdoc />
		public override string ToString() =>
			$"Number of nodes: {Nodes.Count}\n" +
			$"Number of stringers: {Stringers.Count}\n" +
			$"Number of panels: {Panels.Count}\n" +
			$"Force vector: \n{ForceVector}\n" +
			$"Constraint Index: {ConstraintIndex.Select(i => i.ToString()).Aggregate((i, f) => $"{i} - {f}")}";

		#endregion

	}
}