using System.Collections.Generic;
using System.Linq;
using andrefmello91.FEMAnalysis;
using andrefmello91.OnPlaneComponents;
using andrefmello91.SPMElements.PanelProperties;
using andrefmello91.SPMElements.StringerProperties;

namespace andrefmello91.SPMElements;

/// <summary>
///     SPMInput class.
/// </summary>
public class SPMInput : FEMInput
{

	/// <summary>
	///     Get a node at a <see cref="position" />.
	/// </summary>
	/// <param name="position">The required position.</param>
	public Node? this[Point position] => Nodes.Find(n => n.Position == position);

	/// <summary>
	///     Get a stringer that correspond to a <see cref="stringerGeometry" />.
	/// </summary>
	/// <param name="stringerGeometry">The required position.</param>
	public Stringer? this[StringerGeometry stringerGeometry] => Stringers.Find(n => n.Geometry == stringerGeometry);

	/// <summary>
	///     Get a panel that correspond to a <see cref="panelVertices" />.
	/// </summary>
	/// <param name="panelVertices">The required vertices.</param>
	public Panel? this[Vertices panelVertices] => Panels.Find(n => n.Geometry.Vertices == panelVertices);

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

	/// <inheritdoc cref="SPMInput(IEnumerable{Stringer}, IEnumerable{Panel}, IEnumerable{Node})" />
	/// <remarks>
	///     Nodes are taken from <paramref name="stringers" /> and <paramref name="panels" />.
	/// </remarks>
	private SPMInput(IEnumerable<Stringer> stringers, IEnumerable<Panel> panels)
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
	private SPMInput(IEnumerable<Stringer> stringers, IEnumerable<Panel> panels, IEnumerable<Node> nodes)
		: base(stringers.Concat<ISPMElement>(panels).ToList(), nodes)
	{
		Stringers = stringers.ToList();
		Panels    = panels.ToList();
		Nodes     = nodes.ToList();

		// Set stringer dimensions
		Panels.SetStringerDimensions(Stringers);
	}

	/// <inheritdoc cref="From(IEnumerable{Stringer},IEnumerable{Panel},AnalysisType)" />
	/// <param name="nodes">The collection of <see cref="Nodes" />'s.</param>
	public static SPMInput From(IEnumerable<Stringer> stringers, IEnumerable<Panel> panels, IEnumerable<Node> nodes, AnalysisType analysisType = AnalysisType.Linear)
	{
		var model = analysisType.AsElementModel();

		return analysisType switch
		{
			AnalysisType.Linear => new SPMInput(stringers, panels, nodes),
			_                   => new SPMInput(stringers.Select(s => s.As(model)).ToList(), panels.Select(p => p.As(model)).ToList(), nodes)
		};
	}

	/// <summary>
	///     Create a <see cref="SPMInput" /> from element collections.
	/// </summary>
	/// <param name="stringers">The collection of <see cref="Stringer" />'s.</param>
	/// <param name="panels">The collection of <see cref="Panels" />'s.</param>
	/// <param name="analysisType">The <see cref="AnalysisType" /> to perform.</param>
	public static SPMInput From(IEnumerable<Stringer> stringers, IEnumerable<Panel> panels, AnalysisType analysisType = AnalysisType.Linear)
	{
		var model = analysisType.AsElementModel();

		return analysisType switch
		{
			AnalysisType.Linear => new SPMInput(stringers, panels),
			_                   => new SPMInput(stringers.Select(s => s.As(model)).ToList(), panels.Select(p => p.As(model)).ToList())
		};
	}

	/// <inheritdoc />
	public override string ToString() =>
		$"Number of nodes: {Nodes.Count}\n" +
		$"Number of stringers: {Stringers.Count}\n" +
		$"Number of panels: {Panels.Count}\n" +
		$"Force vector: \n{Forces}\n" +
		$"Constraint Index: {ConstraintIndex.Select(i => i.ToString()).Aggregate((i, f) => $"{i} - {f}")}";
}