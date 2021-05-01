﻿using System.Collections.Generic;
using System.Linq;
using andrefmello91.FEMAnalysis;

namespace andrefmello91.SPMElements
{
	/// <summary>
	///     SPMInput class.
	/// </summary>
	public class SPMInput : FEMInput<ISPMElement>, IFEMInput<IFiniteElement>
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

		#region Interface Implementations

		/// <inheritdoc />
		List<IFiniteElement> IFEMInput<IFiniteElement>.Elements => _elements;

		#endregion

		#endregion

		#region Constructors

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

			_elements = Elements.Cast<IFiniteElement>().ToList();
		}

		#endregion

		#region Methods

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

		#region Object override

		/// <inheritdoc />
		public override string ToString() =>
			$"Number of nodes: {Nodes.Count}\n" +
			$"Number of stringers: {Stringers.Count}\n" +
			$"Number of panels: {Panels.Count}\n" +
			$"Force vector: \n{ForceVector}\n" +
			$"Constraint Index: {ConstraintIndex.Select(i => i.ToString()).Aggregate((i, f) => $"{i} - {f}")}";

		#endregion

		#endregion

	}
}