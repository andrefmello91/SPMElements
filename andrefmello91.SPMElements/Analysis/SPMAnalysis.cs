using System;
using System.Collections.Generic;
using System.Linq;
using andrefmello91.FEMAnalysis;

namespace andrefmello91.SPMElements;

/// <summary>
///     Nonlinear analysis class for SPM.
/// </summary>
public class SPMAnalysis : NonlinearAnalysis
{

	#region Fields

	private readonly List<INonlinearSPMElement>
		_concreteYieldedElements = new();

	private readonly List<INonlinearSPMElement>
		_crackedElements = new();

	private readonly List<INonlinearSPMElement>
		_crushedElements = new();

	private readonly List<INonlinearSPMElement>
		_steelYieldedElements = new();

	#endregion

	#region Events

	/// <summary>
	///     Event to execute when concrete of elements yield.
	/// </summary>
	public event EventHandler<SPMElementEventArgs>? ElementsConcreteYielded;

	/// <summary>
	///     Event to execute when elements crack.
	/// </summary>
	public event EventHandler<SPMElementEventArgs>? ElementsCracked;

	/// <summary>
	///     Event to execute when elements crush.
	/// </summary>
	public event EventHandler<SPMElementEventArgs>? ElementsCrushed;

	/// <summary>
	///     Event to execute when steel of elements yield.
	/// </summary>
	public event EventHandler<SPMElementEventArgs>? ElementsSteelYielded;

	#endregion

	#region Constructors

	/// <inheritdoc />
	public SPMAnalysis(IFEMInput nonlinearInput, int? monitoredIndex = null, bool simulate = false)
		: base(nonlinearInput, monitoredIndex, simulate)
	{
		SetEvents();
	}
	/// <inheritdoc />
	public SPMAnalysis(IFEMInput nonlinearInput, AnalysisParameters parameters, int? monitoredIndex = null, bool simulate = false)
		: base(nonlinearInput, parameters, monitoredIndex, simulate)
	{
		SetEvents();
	}

	#endregion

	#region Methods

	/// <inheritdoc />
	protected override void CorrectResults()
	{
		base.CorrectResults();

		// Check elements and set last step
		CheckElements(_crackedElements, ElementsCracked, LastStep);
		CheckElements(_crushedElements, ElementsCrushed, LastStep);
		CheckElements(_concreteYieldedElements, ElementsConcreteYielded, LastStep);
		CheckElements(_steelYieldedElements, ElementsSteelYielded, LastStep);
	}

	// /// <summary>
	// ///     Generate an <see cref="SPMOutput" /> from analysis results.
	// /// </summary>
	// public new SPMOutput GenerateOutput() => new(Steps, _stringerCrackLS, _panelCrackLS);

	/// <inheritdoc />
	protected override void SetStepResults(int? monitoredIndex)
	{
		base.SetStepResults(monitoredIndex);

		// Check elements
		CheckElements(_crackedElements, ElementsCracked, CurrentStep);
		CheckElements(_crushedElements, ElementsCrushed, CurrentStep);
		CheckElements(_concreteYieldedElements, ElementsConcreteYielded, CurrentStep);
		CheckElements(_steelYieldedElements, ElementsSteelYielded, CurrentStep);
	}

	/// <summary>
	///     Check element changes at <paramref name="step" /> and call <paramref name="handler" />.
	/// </summary>
	/// <param name="elements">The collection of elements that changed state.</param>
	/// <param name="handler">The handler to call.</param>
	/// <param name="step">The load step.</param>
	/// <param name="clearList">Clear list after call?</param>
	private void CheckElements(ICollection<INonlinearSPMElement> elements, EventHandler<SPMElementEventArgs>? handler, LoadStep step, bool clearList = true)
	{
		if (!elements.Any())
			return;

		// Check if analysis stopped (add only first element)
		var elmts = Stop
			? new List<INonlinearSPMElement> { elements.First() }
			: elements.ToList();

		Invoke(handler, new SPMElementEventArgs(elmts, (int) step));

		if (clearList)
			elements.Clear();
	}

	/// <summary>
	///     Set events to nonlinear elements.
	/// </summary>
	private void SetEvents()
	{
		var elements = FemInput
			.Where(e => e is INonlinearSPMElement)
			.Cast<INonlinearSPMElement>()
			.ToList();

		foreach (var element in elements)
			element.StateChanged += OnStateChanged;
	}

	private void OnStateChanged(object? sender, StateEventArgs e)
	{
		if (sender is not INonlinearSPMElement element)
			return;

		var list = e.StateName switch
		{
			nameof(INonlinearSPMElement.ConcreteCracked) => _crackedElements,
			nameof(INonlinearSPMElement.ConcreteCrushed) => _crushedElements,
			nameof(INonlinearSPMElement.ConcreteYielded) => _concreteYieldedElements,
			nameof(INonlinearSPMElement.SteelYielded)    => _steelYieldedElements,
			_                                            => null
		};

		list?.Add(element);
	}

	#endregion

}