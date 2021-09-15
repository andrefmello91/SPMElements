using System;
using System.Collections.Generic;
using System.Linq;
using andrefmello91.FEMAnalysis;

namespace andrefmello91.SPMElements
{
	/// <summary>
	///     Nonlinear analysis class for SPM.
	/// </summary>
	public class SPMAnalysis : NonlinearAnalysis
	{

		#region Fields

		private readonly List<string>
			_crackedElements = new(),
			_yieldedElements = new(),
			_crushedElements = new();

		#endregion

		#region Events

		/// <summary>
		///     Event to execute when elements crack.
		/// </summary>
		public event EventHandler<SPMElementEventArgs>? ElementsCracked;

		/// <summary>
		///     Event to execute when elements crush.
		/// </summary>
		public event EventHandler<SPMElementEventArgs>? ElementsCrushed;

		/// <summary>
		///     Event to execute when elements yield.
		/// </summary>
		public event EventHandler<SPMElementEventArgs>? ElementsYielded;

		#endregion

		#region Constructors

		/// <inheritdoc />
		public SPMAnalysis(IFEMInput nonlinearInput, int? monitoredIndex = null, bool simulate = false)
			: base(nonlinearInput, monitoredIndex, simulate)
		{
		}
		/// <inheritdoc />
		public SPMAnalysis(IFEMInput nonlinearInput, AnalysisParameters parameters, int? monitoredIndex = null, bool simulate = false)
			: base(nonlinearInput, parameters, monitoredIndex, simulate)
		{
		}

		#endregion

		#region Methods

		/// <inheritdoc />
		protected override void CorrectResults()
		{
			base.CorrectResults();

			// Check elements and set last step
			CheckCracking(LastStep);
			CheckYielding(LastStep);
			CheckCrushing(LastStep);
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
			CheckCracking(CurrentStep);
			CheckYielding(CurrentStep);
			CheckCrushing(CurrentStep);
		}

		/// <summary>
		///     Check if elements cracked.
		/// </summary>
		private void CheckCracking(LoadStep step)
		{
			var crackedElements = FemInput
				.Where(e => e is INonlinearSPMElement nle && !_crackedElements.Contains(nle.Name) && nle.ConcreteCracked)
				.Cast<INonlinearSPMElement>()
				.ToList();

			if (!crackedElements.Any())
				return;

			_crackedElements.AddRange(crackedElements.Select(e => e.Name));
			Invoke(ElementsCracked, new SPMElementEventArgs(crackedElements, (int) step));
		}

		/// <summary>
		///     Check if elements crushed.
		/// </summary>
		private void CheckCrushing(LoadStep step)
		{
			var crushed = FemInput
				.Where(e => e is INonlinearSPMElement nle && !_crushedElements.Contains(nle.Name) && nle.ConcreteCrushed)
				.Cast<INonlinearSPMElement>()
				.ToList();

			if (!crushed.Any())
				return;

			_crushedElements.AddRange(crushed.Select(e => e.Name));
			Invoke(ElementsCrushed, new SPMElementEventArgs(crushed, (int) step));
		}

		/// <summary>
		///     Check if elements yielded.
		/// </summary>
		private void CheckYielding(LoadStep step)
		{
			var yielded = FemInput
				.Where(e => e is INonlinearSPMElement nle && !_yieldedElements.Contains(nle.Name) && (nle.SteelYielded || nle.ConcreteYielded))
				.Cast<INonlinearSPMElement>()
				.ToList();

			if (!yielded.Any())
				return;

			_yieldedElements.AddRange(yielded.Select(e => e.Name));
			Invoke(ElementsYielded, new SPMElementEventArgs(yielded, (int) step));
		}

		#endregion

	}
}