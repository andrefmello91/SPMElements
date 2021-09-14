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

		private readonly List<string> _crackedElements = new();

		// Load steps of element cracking
		private (int number, int step)? _stringerCrackLS, _panelCrackLS;

		#endregion

		#region Events

		/// <summary>
		///     Event to execute when an element cracks.
		/// </summary>
		public event EventHandler<SPMElementEventArgs>? ElementsCracked;

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

		// /// <summary>
		// ///     Generate an <see cref="SPMOutput" /> from analysis results.
		// /// </summary>
		// public new SPMOutput GenerateOutput() => new(Steps, _stringerCrackLS, _panelCrackLS);

		/// <inheritdoc />
		protected override void SetStepResults(int? monitoredIndex)
		{
			base.SetStepResults(monitoredIndex);

			// Check cracked elements
			CheckCracking();
		}

		private void CheckCracking()
		{
			var crackedElements = FemInput
				.Where(e => e is INonlinearSPMElement nle && !_crackedElements.Contains(nle.Name) && nle.ConcreteCracked)
				.Cast<INonlinearSPMElement>()
				.ToList();

			if (!crackedElements.Any())
				return;

			_crackedElements.AddRange(crackedElements.Select(e => e.Name));
			Invoke(ElementsCracked, new SPMElementEventArgs(crackedElements, (int) CurrentStep));
		}

		#endregion

	}
}