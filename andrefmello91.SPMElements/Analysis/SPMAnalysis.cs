using System;
using System.Collections.Generic;
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
		public event EventHandler<SPMElementEventArgs>? ElementCracked;

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

			if (FemInput is not SPMInput spmInput)
				return;

			// Check if a stringer cracked at the current step
			foreach (var element in spmInput)
			{
				if (element is not INonlinearSPMElement nonlinearSpmElement)
					continue;

				if (_crackedElements.Contains(nonlinearSpmElement.Name) || !nonlinearSpmElement.ConcreteCracked)
					continue;

				_crackedElements.Add(nonlinearSpmElement.Name);
				Invoke(ElementCracked, new SPMElementEventArgs(nonlinearSpmElement, (int) CurrentStep));
			}
		}

		#endregion

	}
}