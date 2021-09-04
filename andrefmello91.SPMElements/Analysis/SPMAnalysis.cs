using System;
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

		// Load steps of element cracking
		private (int number, int step)? _stringerCrackLS, _panelCrackLS;

		#endregion

		#region Events

		/// <summary>
		///     Event to execute when a panel cracks.
		/// </summary>
		public event EventHandler<SPMElementEventArgs>? PanelCracked;

		/// <summary>
		///     Event to execute when a stringer cracks.
		/// </summary>
		public event EventHandler<SPMElementEventArgs>? StringerCracked;

		#endregion

		#region Constructors

		/// <inheritdoc />
		public SPMAnalysis(IFEMInput nonlinearInput)
			: base(nonlinearInput)
		{
		}

		/// <inheritdoc />
		public SPMAnalysis(IFEMInput nonlinearInput, AnalysisParameters parameters)
			: base(nonlinearInput, parameters)
		{
		}

		#endregion

		#region Methods

		/// <summary>
		///     Generate an <see cref="SPMOutput" /> from analysis results.
		/// </summary>
		public new SPMOutput GenerateOutput() => new(Steps, _stringerCrackLS, _panelCrackLS);

		/// <inheritdoc />
		protected override void SetStepResults(int? monitoredIndex)
		{
			base.SetStepResults(monitoredIndex);

			if (FemInput is not SPMInput spmInput)
				return;

			// Check if a stringer cracked at the current step
			if (!_stringerCrackLS.HasValue && spmInput.Stringers.FirstOrDefault(s => s is NLStringer { ConcreteCracked: true }) is NLStringer stringer)
			{
				_stringerCrackLS = (stringer.Number, (int) CurrentStep);
				Invoke(StringerCracked, new SPMElementEventArgs(stringer, (int) CurrentStep));
			}

			// Check if a panel cracked at the current step
			if (!_panelCrackLS.HasValue && spmInput.Panels.FirstOrDefault(s => s is NLPanel { ConcreteCracked: true }) is NLPanel panel)
			{
				_panelCrackLS = (panel.Number, (int) CurrentStep);
				Invoke(PanelCracked, new SPMElementEventArgs(panel, (int) CurrentStep));
			}
		}

		#endregion

	}
}