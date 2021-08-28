using System.Linq;
using andrefmello91.FEMAnalysis;

namespace andrefmello91.SPMElements
{
	/// <summary>
	///		Nonlinear analysis class for SPM.
	/// </summary>
	public class SPMAnalysis : NonlinearAnalysis
	{
		// Load steps of cracking
		private int? _stringerCrackLS, _panelCrackLS;
		
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

		/// <inheritdoc />
		protected override void SetStepResults(int? monitoredIndex)
		{
			base.SetStepResults(monitoredIndex);
			
			if (FemInput is not SPMInput spmInput)
				return;
			
			// Check if a stringer cracked at the current step
			if (!_stringerCrackLS.HasValue && spmInput.Stringers.Any(s => s is NLStringer { ConcreteCracked: true }))
				_stringerCrackLS = (int) CurrentStep;
			
			// Check if a panel cracked at the current step
			if (!_panelCrackLS.HasValue && spmInput.Panels.Any(s => s is NLPanel { ConcreteCracked: true }))
				_panelCrackLS = (int) CurrentStep;
		}

		/// <summary>
		///     Generate an <see cref="SPMOutput" /> from analysis results.
		/// </summary>
		public new SPMOutput GenerateOutput() => new (Steps, _stringerCrackLS, _panelCrackLS);
	}
}