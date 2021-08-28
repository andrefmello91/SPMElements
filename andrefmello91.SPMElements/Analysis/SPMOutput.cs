using System.Collections.Generic;
using andrefmello91.FEMAnalysis;

namespace andrefmello91.SPMElements
{
	/// <summary>
	///		Output class for SPM analysis.
	/// </summary>
	public class SPMOutput : FEMOutput
	{
		/// <summary>
		///		Get the number of the load step where the first crack appears at a stringer.
		/// </summary>
		public int? StringerCrackLoadStep { get; }
		
		/// <summary>
		///		Get the number of the load step where the first crack appears at a panel.
		/// </summary>
		public int? PanelCrackLoadStep { get; }
		
		/// <inheritdoc />
		/// <param name="stringerCrackLoadStep">The number of the load step where the first crack appears at a stringer.</param>
		/// <param name="panelCrackLoadStep">The number of the load step where the first crack appears at a panel.</param>
		public SPMOutput(IEnumerable<LoadStep> loadStepResults, int? stringerCrackLoadStep, int? panelCrackLoadStep)
			: base(loadStepResults)
		{
			StringerCrackLoadStep = stringerCrackLoadStep;
			PanelCrackLoadStep    = panelCrackLoadStep;
		}
	}
}