using System.Collections.Generic;
using andrefmello91.FEMAnalysis;

namespace andrefmello91.SPMElements
{
	/// <summary>
	///     Output class for SPM analysis.
	/// </summary>
	public class SPMOutput : FEMOutput
	{

		#region Properties

		/// <summary>
		///     Get the number of the panel and the load step where the first crack appears at the panel.
		/// </summary>
		/// <remarks>
		///     <b>number</b>: the number of the cracked panel.
		///     <para>
		///         <b>step</b>: the number of the load step.
		///     </para>
		/// </remarks>
		public (int number, int step)? PanelCrackLoadStep { get; }

		/// <summary>
		///     Get the number of the stringer and the load step where the first crack appears at the stringer.
		/// </summary>
		/// <remarks>
		///     <b>number</b>: the number of the cracked cracked.
		///     <para>
		///         <b>step</b>: the number of the load step.
		///     </para>
		/// </remarks>
		public (int number, int step)? StringerCrackLoadStep { get; }

		#endregion

		#region Constructors

		/// <inheritdoc />
		/// <param name="stringerCrackLoadStep">
		///     The number of the stringer and the load step where the first crack appears at the
		///     stringer.
		/// </param>
		/// <param name="panelCrackLoadStep">The number of the panel and the load step where the first crack appears at the panel.</param>
		public SPMOutput(IEnumerable<LoadStep> loadStepResults, (int number, int step)? stringerCrackLoadStep, (int number, int step)? panelCrackLoadStep)
			: base(loadStepResults)
		{
			StringerCrackLoadStep = stringerCrackLoadStep;
			PanelCrackLoadStep    = panelCrackLoadStep;
		}

		#endregion

	}
}