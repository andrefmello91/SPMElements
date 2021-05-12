using andrefmello91.FEMAnalysis;

namespace andrefmello91.SPMElements
{
	/// <summary>
	///		Nonlinear analysis class for SPM.
	/// </summary>
	public class SPMNonlinearAnalysis : NonlinearAnalysis
	{
		/// <inheritdoc />
		/// <param name="spmInput">The <see cref="SPMInput"/>.</param>
		public SPMNonlinearAnalysis(IFEMInput<ISPMElement> spmInput, NonLinearSolver solver = NonLinearSolver.NewtonRaphson, int numLoadSteps = 50, double tolerance = 1E-03, int maxIterations = 10000, int minIterations = 2)
			: base(spmInput, solver, numLoadSteps, tolerance, maxIterations, minIterations)
		{
		}

		/// <inheritdoc />
		protected override void ClearIterations()
		{
			base.ClearIterations();

			if (Solver is NonLinearSolver.Secant)
				return;
			
			foreach (var element in FemInput)
				switch (element)
				{
					case NLStringer stringer:
						stringer.ClearIterations();
						return;

					case NLPanel panel:
						panel.ClearIterations();
						return;

					default:
						return;
				}
		}
	}
}