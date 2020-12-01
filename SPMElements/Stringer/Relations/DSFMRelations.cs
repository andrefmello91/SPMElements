using System;
using Autodesk.AutoCAD.DatabaseServices;
using Extensions.Number;
using Material.Concrete.Uniaxial;
using Material.Reinforcement.Uniaxial;
using MathNet.Numerics;
using MathNet.Numerics.RootFinding;

namespace SPM.Elements
{
	public partial class NLStringer
	{
		/// <summary>
		/// MCFT class for stress-strain relations.
		/// </summary>
		private class DSFMRelations : NLRelations
		{
            /// <summary>
            /// MCFT object for stress-strain relations.
            /// </summary>
            /// <param name="concrete">The <see cref="UniaxialConcrete"/> object.</param>
            /// <param name="reinforcement">The <see cref="UniaxialReinforcement"/> object.</param>
			public DSFMRelations(UniaxialConcrete concrete, UniaxialReinforcement reinforcement) : base(concrete, reinforcement)
			{
			}

			/// <inheritdoc/>
			protected override (double e, double de) TensionedCase(double normalForce, IntegrationPoint intPoint)
			{
				// Calculate uncracked
				if (intPoint.Uncracked)
				{
					var unc = UncrackedState(normalForce);

					// Verify if concrete is cracked
					if (!intPoint.VerifyCracked(unc.e))
						return unc;
				}

				if (intPoint.CrackedAndNotYielding)
				{
					// Calculate cracked
					var crackedN = CrackedState(normalForce);

					if (crackedN.HasValue)
					{
						var cracked = crackedN.Value;

						// Verify if reinforcement yielded
						if (!intPoint.VerifyYielding(cracked.e))
							return cracked;
					}
					else
					{
						// Steel yielded
						intPoint.Yielding = true;
					}
				}

				// Steel is yielding
				return YieldingSteelState(normalForce);
			}

			/// <inheritdoc/>
			protected override (double e, double de) CompressedCase(double normalForce, IntegrationPoint intPoint) => Compressed(normalForce) ?? intPoint.LastGenStrain;

			// Compressed case
			private (double e, double de)? Compressed(double N) => Solver(N, Concrete.ecu, 0);
		}
	}
}