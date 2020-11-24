using System;
using Extensions.Number;
using Material.Concrete.Uniaxial;
using Material.Reinforcement.Uniaxial;
using MathNet.Numerics;
using MathNet.Numerics.RootFinding;

namespace SPM.Elements
{
	public partial class NonLinearStringer
	{
		/// <summary>
		/// MCFT class for stress-strain relations.
		/// </summary>
		private class MCFTRelations : NLRelations
		{
            /// <summary>
            /// MCFT object for stress-strain relations.
            /// </summary>
            /// <param name="concrete">The <see cref="UniaxialConcrete"/> object.</param>
            /// <param name="reinforcement">The <see cref="UniaxialReinforcement"/> object.</param>
			public MCFTRelations(UniaxialConcrete concrete, UniaxialReinforcement reinforcement) : base(concrete, reinforcement)
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
			protected override (double e, double de) CompressedCase(double normalForce, IntegrationPoint intPoint) => normalForce > MaxCompressiveForce ? ConcreteNotCrushedState(normalForce) : ConcreteCrushingState(normalForce);

			// Tension Cases
			/// <summary>
			/// Tension case 1: uncracked.
			/// </summary>
			/// <param name="N">Normal force, in N.</param>
			private (double e, double de) UncrackedState(double N)
			{
                double
                    t1 = Stiffness,
                    e  = N / t1,
					de = 1 / t1;

				return
					(e, de);
			}

			/// <summary>
			/// Tension case 2: Cracked with not yielding steel.
			/// </summary>
			/// <param name="N">Normal force, in N.</param>
			private (double e, double de)? CrackedState(double N) => Solver(N, Concrete.ecr, Steel?.YieldStrain ?? Concrete.ecr);

			/// <summary>
			/// Tension case 3: Cracked with yielding steel.
			/// </summary>
			/// <param name="N">Normal force, in N.</param>
			private (double e, double de) YieldingSteelState(double N)
			{
				double
					ey  = Steel?.YieldStrain ?? 0,
					Nyr = Reinforcement?.YieldForce ?? 0,
					t1  = Stiffness,
                    e   = ey + (N - Nyr) / t1,
					de  = 1 / t1;

				return
					(e, de);
			}

			// Compression Cases
			/// <summary>
			/// Compression case 1: concrete not crushed.
			/// </summary>
			/// <param name="N">Normal force, in N.</param>
			private (double e, double de) ConcreteNotCrushedState(double N)
			{
				// Calculate the strain for steel not yielding
				double
					ec = Concrete.ec,
					Nc = Concrete.MaxForce,
					xi = StiffnessRatio,
					t2 = Math.Sqrt((1 + xi) * (1 + xi) - N / Nc),
					e = ec * (1 + xi - t2);

				// Check the strain
				if (!(Reinforcement is null) && e < -Steel.YieldStrain)
				{
					var Nyr = Reinforcement.YieldForce;

					// Recalculate the strain for steel yielding
                    t2 = Math.Sqrt(1 - (N + Nyr) / Nc);
					e = ec * (1 - t2);
				}

				// Calculate de
				double de = 1 / (Concrete.Stiffness * t2);

				return
					(e, de);
			}

			/// <summary>
			/// Compression case 2: concrete crushed.
			/// </summary>
			/// <param name="N">Normal force, in N.</param>
			private (double e, double de) ConcreteCrushingState(double N)
			{
				// Calculate the strain for steel not yielding
				double
					ec = Concrete.ec,
					Nc = Concrete.MaxForce,
					Nt = MaxCompressiveForce,
					xi = StiffnessRatio,
					t1 = Stiffness,
					t2 = Math.Sqrt((1 + xi) * (1 + xi) - Nt / Nc),
					e = ec * (1 + xi - t2) + (N - Nt) / t1;

				// Check the strain
				if (!(Reinforcement is null) && e < -Steel.YieldStrain)
				{
					var Nyr = Reinforcement.YieldForce;

                    // Recalculate the strain for steel yielding
                    e = ec * (1 - Math.Sqrt(1 - (Nyr + Nt) / Nc)) + (N - Nt) / t1;
				}

				// Calculate de
				double de = 1 / t1;

				return
					(e, de);
			}

			// Compressed case
			private (double e, double de)? Compressed(double N) => Solver(N, Concrete.ecu, 0);
		}
	}
}