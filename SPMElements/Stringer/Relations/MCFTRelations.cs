using System;
using Material.Concrete;
using Material.Reinforcement;
using MathNet.Numerics;

namespace SPMElements
{
	public partial class NonLinearStringer
	{
		/// <summary>
		/// MCFT class for stress-strain relations.
		/// </summary>
		private class MCFTRelations : StressStrainRelations
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
			public override (double e, double de) StringerStrain(double normalForce, IntegrationPoint intPoint)
			{
				double t1 = Concrete.Stiffness + (Reinforcement?.Stiffness ?? 0);

				(double e, double de) result = (0, 1 / t1);

				// Verify the value of N
				// Tensioned Stringer
				if (normalForce > 0)
				{
					// Calculate uncracked
					if (intPoint.Uncracked)
					{
						result = Uncracked(normalForce);

						// Verify if concrete is cracked
						intPoint.VerifyCracked(result.e);
					}

					if (intPoint.CrackedAndNotYielding)
					{
						// Calculate cracked
						var cracked = Cracked(normalForce);

						if (cracked.HasValue)
						{
							result = cracked.Value;

							// Verify if reinforcement yielded
							intPoint.VerifyYielding(result.e);
						}
						else
						{
							// Steel yielded
							intPoint.Yielding = true;
						}
					}

					if (intPoint.CrackedAndYielding)
					{
						// Steel is yielding
						result = YieldingSteel(normalForce);
					}
				}

				else if (normalForce < 0)
				{
					// Compressed Stringer
					if (normalForce > MaxCompressiveForce)
						result = ConcreteNotCrushed(normalForce);

					else
						result = ConcreteCrushing(normalForce);
				}

				return result;
			}

			// Tension Cases
			/// <summary>
			/// Tension case 1: uncracked.
			/// </summary>
			/// <param name="N">Normal force, in N.</param>
			private (double e, double de) Uncracked(double N)
			{
                double
                    t1 = Concrete.Stiffness + (Reinforcement?.Stiffness ?? 0),
                    e  = N / t1,
					de = 1 / t1;

				return
					(e, de);
			}

			/// <summary>
			/// Tension case 2: Cracked with not yielding steel.
			/// </summary>
			/// <param name="N">Normal force, in N.</param>
			private (double e, double de)? Cracked(double N) => Solver(N, Concrete.ecr, Steel.YieldStrain);

			/// <summary>
			/// Tension case 3: Cracked with yielding steel.
			/// </summary>
			/// <param name="N">Normal force, in N.</param>
			private (double e, double de) YieldingSteel(double N)
			{
				double
					ey  = Steel?.YieldStrain ?? 0,
					Nyr = Reinforcement?.YieldForce ?? 0,
					t1  = Concrete.Stiffness + (Reinforcement?.Stiffness ?? 0),
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
			private (double e, double de) ConcreteNotCrushed(double N)
			{
				// Calculate the strain for steel not yielding
				double
					ec = Concrete.ec,
					Nc = Concrete.MaxForce,
					xi = (Reinforcement?.Stiffness ?? 0) / Concrete.Stiffness,
					t2 = Math.Sqrt((1 + xi) * (1 + xi) - N / Nc),
					e = ec * (1 + xi - t2);

				// Check the strain
				if (Steel != null && e < -Steel.YieldStrain)
				{
					double Nyr = Reinforcement.YieldForce;

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
			private (double e, double de) ConcreteCrushing(double N)
			{
				// Calculate the strain for steel not yielding
				double
					ec = Concrete.ec,
					Nc = Concrete.MaxForce,
					xi = (Reinforcement?.Stiffness ?? 0) / Concrete.Stiffness,
					t1 = Concrete.Stiffness + (Reinforcement?.Stiffness ?? 0),
					t2 = Math.Sqrt((1 + xi) * (1 + xi) - MaxCompressiveForce / Nc),
					e = ec * (1 + xi - t2) + (N - MaxCompressiveForce) / t1;

				// Check the strain
				if (Steel != null && e < -Steel.YieldStrain)
				{
					double Nyr = Reinforcement.YieldForce;

                    // Recalculate the strain for steel yielding
                    e = ec * (1 - Math.Sqrt(1 - (Nyr + MaxCompressiveForce) / Nc)) + (N - MaxCompressiveForce) / t1;
				}

				// Calculate de
				double de = 1 / t1;

				return
					(e, de);
			}

			// Compressed case
			private (double e, double de)? Compressed(double N) => Solver(N, Concrete.ecu, 0);

			/// <summary>
			/// Solver to find strain given force.
			/// </summary>
			private (double e, double de)? Solver(double N, double lowerBound, double upperBound)
			{
				// Iterate to find strain
				(double e, double de)? result = null;
				double? e = null;

				try
				{
					e = FindRoots.OfFunction(eps => N - Force(eps), lowerBound, upperBound);
				}
				catch
				{
				}
				finally
				{
					if (e.HasValue)
					{
						// Calculate derivative of function
						double
							dN = Differentiate.FirstDerivative(Force, e.Value),
							de = 1 / dN;

						result = (e.Value, de);
					}
				}

				return result;
			}

			/// <summary>
			/// Calculate force based on strain.
			/// </summary>
			/// <param name="strain">Current strain.</param>
			private double Force(double strain) => Concrete.CalculateForce(strain) + (Reinforcement?.CalculateForce(strain) ?? 0);
		}
	}
}