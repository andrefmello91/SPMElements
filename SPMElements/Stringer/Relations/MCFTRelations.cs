//using System;
//using Extensions.Number;
//using Material.Concrete;
//using Material.Concrete.Uniaxial;
//using Material.Reinforcement;
//using Material.Reinforcement.Uniaxial;
//using MathNet.Numerics;
//using MathNet.Numerics.RootFinding;

//namespace SPM.Elements
//{
//	public partial class NonLinearStringer
//	{
//		/// <summary>
//		/// MCFT class for stress-strain relations.
//		/// </summary>
//		private class MCFTRelations : StressStrainRelations
//		{
//            /// <summary>
//            /// MCFT object for stress-strain relations.
//            /// </summary>
//            /// <param name="concrete">The <see cref="UniaxialConcrete"/> object.</param>
//            /// <param name="reinforcement">The <see cref="UniaxialReinforcement"/> object.</param>
//			public MCFTRelations(UniaxialConcrete concrete, UniaxialReinforcement reinforcement) : base(concrete, reinforcement)
//			{
//			}

//			/// <inheritdoc/>
//			public override (double e, double de) StringerStrain(double normalForce, IntegrationPoint intPoint)
//			{
//				if (normalForce.IsNaN())
//					return intPoint.LastGenStrain;

//				(double e, double de) result;

//				// Verify the value of N
//				if (normalForce.ApproxZero(1E-6))
//					result = (0, 1 / _t1.Value);

//				// Tensioned Stringer
//				else if (normalForce > 0)
//				{
//					// Calculate uncracked
//					if (intPoint.Uncracked)
//					{
//						result = Uncracked(normalForce);

//						// Verify if concrete is cracked
//						intPoint.VerifyCracked(result.e);
//					}

//					if (intPoint.CrackedAndNotYielding)
//					{
//						// Calculate cracked
//						var cracked = Cracked(normalForce);

//						if (cracked.HasValue)
//						{
//							result = cracked.Value;

//							// Verify if reinforcement yielded
//							intPoint.VerifyYielding(result.e);
//						}
//						else
//						{
//							// Steel yielded
//							intPoint.Yielding = true;
//						}
//					}

//					if (intPoint.CrackedAndYielding)
//					{
//						// Steel is yielding
//						result = YieldingSteel(normalForce);
//					}
//				}

//				else if (normalForce < 0)
//				{
//					// Compressed Stringer
//					result = normalForce > MaxCompressiveForce ? ConcreteNotCrushed(normalForce) : ConcreteCrushing(normalForce);
//				}

//				if (result.e.IsNaN())
//					return intPoint.LastGenStrain;

//				intPoint.LastGenStrain = result;

//                return result;
//			}

//			// Tension Cases
//			/// <summary>
//			/// Tension case 1: uncracked.
//			/// </summary>
//			/// <param name="N">Normal force, in N.</param>
//			private (double e, double de) Uncracked(double N) => (N / Stiffness, 1 / Stiffness);

//			/// <summary>
//			/// Tension case 2: Cracked with not yielding steel.
//			/// </summary>
//			/// <param name="N">Normal force, in N.</param>
//			private (double e, double de)? Cracked(double N) => Solver(N, Concrete.ecr, Steel?.YieldStrain ?? Concrete.ecr);

//			/// <summary>
//			/// Tension case 3: Cracked with yielding steel.
//			/// </summary>
//			/// <param name="N">Normal force, in N.</param>
//			private (double e, double de) YieldingSteel(double N)
//			{
//				double
//					ey  = Steel?.YieldStrain ?? 0,
//					Nyr = Reinforcement?.YieldForce ?? 0,
//                    e   = ey + (N - Nyr) / Stiffness,
//					de  = 1 / Stiffness;

//				return
//					(e, de);
//			}

//			// Compression Cases
//			/// <summary>
//			/// Compression case 1: concrete not crushed.
//			/// </summary>
//			/// <param name="N">Normal force, in N.</param>
//			private (double e, double de) ConcreteNotCrushed(double N)
//			{
//				// Calculate the strain for steel not yielding
//				double
//					ec = Concrete.ec,
//					Nc = Concrete.MaxForce,
//					t2 = Math.Sqrt((1 + Stiffness) * (1 + Stiffness) - N / Nc),
//					e  = ec * (1 + Stiffness - t2);

//				// Check the strain
//				if (Steel != null && e < -Steel.YieldStrain)
//				{
//					double Nyr = Reinforcement.YieldForce;

//					// Recalculate the strain for steel yielding
//                    t2 = Math.Sqrt(1 - (N + Nyr) / Nc);
//					e = ec * (1 - t2);
//				}

//				// Calculate de
//				double de = 1 / (Concrete.Stiffness * t2);

//				return
//					(e, de);
//			}

//			/// <summary>
//			/// Compression case 2: concrete crushed.
//			/// </summary>
//			/// <param name="N">Normal force, in N.</param>
//			private (double e, double de) ConcreteCrushing(double N)
//			{
//				// Calculate the strain for steel not yielding
//				double
//					ec = Concrete.ec,
//					Nc = Concrete.MaxForce,
//					xi = (Reinforcement?.Stiffness ?? 0) / Concrete.Stiffness,
//					t1 = Concrete.Stiffness + (Reinforcement?.Stiffness ?? 0),
//					t2 = Math.Sqrt((1 + xi) * (1 + xi) - MaxCompressiveForce / Nc),
//					e = ec * (1 + xi - t2) + (N - MaxCompressiveForce) / t1;

//				// Check the strain
//				if (Steel != null && e < -Steel.YieldStrain)
//				{
//					double Nyr = Reinforcement.YieldForce;

//                    // Recalculate the strain for steel yielding
//                    e = ec * (1 - Math.Sqrt(1 - (Nyr + MaxCompressiveForce) / Nc)) + (N - MaxCompressiveForce) / t1;
//				}

//				// Calculate de
//				double de = 1 / t1;

//				return
//					(e, de);
//			}

//			// Compressed case
//			private (double e, double de)? Compressed(double N) => Solver(N, Concrete.ecu, 0);

//			/// <summary>
//			/// Solver to find strain given force.
//			/// </summary>
//			private (double e, double de)? Solver(double N, double lowerBound, double upperBound)
//			{
//				// Iterate to find strain
//				//(double e, double de)? result = null;
//				//double? e = null;

//				if (!Brent.TryFindRoot(eps => N - Force(eps), lowerBound, upperBound, 1E-4, 1000, out var e))
//					return null;

//				// Calculate derivative of function
//				double
//					dN = Differentiate.FirstDerivative(Force, e),
//					de = 1 / dN;

//				return (e, de);

//				//            try
//				//{
//				//	e = FindRoots.OfFunction(eps => N - Force(eps), lowerBound, upperBound);
//				//}
//				//catch
//				//{
//				//}
//				//finally
//				//{
//				//	if (e.HasValue)
//				//	{
//				//		// Calculate derivative of function
//				//		double
//				//			dN = Differentiate.FirstDerivative(Force, e.Value),
//				//			de = 1 / dN;

//				//		result = (e.Value, de);
//				//	}
//				//}

//				//return result;
//			}

//			/// <summary>
//			/// Calculate force based on strain.
//			/// </summary>
//			/// <param name="strain">Current strain.</param>
//			private double Force(double strain) => Concrete.CalculateForce(strain) + (Reinforcement?.CalculateForce(strain) ?? 0);
//		}
//	}
//}