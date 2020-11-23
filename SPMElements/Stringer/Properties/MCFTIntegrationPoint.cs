using System;
using Extensions.Number;
using Material.Concrete.Uniaxial;
using Material.Reinforcement.Uniaxial;
using MathNet.Numerics;
using MathNet.Numerics.RootFinding;

namespace SPM.Elements.StringerProperties
{
	/// <summary>
	/// Stringer MCFT integration point class.
	/// </summary>
	public class MCFTIntegrationPoint : IntegrationPoint
	{
		/// <summary>
		/// MCFT integration point object.
		/// </summary>
		/// <param name="concrete">The <see cref="UniaxialConcrete"/> object.</param>
		/// <param name="reinforcement">The <see cref="UniaxialReinforcement"/> object.</param>
		public MCFTIntegrationPoint(UniaxialConcrete concrete, UniaxialReinforcement reinforcement)
			: base(concrete, reinforcement)
		{
		}

		/// <inheritdoc/>
		public override (double e, double de) StringerStrain(double normalForce)
		{
			if (normalForce.IsNaN())
				return LastGenStrain;

			double t1 = Concrete.Stiffness + (Reinforcement?.Stiffness ?? 0);

			(double e, double de) result = (0, 1 / t1);

			// Verify the value of N
			// Tensioned Stringer
			if (normalForce > 0)
			{
				// Calculate uncracked
				if (Uncracked)
				{
					result = UncrackedState(normalForce);

					// Verify if concrete is cracked
					VerifyCracked(result.e);
				}

				if (CrackedAndNotYielding)
				{
					// Calculate cracked
					var cracked = CrackedState(normalForce);

					if (cracked.HasValue)
					{
						result = cracked.Value;

						// Verify if reinforcement yielded
						VerifyYielding(result.e);
					}
					else
					{
						// Steel yielded
						Yielding = true;
					}
				}

				if (CrackedAndYielding)
				{
					// Steel is yielding
					result = YieldingSteelState(normalForce);
				}
			}

			else if (normalForce < 0)
			{
				// Compressed Stringer
				result = normalForce > MaxCompressiveForce ? ConcreteNotCrushedState(normalForce) : ConcreteCrushedState(normalForce);
			}

			if (result.e.IsNaN())
				return LastGenStrain;

			LastGenStrain = result;

			return result;
		}

		// Tension Cases
		/// <summary>
		/// Tension case 1: uncracked.
		/// </summary>
		/// <param name="N">Normal force, in N.</param>
		private (double e, double de) UncrackedState(double N)
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
		private (double e, double de)? CrackedState(double N) => Solver(N, Concrete.ecr, Steel?.YieldStrain ?? Concrete.ecr);

		/// <summary>
		/// Tension case 3: Cracked with yielding steel.
		/// </summary>
		/// <param name="N">Normal force, in N.</param>
		private (double e, double de) YieldingSteelState(double N)
		{
			double
				ey = Steel?.YieldStrain ?? 0,
				Nyr = Reinforcement?.YieldForce ?? 0,
				t1 = Concrete.Stiffness + (Reinforcement?.Stiffness ?? 0),
				e = ey + (N - Nyr) / t1,
				de = 1 / t1;

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
		private (double e, double de) ConcreteCrushedState(double N)
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
			//(double e, double de)? result = null;
			//double? e = null;

			if (!Brent.TryFindRoot(eps => N - Force(eps), lowerBound, upperBound, 1E-4, 1000, out var e))
				return null;

			// Calculate derivative of function
			double
				dN = Differentiate.FirstDerivative(Force, e),
				de = 1 / dN;

			return (e, de);
		}

		/// <summary>
		/// Calculate force based on strain.
		/// </summary>
		/// <param name="strain">Current strain.</param>
		private double Force(double strain) => Concrete.CalculateForce(strain) + (Reinforcement?.CalculateForce(strain) ?? 0);
	}
}