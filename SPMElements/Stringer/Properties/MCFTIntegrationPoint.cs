﻿using System;
using Extensions.Number;
using Material.Concrete;
using Material.Concrete.Uniaxial;
using Material.Reinforcement;
using Material.Reinforcement.Uniaxial;
using MathNet.Numerics;
using MathNet.Numerics.RootFinding;

namespace SPM.Elements.StringerProperties
{
	/// <summary>
	/// Struct to verify cracked or yielded state on each integration point
	/// </summary>
	public class MCFTIntegrationPoint : IntegrationPoint
	{
		/// <summary>
		/// Base object for integration point.
		/// </summary>
		/// <param name="concrete">The <see cref="UniaxialConcrete"/> object.</param>
		/// <param name="reinforcement">The <see cref="UniaxialReinforcement"/> object.</param>
		public MCFTIntegrationPoint(UniaxialConcrete concrete, UniaxialReinforcement reinforcement)
			: base(concrete, reinforcement)
		{
		}

		/// <inheritdoc/>
		protected override (double e, double de) Tensioned(double normalForce)
		{
			(double e, double de) result = (0, 1 / Stiffness);

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

			return result;
		}

		/// <inheritdoc/>
		protected override (double e, double de) Compressed(double normalForce)
		{
			// Verify if concrete is crushed
			VerifyCrushed(normalForce);

			return !Crushed
				? ConcreteNotCrushedState(normalForce)
				: ConcreteCrushingState(normalForce);
		}

		// Tension Cases
		/// <summary>
		/// Tension case 1: uncracked.
		/// </summary>
		/// <param name="N">Normal force, in N.</param>
		private (double e, double de) UncrackedState(double N) => (N / Stiffness, 1 / Stiffness);

		/// <summary>
		/// Tension case 2: Cracked with not yielding steel.
		/// </summary>
		/// <param name="N">Normal force, in N.</param>
		private (double e, double de)? CrackedState(double N) => Solver(N, Concrete.ecr, Steel?.YieldStrain ?? 3E-3);

		/// <summary>
		/// Tension case 3: Cracked with yielding steel.
		/// </summary>
		/// <param name="N">Normal force, in N.</param>
		private (double e, double de) YieldingSteelState(double N)
		{
			double
				ey = Steel?.YieldStrain ?? 0,
				Nyr = Reinforcement?.YieldForce ?? 0,
				e = ey + (N - Nyr) / Stiffness,
				de = 1 / Stiffness;

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
			// Calculate e and t2
			var e = Calculate(out var t2);

			// Calculate de
			var de = 1 / (Concrete.Stiffness * t2);

			return
				(e, de);

			double Calculate(out double constantT2)
			{
				double
					ec = Concrete.ec,
					Nc = Concrete.MaxForce;

				// Calculate the strain for steel not yielding
				if (!Yielding || Reinforcement is null)
				{
					constantT2 = Math.Sqrt((1 + Stiffness) * (1 + Stiffness) - N / Nc);
					var strain = ec * (1 + Stiffness - constantT2);

					if (!VerifyYielding(strain))
						return strain;
				}

				var Nyr = Reinforcement.YieldForce;

				// Recalculate the strain for steel yielding
				constantT2 = Math.Sqrt(1 - (N + Nyr) / Nc);

				return 
					ec * (1 - constantT2);
			}
		}

		/// <summary>
		/// Compression case 2: concrete crushed.
		/// </summary>
		/// <param name="N">Normal force, in N.</param>
		private (double e, double de) ConcreteCrushingState(double N)
		{
			// Calculate strain
			var e = Calculate();

			// Calculate de
			var de = 1 / Stiffness;

			return
				(e, de);

			double Calculate()
			{
				double
					ec = Concrete.ec,
					Nc = Concrete.MaxForce;

				// Calculate the strain for steel not yielding
				if (!Yielding || Reinforcement is null)
				{
					var t2     = Math.Sqrt((1 + StiffnessRatio) * (1 + StiffnessRatio) - MaxCompressiveForce / Nc);
					var strain = ec * (1 + StiffnessRatio - t2) + (N - MaxCompressiveForce) / Stiffness;

					if (!VerifyYielding(strain))
						return strain;
				}

				var Nyr = Reinforcement.YieldForce;

				// Recalculate the strain for steel yielding
				return
					ec * (1 - Math.Sqrt(1 - (Nyr + MaxCompressiveForce) / Nc)) + (N - MaxCompressiveForce) / Stiffness;
			}
		}

		/// <summary>
		/// Solver to find strain given force.
		/// </summary>
		private (double e, double de)? Solver(double N, double lowerBound, double upperBound)
		{
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