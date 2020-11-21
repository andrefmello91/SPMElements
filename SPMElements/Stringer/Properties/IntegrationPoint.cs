using System;
using Extensions.Number;
using Material.Concrete;
using Material.Concrete.Uniaxial;
using Material.Reinforcement;
using Material.Reinforcement.Uniaxial;

namespace SPM.Elements.StringerProperties
{
	/// <summary>
	/// Stringer integration point base class.
	/// </summary>
	public abstract class IntegrationPoint
	{
		// Auxiliary fields
		private double? _Nt, _Ncr, _xi, _t1;

		/// <summary>
		/// Last calculated strain and derivative.
		/// </summary>
		protected (double e, double de) LastStrain;

		/// <summary>
		/// Current calculated strain and derivative.
		/// </summary>
		protected (double e, double de) CurrentStrain;

		/// <summary>
		/// Get <see cref="UniaxialConcrete"/> object.
		/// </summary>
		protected UniaxialConcrete Concrete { get; }

		/// <summary>
		/// Get <see cref="UniaxialReinforcement"/> object.
		/// </summary>
		protected UniaxialReinforcement Reinforcement { get; }

		/// <summary>
		/// Get <see cref="Steel"/> object of <see cref="Reinforcement"/>.
		/// </summary>
		protected Steel Steel => Reinforcement?.Steel;

		/// <summary>
		/// Get maximum compressive force, in N (negative value).
		/// </summary>
		public double MaxCompressiveForce
		{
			get
			{
				if (!_Nt.HasValue)
					_Nt = Reinforcement is null
						? Concrete.MaxForce
						: Math.Max(Concrete.MaxForce * (1 + StiffnessRatio) * (1 + StiffnessRatio), Concrete.MaxForce - Reinforcement.YieldForce);

				return _Nt.Value;
			}
		}

		/// <summary>
		/// Get cracking force, in N.
		/// </summary>
		protected double CrackingForce
		{
			get
			{
				if (!_Ncr.HasValue)
					_Ncr = Concrete.ft * Concrete.Area * (1 + StiffnessRatio) / (1 + StiffnessRatio).Sqrt();

				return _Ncr.Value;
			}
		}

		/// <summary>
		/// Get stiffness.
		/// </summary>
		protected double Stiffness
		{
			get
			{
				if (!_t1.HasValue)
					_t1 = Concrete.Stiffness + (Reinforcement?.Stiffness ?? 0);

				return _t1.Value;
			}
		}

		/// <summary>
		/// Get stiffness ratio.
		/// </summary>
		protected double StiffnessRatio
		{
			get
			{
				if (!_xi.HasValue)
					_xi = Reinforcement is null ? 0 : Reinforcement.Stiffness / Concrete.Stiffness;

				return _xi.Value;
			}
		}

		/// <summary>
		/// Get/set cracked state.
		/// </summary>
		protected bool Cracked  { get; set; }

		/// <summary>
		/// Get/set yielding state.
		/// </summary>
		protected bool Yielding { get; set; }

		/// <summary>
		/// Get/set crushed state.
		/// </summary>
		protected bool Crushed { get; set; }

		/// <summary>
		/// Returns true if state if state is uncracked.
		/// </summary>
		protected bool Uncracked => !Cracked && !Yielding;

		/// <summary>
		/// Returns  true if concrete is cracked and steel is not yielding.
		/// </summary>
		protected bool CrackedAndNotYielding => Cracked && !Yielding;

		/// <summary>
		/// Returns true if concrete is cracked and steel is yielding.
		/// </summary>
		protected bool CrackedAndYielding => Cracked &&  Yielding;

		/// <summary>
		/// Returns  true if concrete is not crushed and steel is not yielding.
		/// </summary>
		protected bool UncrushedAndNotYielding => !Crushed && !Yielding;

		/// <summary>
		/// Returns true if concrete is not crushed and steel is yielding.
		/// </summary>
		protected bool UncrushedAndYielding => !Crushed && Yielding;

		/// <summary>
		/// Returns  true if concrete is crushed and steel is not yielding.
		/// </summary>
		protected bool CrushedAndNotYielding => Crushed && !Yielding;

		/// <summary>
		/// Returns true if concrete is crushed and steel is yielding.
		/// </summary>
		protected bool CrushedAndYielding => Crushed && Yielding;

		/// <summary>
		/// Base object for integration point.
		/// </summary>
		/// <param name="concrete">The <see cref="UniaxialConcrete"/> object.</param>
		/// <param name="reinforcement">The <see cref="UniaxialReinforcement"/> object.</param>
		protected IntegrationPoint(UniaxialConcrete concrete, UniaxialReinforcement reinforcement)
		{
			Concrete      = concrete;
			Reinforcement = reinforcement;
			Cracked       = false;
			Yielding      = false;
			Crushed       = false;
			LastStrain    = (0, 1 / Stiffness);
		}

		/// <summary>
		/// Calculate the strain and its derivative in this <see cref="IntegrationPoint"/>.
		/// </summary>
		/// <param name="normalForce">Current force, in N.</param>
		public (double e, double de) CalculateStrain(double normalForce)
		{
			if (normalForce.IsNaN())
				return LastStrain;

			// Verify the value of N
			if (normalForce.ApproxZero(1E-6))
				CurrentStrain = (0, 1 / Stiffness);

			// Tensioned Stringer
			else if (normalForce > 0)
				CurrentStrain = Tensioned(normalForce);

			else
				CurrentStrain = Compressed(normalForce);

			if (CurrentStrain.e.IsNaN())
				CurrentStrain = LastStrain;

			else
				LastStrain = CurrentStrain;

			return CurrentStrain;
		}

		/// <summary>
		/// Calculate strain and derivative for tensioned state.
		/// </summary>
		/// <param name="normalForce">Current force, in N.</param>
		protected abstract (double e, double de) Tensioned(double normalForce);

		/// <summary>
		/// Calculate strain and derivative for compressed state.
		/// </summary>
		/// <param name="normalForce">Current force, in N.</param>
		protected abstract (double e, double de) Compressed(double normalForce);

		/// <summary>
		/// Verify if stringer is cracked. Returns true if it is cracked.
		/// </summary>
		/// <param name="strain">Current strain</param>
		protected bool VerifyCracked(double strain)
		{
			if (!Cracked && strain >= Concrete.ecr)
				Cracked = true;

			return Cracked;
		}

		/// <summary>
		/// Verify if steel is yielding. Returns true if it is yielding.
		/// </summary>
		/// <param name="strain">Current strain</param>
		protected bool VerifyYielding(double strain)
		{
			if (!(Steel is null) && !Yielding && strain.Abs() >= Steel.YieldStrain)
				Yielding = true;

			return Yielding;
		}
		/// <summary>
		/// Verify if stringer is crushed. Returns true if it is crushed.
		/// </summary>
		/// <param name="force">Current normal force</param>
		protected bool VerifyCrushed(double force)
		{
			if (!Crushed && force <= MaxCompressiveForce)
				Crushed = true;

			return Crushed;
		}

		/// <summary>
		/// Get an <see cref="IntegrationPoint"/> based on <paramref name="concrete"/>'s <see cref="ConstitutiveModel"/>.
		/// </summary>
		/// <param name="concrete">The <see cref="UniaxialConcrete"/> object.</param>
		/// <param name="reinforcement">The <see cref="UniaxialReinforcement"/> object.</param>
		public static IntegrationPoint Read(UniaxialConcrete concrete, UniaxialReinforcement reinforcement)
		{
			switch (concrete.Model)
			{
				case ConstitutiveModel.MCFT:
					return new MCFTIntegrationPoint(concrete, reinforcement);
				
				// Implement DSFM
				default:
					return null;
			}
		}
	}
}