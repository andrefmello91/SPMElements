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
		/// Get maximum compressive force, in N.
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
					_xi = Reinforcement.Stiffness / Concrete.Stiffness;

				return _xi.Value;
			}
		}

		/// <summary>
		/// Get/set cracked state.
		/// </summary>
		public bool Cracked  { get; protected set; }

		/// <summary>
		/// Get/set yielding state.
		/// </summary>
		public bool Yielding { get; protected set; }
			
		/// <summary>
		/// Returns true if state if state is uncracked.
		/// </summary>
		public bool Uncracked => !Cracked && !Yielding;

		/// <summary>
		/// Returns  true if concrete is cracked and steel is not yielding.
		/// </summary>
		public bool CrackedAndNotYielding => Cracked && !Yielding;

		/// <summary>
		/// Returns true if concrete is cracked and steel is yielding.
		/// </summary>
		public bool CrackedAndYielding    =>  Cracked &&  Yielding;

		/// <summary>
		/// Get/set last integration point generalized strain.
		/// </summary>
		public (double e, double de) LastGenStrain { get; set; }

		/// <summary>
		/// Base object for integration point.
		/// </summary>
		/// <param name="concrete">The <see cref="UniaxialConcrete"/> object.</param>
		/// <param name="reinforcement">The <see cref="UniaxialReinforcement"/> object.</param>
		protected IntegrationPoint(UniaxialConcrete concrete, UniaxialReinforcement reinforcement)
		{
			Concrete      = concrete.Copy();
			Reinforcement = reinforcement.Copy();
			Cracked       = false;
			Yielding      = false;
			LastGenStrain = (0, 1 / Stiffness);
		}

		/// <summary>
		/// Calculate the strain and its derivative in the integration point.
		/// </summary>
		/// <param name="normalForce">Current force, in N.</param>
		public (double e, double de) StringerStrain(double normalForce)
		{
			if (normalForce.IsNaN())
				return LastGenStrain;

			(double e, double de) result;

			// Verify the value of N
			if (normalForce.ApproxZero(1E-6))
				result = (0, 1 / Stiffness);

			// Tensioned Stringer
			else if (normalForce > 0)
				result = Tensioned(normalForce);

			else
				result = Compressed(normalForce);

			if (result.e.IsNaN())
				return LastGenStrain;

			LastGenStrain = result;

			return result;
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
		/// Verify if stringer is cracked.
		/// </summary>
		/// <param name="strain">Current strain</param>
		public void VerifyCracked(double strain)
		{
			if (!Cracked && strain >= Concrete.ecr)
				Cracked = true;
		}

		/// <summary>
		/// Verify if steel is yielding.
		/// </summary>
		/// <param name="strain">Current strain</param>
		public void VerifyYielding(double strain)
		{
			if (!(Steel is null) && !Yielding && strain.Abs() >= Steel.YieldStrain)
				Yielding = true;
		}
	}
}