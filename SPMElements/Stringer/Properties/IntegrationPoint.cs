using System;
using Extensions.Number;
using Material.Concrete;
using Material.Concrete.Uniaxial;
using Material.Reinforcement;
using Material.Reinforcement.Uniaxial;

namespace SPM.Elements.StringerProperties
{
	/// <summary>
	/// Stringer integration point class.
	/// </summary>
	public abstract class IntegrationPoint
	{
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
		public bool CrackedAndYielding => Cracked && Yielding;

		/// <summary>
		/// Get/set last integration point generalized strain.
		/// </summary>
		public (double e, double de) LastGenStrain { get; protected set; }

		/// <summary>
		/// Get maximum compressive force, in N.
		/// </summary>
		public double MaxCompressiveForce
		{
			get
			{
				var Nc = Concrete.MaxForce;

				if (Reinforcement is null)
					return Nc;

				double
					Nyr = Reinforcement.YieldForce,
					xi = Reinforcement.Stiffness / Concrete.Stiffness;

				double
					Nt1 = Nc * (1 + xi) * (1 + xi),
					Nt2 = Nc - Nyr;

				return
					Math.Max(Nt1, Nt2);
			}
		}

		/// <summary>
		/// Get cracking force, in N.
		/// </summary>
		protected double CrackingForce
		{
			get
			{
				double
					xi  = Reinforcement.Stiffness / Concrete.Stiffness,
					Ncr = Concrete.ft * Concrete.Area * (1 + xi);

				return
					Ncr / Math.Sqrt(1 + xi);
			}
		}

		/// <summary>
		/// Base integration point object.
		/// </summary>
		/// <param name="concrete">The <see cref="UniaxialConcrete"/> object.</param>
		/// <param name="reinforcement">The <see cref="UniaxialReinforcement"/> object.</param>
		protected IntegrationPoint(UniaxialConcrete concrete, UniaxialReinforcement reinforcement)
		{
			Concrete      = concrete;
			Reinforcement = reinforcement;
			Cracked       = false;
			Yielding      = false;
			LastGenStrain = (0, 0);
		}

		/// <summary>
		/// Calculate the strain and its derivative in the integration point.
		/// </summary>
		/// <param name="normalForce">Current force, in N.</param>
		public abstract (double e, double de) StringerStrain(double normalForce);

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
			if (!Yielding && strain.Abs() >= (Steel?.YieldStrain ?? 0))
				Yielding = true;
		}

		/// <summary>
		/// Get stress-strain relations.
		/// </summary>
		/// <param name="concrete">The <see cref="UniaxialConcrete"/> object.</param>
		/// <param name="reinforcement">The <see cref="UniaxialReinforcement"/> object.</param>
		public static IntegrationPoint Read(UniaxialConcrete concrete, UniaxialReinforcement reinforcement)
		{
			switch (concrete.Model)
			{
				case ConstitutiveModel.MCFT:
					return
						new MCFTIntegrationPoint(concrete, reinforcement);

				case ConstitutiveModel.DSFM:
					throw new NotImplementedException(); //Implement DSFM
			}

			return null;
		}

	}
}