using System;
using Material.Concrete;
using Material.Concrete.Uniaxial;
using Material.Reinforcement;
using Material.Reinforcement.Uniaxial;

namespace SPM.Elements
{
	public partial class NonLinearStringer
	{
		/// <summary>
		/// Base class for stress-strain relations.
		/// </summary>
		private abstract class StressStrainRelations
		{
			/// <summary>
            /// Get <see cref="Material.Concrete.Uniaxial.UniaxialConcrete"/> object.
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
            /// Base object for stress-strain relations.
            /// </summary>
            /// <param name="concrete">The <see cref="UniaxialConcrete"/> object.</param>
            /// <param name="reinforcement">The <see cref="UniaxialReinforcement"/> object.</param>
            public StressStrainRelations(UniaxialConcrete concrete, UniaxialReinforcement reinforcement)
			{
				Concrete      = concrete;
				Reinforcement = reinforcement;
			}

			/// <summary>
            /// Get maximum compressive force, in N.
            /// </summary>
			public double MaxCompressiveForce
			{
				get
				{
					var Nc = Concrete.MaxForce;

					if (Steel is null)
						return Nc;

					double
						Nyr = Reinforcement.YieldForce,
						xi = (Reinforcement?.Stiffness ?? 0) / Concrete.Stiffness;

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
						xi  = Concrete.Stiffness / Reinforcement.Stiffness,
						Ncr = Concrete.ft * Concrete.Area * (1 + xi);

                    return
	                    Ncr / Math.Sqrt(1 + xi);
				}
			}

			/// <summary>
			/// Calculate the strain and its derivative in the integration point.
			/// </summary>
			/// <param name="normalForce">Current force, in N.</param>
			/// <param name="intPoint">Integration point (<see cref="IntegrationPoint"/>).</param>
			/// <returns></returns>
			public abstract (double e, double de) StringerStrain(double normalForce, IntegrationPoint intPoint);

            /// <summary>
            /// Get stress-strain relations.
            /// </summary>
            /// <param name="concrete">The <see cref="UniaxialConcrete"/> object.</param>
            /// <param name="reinforcement">The <see cref="UniaxialReinforcement"/> object.</param>
            public static StressStrainRelations GetRelations(UniaxialConcrete concrete, UniaxialReinforcement reinforcement)
			{
				switch (concrete.Model)
				{
					case ConstitutiveModel.MCFT:
						return
							new MCFTRelations(concrete, reinforcement);

					case ConstitutiveModel.DSFM:
						throw new NotImplementedException(); //Implement DSFM
				}

				return null;
			}
		}
	}
}

// MC2010 model for concrete
//        public class MC2010 : NonLinear
//        {
//         public MC2010(ObjectId stringerObject, ConcreteParameters concreteParameters) : base(stringerObject, concreteParameters)
//         {
//         }

//            // Get concrete parameters
//            //public override double fc     => Concrete.fc;
//            //public override double ec     => Concrete.ec;
//            //public override double ecu    => Concrete.ecu;
//            //public override double Ec     => Concrete.Ec;
//            //public override double fcr    => Concrete.fcr;
//            //public override double ecr    => Concrete.ecr;
//            public double          k      => Concrete.Ec / Concrete.Ecs;
//            private double         beta   =  0.6;
//            private double         sigSr  => fcr * (1 + xi) / ps;

//            // Calculate the yield force on compression
//            public override double Nyc => -Nyr + Nc * (-k * ey_ec - ey_ec * ey_ec) / (1 - (k - 2) * ey_ec);

//            // Calculate limit force on compression
//            private double NlimC => EsAs * ec + Nc;
//private double NlimS => -Nyr + Nc;
//public override double Nt
//{
//	get
//	{
//		if (-ey < ec)
//			return
//				NlimC;

//		return
//			NlimS;
//	}
//            }

//            // Calculate the strain and derivative on a Stringer given a force N and the concrete parameters
//            public override (double e, double de) StringerStrain(double N)
//            {
//             // Verify the value of N
//             if (N >= 0) // tensioned Stringer
//             {
//              if (N < Ncr)
//               return
//                Uncracked(N);

//              if (N <= Nyr)
//               return
//                Cracked(N);

//              return
//               YieldingSteel(N);
//             }

//             // Verify if steel yields before concrete crushing
//             if (-ey <= ec)
//             {
//              // Steel doesn't yield
//              if (N > NlimC)
//               return
//                SteelNotYielding(N);

//              // Else, concrete crushes
//              return
//               ConcreteCrushing(N);
//             }

//             // Else, steel yields first
//             if (N >= Nyc)
//              return
//               SteelNotYielding(N);

//             if (N >= NlimS)
//              return
//               SteelYielding(N);

//             return
//              SteelYieldingConcreteCrushed(N);
//            }

//            // Tension Cases
//            // Case T.1: Uncracked
//            private (double e, double de) Uncracked(double N)
//            {
//                double
//                    e  = N / t1,
//                    de = 1 / t1;

//                return
//                    (e, de);
//            }

//            // Case T.2: Cracked with not yielding steel
//            private (double e, double de) Cracked(double N)
//            {
//                double
//                    e  = (N / As - beta * sigSr) / Es,
//                    de = 1 / EsAs;

//                return
//                    (e, de);
//            }

//            // Case T.3: Cracked with yielding steel
//            private (double e, double de) YieldingSteel(double N)
//            {

//                double
//                    e = (Nyr / As - beta * sigSr) / Es + (N - Nyr) / t1,
//                    de = 1 / t1;

//                return
//                    (e, de);
//            }


//            // Compression Cases
//            // Case C.1: steel not yielding
//            private (double e, double de) SteelNotYielding(double N)
//            {
//                double
//                    k1     = (Nc / ec - EsAs * (k - 2)) / ec,
//                    k2     = (N * (k - 2) - Nc * k) / ec - EsAs,
//                    dk2    = (k - 2) / ec,
//                    rdelta = Math.Sqrt(k2 * k2 - 4 * k1 * N),

//                    // Calculate e and de
//                    e  = 0.5 * (-k2 + rdelta) / k1,
//                    de = 0.5 * (dk2 * (-k2 + rdelta) + 2 * k1) / (k1 * rdelta);

//                return
//                    (e, de);
//            }

//            // Case C.2: Concrete crushing and steel is not yielding
//            private (double e, double de) ConcreteCrushing(double N)
//            {
//                double
//                    k1     = (Nc / ec - EsAs * (k - 2)) / ec,
//                    k2     = (NlimC * (k - 2) - Nc * k) / ec - EsAs,
//                    rdelta = Math.Sqrt(k2 * k2 - 4 * k1 * NlimC),

//                    // Calculate e and de
//                    e  = 0.5 * (-k2 + rdelta) / k1 + (N - NlimC) / t1,
//                    de = 1 / t1;

//                return
//                    (e, de);
//            }

//            // Case C.3: steel is yielding and concrete is not crushed
//            private (double e, double de) SteelYielding(double N)
//            {
//                double
//                    k3     = Nc / (ec * ec),
//                    k4     = ((Nyr + N) * (k - 2) - Nc * k) / ec,
//                    k5     = Nyr + N,
//                    dk4    = (k - 2) / ec,
//                    rdelta = Math.Sqrt(k4 * k4 - 4 * k3 * k5),

//                    // Calculate e and de
//                    e  = 0.5 * (-k4 + rdelta) / k3,
//                    de = 0.5 * (dk4 * (-k4 + rdelta) + 2 * k3) / (k3 * rdelta);

//                return
//                    (e, de);
//            }

//            // Case C.4: steel is yielding and concrete is crushed
//            private (double e, double de) SteelYieldingConcreteCrushed(double N)
//            {
//                double
//                    k3    = Nc / (ec * ec),
//                    k4    = ((Nyr + NlimS) * (k - 2) - Nc * k) / ec,
//                    k5    = Nyr + NlimS,
//                    delta = Math.Sqrt(k4 * k4 - 4 * k3 * k5),

//                    // Calculate e and de
//                    e = 0.5 * (-k4 + delta) / k3 + (N - NlimS) / t1,
//                    de = 1 / t1;

//                return
//                    (e, de);
//            }
//        }

// Classic SPM model
//public class Classic : NonLinear
//{
//    public Classic(ObjectId stringerObject, ConcreteParameters concreteParameters) : base(stringerObject, concreteParameters)
//    {
//    }

//    // Calculate concrete parameters
//    //public override double fc  => Concrete.fc;
//    //public override double ec  => -0.002;
//    //public override double ecu => -0.0035;
//    //public override double Ec  => -2 * fc / ec;
//    //public override double fcr => 0.33 * Math.Sqrt(fc);
//    //public override double ecr => fcr / Ec;

//    // Maximum Stringer forces
//    public override double Nyc => -Nyr + Nc * (-2 * ey_ec - (-ey_ec) * (-ey_ec));
//    public override double Nt
//    {
//        get
//        {
//            double
//                Nt1 = Nc * (1 + xi) * (1 + xi),
//                Nt2 = Nc - Nyr;

//            return
//                Math.Max(Nt1, Nt2);
//        }
//    }

//    // Calculate the strain and derivative on a Stringer given a force N and the concrete parameters
//    public override (double e, double de) StringerStrain(double N, IntegrationPoint intPoint)
//    {
//        // Verify the value of N
//        if (N == 0)
//            return (0, 1 / t1);

//        // Tensioned Stringer
//        if (N >= 0)
//        {
//            // Calculate uncracked
//            var res = Uncracked(N);

//            // Verify if concrete is cracked
//            intPoint.VerifyCracked(res.e);


//            if (intPoint.Uncracked)
//                return res;

//            if (intPoint.CrackedAndNotYielding)
//            {
//                // Calculate cracked
//                var cracked = Cracked(N);

//                // Verify if reinforcement yielded
//                intPoint.VerifyYielding(cracked.e);

//                if (intPoint.CrackedAndNotYielding)
//                    return cracked;
//            }

//            // Steel is yielding
//            return
//                YieldingSteel(N);
//        }

//        // Compressed Stringer
//        if (N > Nt)
//            return
//                ConcreteNotCrushed(N);

//        return
//            ConcreteCrushing(N);
//    }

//    // Tension Cases
//    // Case T.1: Uncracked
//    public (double e, double de) Uncracked(double N)
//    {
//        double
//            e = N / t1,
//            de = 1 / t1;

//        return
//            (e, de);
//    }

//    // Case T.2: Cracked with not yielding steel
//    public (double e, double de) Cracked(double N)
//    {
//        double
//            e = (N * N - Nr * Nr) / (EsAs * N),
//            de = (N * N + Nr * Nr) / (EsAs * N * N);

//        return
//            (e, de);
//    }

//    // Case T.3: Cracked with yielding steel
//    public (double e, double de) YieldingSteel(double N)
//    {

//        double
//            e = (Nyr * Nyr - Nr * Nr) / (EsAs * Nyr) + (N - Nyr) / t1,
//            de = 1 / t1;

//        return
//            (e, de);
//    }

//    // Compression Cases
//    // Case C.1: concrete not crushed
//    public (double e, double de) ConcreteNotCrushed(double N)
//    {
//        // Calculate the strain for steel not yielding
//        double
//            ec = Concrete.ec,
//            ey = Steel.YieldStrain,
//            t2 = Math.Sqrt((1 + xi) * (1 + xi) - N / Nc),
//            e = ec * (1 + xi - t2);

//        // Check the strain
//        if (e < -ey)
//        {
//            // Recalculate the strain for steel yielding
//            t2 = Math.Sqrt(1 - (N + Nyr) / Nc);
//            e = ec * (1 - t2);
//        }

//        // Calculate de
//        double de = 1 / (EcAc * t2);

//        return
//            (e, de);
//    }

//    // Case C.2: Concrete crushing
//    public (double e, double de) ConcreteCrushing(double N)
//    {
//        // Calculate the strain for steel not yielding
//        double
//            ec = Concrete.ec,
//            ey = Steel.YieldStrain,
//            t2 = Math.Sqrt((1 + xi) * (1 + xi) - Nt / Nc),
//            e = ec * ((1 + xi) - t2) + (N - Nt) / t1;

//        // Check the strain
//        if (e < -ey)
//        {
//            // Recalculate the strain for steel yielding
//            e = ec * (1 - Math.Sqrt(1 - (Nyr + Nt) / Nc)) + (N - Nt) / t1;
//        }

//        // Calculate de
//        double de = 1 / t1;

//        return
//            (e, de);
//    }
//}