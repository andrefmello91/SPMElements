using System;
using Extensions.Number;

namespace SPM.Elements
{
	public partial class NLStringer
	{
		/// <summary>
		/// Struct to verify cracked or yielded state on each integration point
		/// </summary>
		private struct IntegrationPoint
		{
			// Auxiliary fields
			private readonly double _ecr, _ey;

            /// <summary>
            /// Get/set cracked state.
            /// </summary>
            private bool Cracked  { get; set; }

			/// <summary>
			/// Get/set yielding state.
			/// </summary>
			public bool Yielding { get; set; }
			
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
			public (double e, double de) LastGenStrain { get; set; }

			public IntegrationPoint(double ecr, double ey)
			{
				_ecr = ecr;
				_ey  = ey;
				Cracked  = false;
				Yielding = false;
				LastGenStrain = (0, 0);
			}

            /// <summary>
            /// Verify if stringer is cracked. Returns true if cracked.
            /// </summary>
            /// <param name="strain">Current strain</param>
            public bool VerifyCracked(double strain)
			{
				if (!Cracked && strain >= _ecr)
					Cracked = true;

				return Cracked;
			}

            /// <summary>
            /// Verify if steel is yielding. Returns true if yielding.
            /// </summary>
            /// <param name="strain">Current strain</param>
            public bool VerifyYielding(double strain)
			{
				if (!Yielding && strain.Abs() >= _ey)
					Yielding = true;

				return Yielding;
			}
		}
	}
}