using System;

namespace SPMElements
{
	public partial class NonLinearStringer
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
            public bool Cracked  { get; set; }

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
            public bool CrackedAndYielding    =>  Cracked &&  Yielding;

			public IntegrationPoint(double ecr, double ey)
			{
				_ecr = ecr;
				_ey  = ey;
				Cracked  = false;
				Yielding = false;
			}

            /// <summary>
            /// Verify if stringer is cracked.
            /// </summary>
            /// <param name="strain">Current strain</param>
            public void VerifyCracked(double strain)
			{
				if (!Cracked && strain >= _ecr)
					Cracked = true;
			}

            /// <summary>
            /// Verify if steel is yielding.
            /// </summary>
            /// <param name="strain">Current strain</param>
            public void VerifyYielding(double strain)
			{
				if (!Yielding && Math.Abs(strain) >= _ey)
					Yielding = true;
			}
		}
	}
}