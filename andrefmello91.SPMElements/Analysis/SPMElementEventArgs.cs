using System;

namespace andrefmello91.SPMElements
{
	/// <summary>
	///     Class for SPM elements event args
	/// </summary>
	public class SPMElementEventArgs : EventArgs
	{

		#region Properties

		/// <summary>
		///     The SPM element.
		/// </summary>
		public ISPMElement Element { get; }

		/// <summary>
		///     The load step.
		/// </summary>
		/// <remarks>
		///     Optional.
		/// </remarks>
		public int? LoadStep { get; }

		#endregion

		#region Constructors

		/// <summary>
		///     Create a SPM element event arg.
		/// </summary>
		/// <param name="element">The element.</param>
		/// <param name="loadStep">The load step (optional).</param>
		public SPMElementEventArgs(ISPMElement element, int? loadStep = null)
		{
			Element  = element;
			LoadStep = loadStep;
		}

		#endregion

	}
}