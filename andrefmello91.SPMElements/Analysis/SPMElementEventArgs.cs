using System;
using System.Collections.Generic;
using System.Linq;

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
		public IList<ISPMElement> Elements { get; }

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
		/// <param name="elements">The collection of elements.</param>
		/// <param name="loadStep">The load step (optional).</param>
		public SPMElementEventArgs(IEnumerable<ISPMElement> elements, int? loadStep = null)
		{
			Elements = elements.ToList();
			LoadStep = loadStep;
		}

		#endregion

	}
}