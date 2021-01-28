using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using Extensions;

namespace SPM.Elements
{
	/// <summary>
    /// SPM element types.
    /// </summary>
	public enum ElementType
	{
		Default,
		Node,
		Stringer,
		Panel,
		Support,
		Force
	}

	/// <summary>
    /// The analysis types.
    /// </summary>
	public enum AnalysisType
	{
		Linear,
		NonLinear
	}

	/// <summary>
    /// Base class of SPM elements.
    /// </summary>
    public interface ISPMElement
    {
		/// <summary>
        /// Get or set the number of the element.
        /// </summary>
	    int Number { get; set ; }

		/// <summary>
        /// Get the DoF index of the element.
        /// </summary>
	    int[] DoFIndex { get; }
    }
}
