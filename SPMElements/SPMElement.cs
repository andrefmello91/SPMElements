using System;
using Autodesk.AutoCAD.DatabaseServices;

namespace SPM.Elements
{
	/// <summary>
    /// SPM element types.
    /// </summary>
	public enum ElementTypes
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
    public abstract class SPMElement
    {
		// Auxiliary fields
		protected int[] Indexes;

		/// <summary>
        /// Get or set the number of the element.
        /// </summary>
	    public int Number { get; set; }

        /// <summary>
        /// Get or set the <see cref="Autodesk.AutoCAD.DatabaseServices.ObjectId"/> of element.
        /// </summary>
        public ObjectId ObjectId { get; set; }

		/// <summary>
        /// Get the DoF index of the element.
        /// </summary>
	    public abstract int[] DoFIndex { get; }

		/// <summary>
		/// Base object of SPM elements.
		/// </summary>
		public SPMElement(ObjectId objectId, int number)
		{
			ObjectId = objectId;
			Number   = number;
		}

        /// <summary>
        /// Get global indexes of a grip.
        /// </summary>
        /// <param name="gripNumber">The grip number.</param>
        /// <returns></returns>
        protected int[] GlobalIndexes(int gripNumber)
	    {
		    Indexes =
			    new[]
			    {
				    2 * gripNumber - 2, 2 * gripNumber - 1
			    };

		    return Indexes;
	    }

        /// <summary>
        /// Get global indexes of an element's grips
        /// </summary>
        /// <param name="gripNumbers">The grip numbers of the element.</param>
        /// <returns></returns>
        protected int[] GlobalIndexes(int[] gripNumbers)
	    {
		    // Initialize the array
		    int[] ind = new int[2 * gripNumbers.Length];

		    // Get the indexes
		    for (int i = 0; i < gripNumbers.Length; i++)
		    {
			    int j = 2 * i;

			    ind[j] = 2 * gripNumbers[i] - 2;
			    ind[j + 1] = 2 * gripNumbers[i] - 1;
		    }

		    Indexes = ind;

		    return ind;
	    }
    }
}
