using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using Autodesk.AutoCAD.DatabaseServices;

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
    public abstract class SPMElement : IEquatable<SPMElement>
    {
		// Auxiliary fields
		protected int[] Indexes;
		private int _number;
		private ObjectId _id;

		/// <summary>
		/// Event to run when <see cref="Number"/> changes.
		/// </summary>
		public EventHandler<ParameterChangedEventArgs<int>> NumberChanged;

		/// <summary>
		/// Event to run when <see cref="ObjectId"/> changes.
		/// </summary>
		public EventHandler<ParameterChangedEventArgs<ObjectId>> ObjectIdChanged;
		
		/// <summary>
        /// Get or set the number of the element.
        /// </summary>
	    public int Number
		{ 
			get => _number;
			set
			{
				var old = _number;
				_number = value;

				if (old != value)
					RaiseParameterEvent(NumberChanged, old, value);
			}
		}

		/// <summary>
		/// Get or set the <see cref="Autodesk.AutoCAD.DatabaseServices.ObjectId"/> of element.
		/// </summary>
		public ObjectId ObjectId
		{
			get => _id;
			set
			{
				var old = _id;
				_id     = value;

				if (old != value)
					RaiseParameterEvent(ObjectIdChanged, old, value);
			}
		}

		/// <summary>
        /// Get the DoF index of the element.
        /// </summary>
	    public abstract int[] DoFIndex { get; }

		/// <summary>
		/// Base object of SPM elements.
		/// </summary>
		protected SPMElement(ObjectId objectId, int number)
		{
			ObjectId = objectId;
			Number   = number;
		}

        /// <summary>
        /// Get global indexes of a grip.
        /// </summary>
        /// <param name="gripNumber">The grip number.</param>
        protected int[] GetIndexes(int gripNumber)
        {
	        Indexes = GlobalIndexes(gripNumber).ToArray();

		    return Indexes;
	    }

        /// <summary>
        /// Get global indexes of an element's grips
        /// </summary>
        /// <param name="gripNumbers">The grip numbers of the element.</param>
        protected int[] GetIndexes(IEnumerable<int> gripNumbers)
	    {
		    Indexes = GlobalIndexes(gripNumbers).ToArray();

		    return Indexes;
	    }

        /// <summary>
        /// Get global indexes of an element's grips
        /// </summary>
        /// <param name="gripNumbers">The grip numbers of the element.</param>
        public static IEnumerable<int> GlobalIndexes(IEnumerable<int> gripNumbers)
	    {
		    // Initialize the array
		    var count = gripNumbers.Count();

		    // Get the indexes
		    for (int i = 0; i < count; i++)
		    {
			    var n = 2 * gripNumbers.ElementAt(i);

			    yield return n - 2;
			    yield return n - 1;
		    }
	    }

		/// <summary>
		/// Get global indexes of an element's grips
		/// </summary>
		/// <param name="gripNumber">The grip number of the element.</param>
		public static IEnumerable<int> GlobalIndexes(int gripNumber)
	    {
		    var n = 2 * gripNumber;

		    yield return n - 2;
		    yield return n - 1;
	    }

        /// <summary>
        /// Raise the parameter changed event.
        /// </summary>
        protected void RaiseParameterEvent<T>(EventHandler<ParameterChangedEventArgs<T>> eventHandler, T oldValue, T newValue)
        {
	        // Copy to a temporary variable to be thread-safe (MSDN).
	        var tmp = eventHandler;
	        tmp?.Invoke(this, new ParameterChangedEventArgs<T>(oldValue, newValue)); ;
        }

        /// <summary>
        /// Returns true if this elements is equal to <paramref name="other"/>.
        /// </summary>
        public abstract bool Equals(SPMElement other);
    }

	/// <summary>
	/// Parameter changed class.
	/// </summary>
	public class ParameterChangedEventArgs<T> : EventArgs
	{
		/// <summary>
		/// Get the old value of the parameter.
		/// </summary>
		public T OldValue { get; }

		/// <summary>
		/// Get the new value of the parameter.
		/// </summary>
		public T NewValue { get; }

		public ParameterChangedEventArgs(T oldValue, T newValue)
		{
			OldValue = oldValue;
			NewValue = newValue;
		}
	}
}
