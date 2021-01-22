using System;
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
    public abstract class SPMElement
    {
		// Auxiliary fields
		protected int[] Indexes;
		private int? _number;
		private ObjectId? _id;

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
			get => _number ?? 0;
			set
			{
				var old = _number;
				_number = value;

				if (old.HasValue)
					RaiseIntEvent(NumberChanged, old.Value, value);
			}
		}

		/// <summary>
		/// Get or set the <see cref="Autodesk.AutoCAD.DatabaseServices.ObjectId"/> of element.
		/// </summary>
		public ObjectId ObjectId
		{
			get => _id ?? ObjectId.Null;
			set
			{
				var old = _id;
				_id     = value;

				if (old.HasValue)
					RaiseObjectIdEvent(ObjectIdChanged, old.Value, value);
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

        /// <summary>
        /// Raise the <see cref="int"/> changed event.
        /// </summary>
        protected void RaiseIntEvent(EventHandler<ParameterChangedEventArgs<int>> eventHandler, int oldValue, int newValue)
        {
	        // Copy to a temporary variable to be thread-safe (MSDN).
	        var tmp = eventHandler;
	        tmp?.Invoke(this, new ParameterChangedEventArgs<int>(oldValue, newValue)); ;
        }

        /// <summary>
        /// Raise the <see cref="ObjectId"/> changed event.
        /// </summary>
        protected void RaiseObjectIdEvent(EventHandler<ParameterChangedEventArgs<ObjectId>> eventHandler, ObjectId oldValue, ObjectId newValue)
        {
	        // Copy to a temporary variable to be thread-safe (MSDN).
	        var tmp = eventHandler;
	        tmp?.Invoke(this, new ParameterChangedEventArgs<ObjectId>(oldValue, newValue)); ;
        }
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
