namespace andrefmello91.SPMElements
{
	/// <summary>
	///     Element models.
	/// </summary>
	public enum ElementModel
	{
		/// <summary>
		///     Elastic model for linear analysis.
		/// </summary>
		Elastic,

		/// <summary>
		///     Nonlinear model, for nonlinear analysis.
		/// </summary>
		Nonlinear
	}

	/// <summary>
	///     Node types.
	/// </summary>
	public enum NodeType
	{
		/// <summary>
		///     External nodes (external grips to stringers).
		/// </summary>
		External,

		/// <summary>
		///     External nodes (mid grip to stringers and grip for panels).
		/// </summary>
		Internal,

		/// <summary>
		///     Displaced nodes (auxiliary, only for result drawing).
		/// </summary>
		Displaced
	}

	/// <summary>
	///     Type of forces that stringer can be loaded.
	/// </summary>
	public enum StringerForceState
	{
		/// <summary>
		///     Stringer is not loaded.
		/// </summary>
		Unloaded,

		/// <summary>
		///     Stringer is fully tensioned.
		/// </summary>
		PureTension,

		/// <summary>
		///     Stringer is fully compressed.
		/// </summary>
		PureCompression,

		/// <summary>
		///     Stringer is tensioned at an end and compressed at the other end.
		/// </summary>
		Combined
	}

}