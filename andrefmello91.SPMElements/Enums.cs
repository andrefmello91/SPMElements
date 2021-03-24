namespace andrefmello91.SPMElements
{
	/// <summary>
	///		Element models.
	/// </summary>
	public enum ElementModel
	{
		/// <summary>
		///		Elastic model for linear analysis.
		/// </summary>
		Elastic,
		
		/// <summary>
		///		Nonlinear model, for nonlinear analysis.
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
}