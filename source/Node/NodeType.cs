namespace SPM.Elements
{
	/// <summary>
	///     Node types.
	/// </summary>
	public enum NodeType
	{
		/// <summary>
		///     All nodes (excluding displaced nodes).
		/// </summary>
		All,

		/// <summary>
		///     External nodes (grips to stringers).
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