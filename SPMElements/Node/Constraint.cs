namespace SPM.Elements
{
	/// <summary>
	///     Constraint directions.
	/// </summary>
	public enum Constraint
	{
		/// <summary>
		///     Displacement free at both directions.
		/// </summary>
		Free,

		/// <summary>
		///     Displacement restricted at X direction.
		/// </summary>
		X,

		/// <summary>
		///     Displacement restricted at Y direction.
		/// </summary>
		Y,

		/// <summary>
		///     Displacement restricted at both directions.
		/// </summary>
		XY
	}
}