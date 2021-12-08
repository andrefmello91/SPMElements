using andrefmello91.Extensions;
using andrefmello91.FEMAnalysis;
using MathNet.Numerics.LinearAlgebra;
using UnitsNet;
using UnitsNet.Units;

namespace andrefmello91.SPMElements.Monitors;

/// <summary>
///     Stringer monitor class.
/// </summary>
internal class StringerMonitor : ElementMonitor
{

	#region Fields

	private readonly LengthUnit _crackUnit;
	private readonly ForceUnit _forceUnit;

	#endregion

	#region Constructors

	/// <summary>
	///     Create a stringer monitor.
	/// </summary>
	/// <inheritdoc />
	/// <param name="forceUnit">The unit for stringer forces.</param>
	/// <param name="crackUnit">The unit for crack openings.</param>
	public StringerMonitor(string name, ForceUnit forceUnit = ForceUnit.Kilonewton, LengthUnit crackUnit = LengthUnit.Millimeter)
		: base(name, Label(forceUnit, crackUnit))
	{
		_crackUnit = crackUnit;
		_forceUnit = forceUnit;
	}

	#endregion

	#region Methods

	private static string[] Label(ForceUnit forceUnit, LengthUnit crackUnit)
	{
		var fc = forceUnit.Abbrev();

		return new[]
		{
			nameof(MonitoredValue.LoadFactor),
			nameof(MonitoredValue.MinStrain),
			nameof(MonitoredValue.MaxStrain),
			$"{nameof(MonitoredValue.MinForce)}({fc})",
			$"{nameof(MonitoredValue.MaxForce)}({fc})",
			$"{nameof(MonitoredValue.MaxCrack)}({crackUnit.Abbrev()})"
		};
	}

	/// <inheritdoc />
	public override void AddMonitoredValue(double loadFactor, INumberedElement element)
	{
		if (element is not NLStringer stringer)
			return;

		Values.Add(new MonitoredValue(stringer, loadFactor, _forceUnit, _crackUnit));
	}

	#endregion

	private class MonitoredValue
		: IVectorTransformable
	{

		#region Properties

		public double LoadFactor { get; }

		public double MaxCrack { get; }

		public double MaxForce { get; }

		public double MaxStrain { get; }

		public double MinForce { get; }

		public double MinStrain { get; }

		#endregion

		#region Constructors

		public MonitoredValue(NLStringer stringer, double loadFactor, ForceUnit forceUnit = ForceUnit.Kilonewton, LengthUnit crackUnit = LengthUnit.Millimeter)
		{
			LoadFactor = loadFactor;
			MinStrain  = stringer.Strains.Minimum();
			MaxStrain  = stringer.Strains.Maximum();
			MinForce   = UnitMath.Min(stringer.NormalForces.N1, stringer.NormalForces.N3).As(forceUnit);
			MaxForce   = UnitMath.Max(stringer.NormalForces.N1, stringer.NormalForces.N3).As(forceUnit);
			MaxCrack   = stringer.CrackOpenings.Max(crackUnit).Value;
		}

		#endregion

		#region Methods

		/// <inheritdoc />
		public Vector<double> AsVector() => new[]
		{
			LoadFactor,
			MinStrain,
			MaxStrain,
			MinForce,
			MaxForce,
			MaxCrack
		}.ToVector();

		#endregion

	}
}