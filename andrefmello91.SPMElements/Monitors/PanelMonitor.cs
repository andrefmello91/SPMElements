using andrefmello91.Extensions;
using andrefmello91.FEMAnalysis;
using andrefmello91.OnPlaneComponents;
using MathNet.Numerics.LinearAlgebra;
using UnitsNet;
using UnitsNet.Units;

namespace andrefmello91.SPMElements.Monitors;

/// <summary>
///     Panel monitor class.
/// </summary>
internal class PanelMonitor : ElementMonitor
{

	#region Fields

	private readonly LengthUnit _crackUnit;
	private readonly PressureUnit _stressUnit;

	#endregion

	#region Constructors

	/// <summary>
	///     Create a panel monitor.
	/// </summary>
	/// <inheritdoc />
	/// <param name="stressUnit">The unit for panel stresses.</param>
	/// <param name="crackUnit">The unit for crack openings.</param>
	public PanelMonitor(string name, PressureUnit stressUnit = PressureUnit.Megapascal, LengthUnit crackUnit = LengthUnit.Millimeter)
		: base(name, Label(stressUnit, crackUnit))
	{
		_crackUnit  = crackUnit;
		_stressUnit = stressUnit;
	}

	#endregion

	#region Methods

	private static string[] Label(PressureUnit stressUnit, LengthUnit crackUnit)
	{
		var p = stressUnit.Abbrev();

		return new[]
		{
			nameof(MonitoredValue.LoadFactor),
			"ex",
			"ey",
			"exy",
			$"sx ({p})",
			$"sy ({p})",
			$"sxy ({p})",
			$"{nameof(MonitoredValue.CrackWidth)}({crackUnit.Abbrev()})"
		};
	}

	/// <inheritdoc />
	public override void AddMonitoredValue(double loadFactor, INumberedElement element)
	{
		if (element is not NLPanel panel)
			return;

		Values.Add(new MonitoredValue(panel, loadFactor, _stressUnit, _crackUnit));
	}

	#endregion

	private class MonitoredValue
		: IVectorTransformable
	{

		#region Properties

		public IState<double> AverageStrains { get; }

		public IState<Pressure> AverageStress { get; }

		public double CrackWidth { get; }
		public double LoadFactor { get; }

		#endregion

		#region Constructors

		public MonitoredValue(NLPanel panel, double loadFactor, PressureUnit stressUnit = PressureUnit.Megapascal, LengthUnit crackUnit = LengthUnit.Millimeter)
		{
			LoadFactor     = loadFactor;
			AverageStrains = panel.AverageStrains;
			AverageStress  = panel.AverageStresses.Convert(stressUnit);
			CrackWidth     = panel.CrackOpening.As(crackUnit);
		}

		#endregion

		#region Methods

		/// <inheritdoc />
		public Vector<double> AsVector() => new[]
		{
			LoadFactor,
			AverageStrains.X,
			AverageStrains.Y,
			AverageStrains.XY,
			AverageStress.X.Value,
			AverageStress.Y.Value,
			AverageStress.XY.Value,
			CrackWidth
		}.ToVector();

		#endregion

	}
}