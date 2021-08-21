using System.Collections.Generic;
using System.Linq;
using andrefmello91.Extensions;
using andrefmello91.FEMAnalysis;
using andrefmello91.OnPlaneComponents;
using andrefmello91.SPMElements.PanelProperties;
using UnitsNet;

namespace andrefmello91.SPMElements
{
	/// <summary>
	///     Extensions class.
	/// </summary>
	public static class Extensions
	{

		#region Methods

		/// <summary>
		///     Get the <see cref="AnalysisType" /> from this <see cref="ElementModel" />.
		/// </summary>
		/// <returns>
		///     <see cref="AnalysisType.Linear" /> if <paramref name="model" /> is <see cref="ElementModel.Elastic" />,
		///     <see cref="AnalysisType.Nonlinear" /> otherwise.
		/// </returns>
		public static AnalysisType AsAnalysisType(this ElementModel model) =>
			model switch
			{
				ElementModel.Elastic => AnalysisType.Linear,
				_                    => AnalysisType.Nonlinear
			};

		/// <summary>
		///     Get the <see cref="ElementModel" /> from this <see cref="AnalysisType" />.
		/// </summary>
		/// <returns>
		///     <see cref="ElementModel.Elastic" /> if <paramref name="type" /> is <see cref="AnalysisType.Linear" />,
		///     <see cref="ElementModel.Nonlinear" /> otherwise.
		/// </returns>
		public static ElementModel AsElementModel(this AnalysisType type) =>
			type switch
			{
				AnalysisType.Linear => ElementModel.Elastic,
				_                   => ElementModel.Nonlinear
			};

		/// <summary>
		///     Return the <see cref="Node" /> of an array, in given <paramref name="position" />.
		/// </summary>
		/// <param name="nodes">The collection of <see cref="Node" />'s.</param>
		/// <param name="position">The position wanted.</param>
		public static Node GetByPosition(this IEnumerable<Node> nodes, Point position) => nodes.First(node => position == node.Position);

		/// <summary>
		///     Set stringer dimensions on edges of each <see cref="NLPanel" />.
		///     <para>See: <see cref="Edge.SetStringerDimension" /></para>
		/// </summary>
		/// <param name="stringers">The array containing all of the stringers.</param>
		public static void SetStringerDimensions(this IEnumerable<Panel> panels, IEnumerable<Stringer> stringers)
		{
			foreach (var panel in panels)
				switch (panel)
				{
					case NLPanel nlPanel:
						nlPanel.SetStringersDimensions(stringers);
						break;
				}
		}

		/// <summary>
		///		Check if a point is inside a panel geometry.
		/// </summary>
		/// <param name="point">The point.</param>
		/// <param name="panelGeometry">The panel geometry.</param>
		/// <returns>
		///		<b>True</b> if <paramref name="point"/> is inside <paramref name="panelGeometry"/>.
		/// </returns>
		public static bool IsInside(this Point point, PanelGeometry panelGeometry) =>
			point.X.IsBetween(panelGeometry.Edge4.CenterPoint.X, panelGeometry.Edge2.CenterPoint.X) &&
			point.Y.IsBetween(panelGeometry.Edge1.CenterPoint.Y, panelGeometry.Edge3.CenterPoint.Y);

		/// <summary>
		///		Check if a point is inside a panel vertices.
		/// </summary>
		/// <param name="point">The point.</param>
		/// <param name="vertices">The vertices.</param>
		/// <returns>
		///		<b>True</b> if <paramref name="point"/> is inside <paramref name="vertices"/>.
		/// </returns>
		public static bool IsInside(this Point point, Vertices vertices) =>
			point.X.IsBetween(vertices.Vertex1.MidPoint(vertices.Vertex4).X, vertices.Vertex2.MidPoint(vertices.Vertex3).X) &&
			point.Y.IsBetween(vertices.Vertex1.MidPoint(vertices.Vertex2).Y, vertices.Vertex4.MidPoint(vertices.Vertex3).Y);


		#endregion

	}
}