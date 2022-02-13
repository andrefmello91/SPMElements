﻿using System;
using System.Collections.Generic;
using System.Linq;

namespace andrefmello91.SPMElements;

/// <summary>
///     Class for SPM elements event args
/// </summary>
public class SPMElementEventArgs : EventArgs
{

	/// <summary>
	///     The SPM element.
	/// </summary>
	public IList<ISPMElement> Elements { get; }

	/// <summary>
	///     The load step.
	/// </summary>
	/// <remarks>
	///     Optional.
	/// </remarks>
	public int? LoadStep { get; }

	/// <summary>
	///     Create a SPM element event arg.
	/// </summary>
	/// <param name="elements">The collection of elements.</param>
	/// <param name="loadStep">The load step (optional).</param>
	public SPMElementEventArgs(IEnumerable<ISPMElement> elements, int? loadStep = null)
	{
		Elements = elements.ToList();
		LoadStep = loadStep;
	}
}

/// <summary>
///     Event args class for state changing of nonlinear SPM elements.
/// </summary>
public class StateEventArgs : EventArgs
{

	/// <summary>
	///     The name of the state changed.
	/// </summary>
	public string StateName { get; }

	/// <inheritdoc />
	/// <param name="stateName">The name of the state changed.</param>
	public StateEventArgs(string stateName) => StateName = stateName;
}