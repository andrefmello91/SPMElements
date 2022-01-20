# SPMElements
Stringer-Panel Model finite elements repository.

This library implements the finite elements of the **Stringer-Panel Method**.

The implementation is based in [Hoogenboom (1998) formulation](http://resolver.tudelft.nl/uuid:9ebcace7-f3d7-4ee0-93ff-cd157f4e5774).

This library uses:

- [MathNet.Numerics](https://github.com/mathnet/mathnet-numerics) for Linear Algebra operations;

- [Units.NET](https://github.com/angularsen/UnitsNet) for simple unit conversions;

- [OnPlaneComponents](https://github.com/andrefmello91/On-Plane-Components) for stress and strain transformations;

- [Extensions](https://github.com/andrefmello91/Extensions) for some numeric extensions;

- [Material](https://github.com/andrefmello91/Material) for concrete and reinforcement constitutive models;

- [Reinforced Concrete Membrane](https://github.com/andrefmello91/Reinforced-Concrete-Membrane) for membrane calculations;

- [FEM-Analysis](https://github.com/andrefmello91/FEM-Analysis) as base for finite elements.

## Usage

### Package reference:

`<PackageReference Include="andrefmello91.SPMElements" Version="1.X.X" />`

### .NET CLI:

`dotnet add package andrefmello91.SPMElements --version 1.X.X`