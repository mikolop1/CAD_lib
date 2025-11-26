# CADLib - 3D CAD Library for Sheet Objects

A lightweight C++ CAD library focused on 3D geometry, curves, and sheet surfaces. Designed for compatibility with Qt and C++ visualization frameworks.

## Features

- **3D Point Operations**: Complete Point3D class with distance calculations and arithmetic operations
- **Vector Mathematics**: Vector3D class with dot product, cross product, normalization, and angle calculations
- **Transformation System**: Matrix4x4 for translations, rotations, scaling, and combined transformations
- **Curve Support**: 
  - Line segments
  - Bezier curves (any degree)
  - Parametric curve evaluation and sampling
- **Surface Support**:
  - Planar surfaces
  - Bezier surfaces (tensor product surfaces)
  - Surface normals and parameter space evaluation
- **Qt Compatible**: Designed to integrate seamlessly with Qt/C++ visualization

## Project Structure

```
CAD_lib/
├── include/           # Header files
│   ├── Point3D.h
│   ├── Vector3D.h
│   ├── Matrix4x4.h
│   └── Geometry.h
├── src/              # Implementation files
│   ├── Point3D.cpp
│   ├── Vector3D.cpp
│   ├── Matrix4x4.cpp
│   └── Geometry.cpp
├── examples/         # Example programs
│   ├── basic_example.cpp
│   ├── curves_example.cpp
│   ├── surfaces_example.cpp
│   └── transformations_example.cpp
├── cmake/            # CMake configuration
└── CMakeLists.txt    # Build configuration
```

## Building the Library

### Prerequisites

- CMake 3.10 or higher
- C++17 compatible compiler (GCC, Clang, MSVC)
- Optional: Qt5 or Qt6 for enhanced integration

### Build Instructions

```bash
# Create build directory
mkdir build
cd build

# Configure with CMake
cmake ..

# Build the library
cmake --build .

# Optional: Install the library
sudo cmake --install .
```

### Build Options

- `BUILD_EXAMPLES`: Build example programs (default: ON)

```bash
cmake .. -DBUILD_EXAMPLES=OFF
```

## Usage Examples

### Basic Point and Vector Operations

```cpp
#include "Point3D.h"
#include "Vector3D.h"

using namespace CADLib;

// Create points
Point3D p1(1.0, 2.0, 3.0);
Point3D p2(4.0, 5.0, 6.0);

// Calculate distance
double dist = p1.distanceTo(p2);

// Create vectors
Vector3D v1(1.0, 0.0, 0.0);
Vector3D v2(0.0, 1.0, 0.0);

// Vector operations
double dot = v1.dot(v2);
Vector3D cross = v1.cross(v2);
double angle = v1.angleToInDegrees(v2);
```

### Transformations

```cpp
#include "Matrix4x4.h"

using namespace CADLib;

Point3D point(1.0, 0.0, 0.0);

// Translation
Matrix4x4 translation = Matrix4x4::translation(5.0, 3.0, 2.0);
Point3D translated = translation.transform(point);

// Rotation (90 degrees around Z axis)
Matrix4x4 rotation = Matrix4x4::rotationZ(M_PI / 2);
Point3D rotated = rotation.transform(point);

// Combined transformations
Matrix4x4 combined = translation * rotation;
Point3D transformed = combined.transform(point);
```

### Working with Curves

```cpp
#include "Geometry.h"

using namespace CADLib;

// Create a cubic Bezier curve
std::vector<Point3D> controlPoints = {
    Point3D(0, 0, 0),
    Point3D(2, 8, 0),
    Point3D(8, 8, 0),
    Point3D(10, 0, 0)
};

BezierCurve curve(controlPoints);

// Evaluate curve at parameter t
Point3D point = curve.pointAt(0.5);
Vector3D tangent = curve.tangentAt(0.5);

// Sample the curve
std::vector<Point3D> samples = curve.sample(20);

// Transform the curve
Matrix4x4 transform = Matrix4x4::rotationZ(M_PI / 4);
curve.transform(transform);
```

### Working with Surfaces

```cpp
#include "Geometry.h"

using namespace CADLib;

// Create a planar surface
Point3D origin(0, 0, 0);
Vector3D uAxis(1, 0, 0);
Vector3D vAxis(0, 1, 0);
PlanarSurface surface(origin, uAxis, vAxis, 10.0, 5.0);

// Evaluate surface at parameters (u, v)
Point3D point = surface.pointAt(0.5, 0.5);
Vector3D normal = surface.normalAt(0.5, 0.5);

// Create a Bezier surface
std::vector<std::vector<Point3D>> controlNet = {
    {Point3D(0, 0, 0), Point3D(0, 10, 0)},
    {Point3D(10, 0, 0), Point3D(10, 10, 2)}
};

BezierSurface bezier(controlNet);
Point3D surfacePoint = bezier.pointAt(0.5, 0.5);
```

## Integration with Qt

The library is designed to be compatible with Qt. When Qt is detected during build, additional integration features are enabled:

```cpp
// The library can be linked with Qt projects
// Column-major matrix format matches OpenGL/Qt conventions
Matrix4x4 transform = Matrix4x4::rotationZ(angle);
// Use transform data with QMatrix4x4 or OpenGL
```

## API Overview

### Point3D
- Constructors and arithmetic operators
- Distance calculations
- Midpoint calculation
- Transformation support

### Vector3D
- Direction and magnitude operations
- Dot and cross products
- Normalization
- Angle calculations
- Projection and reflection

### Matrix4x4
- 4x4 transformation matrices
- Translation, rotation, scaling
- Matrix multiplication
- Inversion and transposition
- Column-major storage (OpenGL/Qt compatible)

### Geometry Classes
- **Geometry**: Base class for all geometric entities
- **Line**: Line segment with closest point and distance calculations
- **Curve**: Base class for parametric curves
- **BezierCurve**: Bezier curves of any degree
- **Surface**: Base class for parametric surfaces
- **PlanarSurface**: Flat rectangular surfaces
- **BezierSurface**: Tensor product Bezier surfaces

## Running Examples

After building, run the example programs:

```bash
# From the build directory
./examples/basic_example
./examples/curves_example
./examples/surfaces_example
./examples/transformations_example
```

## License

This library is provided as-is for educational and commercial use.

## Future Enhancements

Potential areas for expansion:
- NURBS curves and surfaces
- Trimmed surfaces
- Boolean operations on sheet objects
- Import/export formats (STEP, IGES)
- Mesh generation from surfaces
- Intersection calculations
- Qt widget for 3D visualization

## Contributing

Feel free to extend and modify this library for your CAD applications.
