#ifndef MESH_H
#define MESH_H

#include "Point3D.h"
#include "Vector3D.h"
#include <vector>
#include <cstdint>

namespace CADLib {

// Forward declarations
class Surface;
class Curve;
class Line;
class Circle;
class Arc;
class Plane;

/**
 * @brief Simple mesh structure for visualization
 * 
 * Contains vertices, normals, and indices for triangle mesh rendering.
 * This is a pure C++ class with no dependencies, suitable for OpenGL rendering.
 */
struct Mesh {
    std::vector<float> vertices;   // x,y,z,x,y,z,... (3 floats per vertex)
    std::vector<float> normals;    // nx,ny,nz,nx,ny,nz,... (3 floats per normal)
    std::vector<uint32_t> indices; // Triangle indices (3 per triangle)
    
    // Constructor
    Mesh() = default;
    
    // Add a vertex and normal
    void addVertex(const Point3D& vertex, const Vector3D& normal);
    
    // Add a triangle by indices
    void addTriangle(uint32_t i0, uint32_t i1, uint32_t i2);
    
    // Get counts
    size_t vertexCount() const { return vertices.size() / 3; }
    size_t triangleCount() const { return indices.size() / 3; }
    
    // Clear all data
    void clear();
    
    // Compute bounds
    void getBounds(Point3D& min, Point3D& max) const;
    
    // Merge another mesh into this one
    void merge(const Mesh& other);
    
    // Generate edge lines for wireframe rendering
    std::vector<float> generateEdges() const;
};

/**
 * @brief Mesh generation utilities
 */
class MeshGenerator {
public:
    /**
     * @brief Tessellate a surface into a triangle mesh
     */
    static Mesh tessellate(const Surface& surface, int uSamples = 20, int vSamples = 20);
    
    /**
     * @brief Generate mesh for a line (as a thin cylinder or line segments)
     */
    static Mesh generateLine(const Line& line, bool asLineSegment = true);
    
    /**
     * @brief Generate mesh for a curve (as connected line segments)
     */
    static Mesh generateCurve(const Curve& curve, int samples = 50);
    
    /**
     * @brief Generate mesh for a circle (as a line loop or filled disk)
     */
    static Mesh generateCircle(const Circle& circle, int segments = 32, bool filled = false);
    
    /**
     * @brief Generate mesh for an arc
     */
    static Mesh generateArc(const Arc& arc, int segments = 32);
    
    /**
     * @brief Generate a sphere mesh
     */
    static Mesh generateSphere(const Point3D& center, double radius, 
                               int stacks = 20, int slices = 20);
    
    /**
     * @brief Generate a grid/plane mesh for construction planes
     */
    static Mesh generatePlaneGrid(const Plane& plane, double size = 10.0, 
                                  int divisions = 10);
    
    /**
     * @brief Generate coordinate axes
     */
    static Mesh generateAxes(double length = 1.0);
};

} // namespace CADLib

#endif // MESH_H
