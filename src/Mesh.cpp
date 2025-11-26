#include "Mesh.h"
#include "Geometry.h"
#include "Circle.h"
#include "Plane.h"
#include <cmath>
#include <algorithm>

namespace CADLib {

// ============================================================================
// Mesh Implementation
// ============================================================================

void Mesh::addVertex(const Point3D& vertex, const Vector3D& normal) {
    vertices.push_back(static_cast<float>(vertex.x));
    vertices.push_back(static_cast<float>(vertex.y));
    vertices.push_back(static_cast<float>(vertex.z));
    
    normals.push_back(static_cast<float>(normal.x));
    normals.push_back(static_cast<float>(normal.y));
    normals.push_back(static_cast<float>(normal.z));
}

void Mesh::addTriangle(uint32_t i0, uint32_t i1, uint32_t i2) {
    indices.push_back(i0);
    indices.push_back(i1);
    indices.push_back(i2);
}

void Mesh::clear() {
    vertices.clear();
    normals.clear();
    indices.clear();
}

void Mesh::getBounds(Point3D& min, Point3D& max) const {
    if (vertices.empty()) {
        min = max = Point3D(0, 0, 0);
        return;
    }
    
    min = Point3D(vertices[0], vertices[1], vertices[2]);
    max = min;
    
    for (size_t i = 0; i < vertices.size(); i += 3) {
        min.x = std::min(min.x, static_cast<double>(vertices[i]));
        min.y = std::min(min.y, static_cast<double>(vertices[i + 1]));
        min.z = std::min(min.z, static_cast<double>(vertices[i + 2]));
        
        max.x = std::max(max.x, static_cast<double>(vertices[i]));
        max.y = std::max(max.y, static_cast<double>(vertices[i + 1]));
        max.z = std::max(max.z, static_cast<double>(vertices[i + 2]));
    }
}

void Mesh::merge(const Mesh& other) {
    uint32_t baseIndex = static_cast<uint32_t>(vertexCount());
    
    vertices.insert(vertices.end(), other.vertices.begin(), other.vertices.end());
    normals.insert(normals.end(), other.normals.begin(), other.normals.end());
    
    for (uint32_t idx : other.indices) {
        indices.push_back(idx + baseIndex);
    }
}

std::vector<float> Mesh::generateEdges() const {
    std::vector<float> edges;
    
    for (size_t i = 0; i < indices.size(); i += 3) {
        uint32_t i0 = indices[i] * 3;
        uint32_t i1 = indices[i + 1] * 3;
        uint32_t i2 = indices[i + 2] * 3;
        
        // Edge 0-1
        edges.push_back(vertices[i0]);
        edges.push_back(vertices[i0 + 1]);
        edges.push_back(vertices[i0 + 2]);
        edges.push_back(vertices[i1]);
        edges.push_back(vertices[i1 + 1]);
        edges.push_back(vertices[i1 + 2]);
        
        // Edge 1-2
        edges.push_back(vertices[i1]);
        edges.push_back(vertices[i1 + 1]);
        edges.push_back(vertices[i1 + 2]);
        edges.push_back(vertices[i2]);
        edges.push_back(vertices[i2 + 1]);
        edges.push_back(vertices[i2 + 2]);
        
        // Edge 2-0
        edges.push_back(vertices[i2]);
        edges.push_back(vertices[i2 + 1]);
        edges.push_back(vertices[i2 + 2]);
        edges.push_back(vertices[i0]);
        edges.push_back(vertices[i0 + 1]);
        edges.push_back(vertices[i0 + 2]);
    }
    
    return edges;
}

// ============================================================================
// MeshGenerator Implementation
// ============================================================================

Mesh MeshGenerator::tessellate(const Surface& surface, int uSamples, int vSamples) {
    Mesh mesh;
    
    if (uSamples < 2) uSamples = 2;
    if (vSamples < 2) vSamples = 2;
    
    double uMin, uMax, vMin, vMax;
    surface.getParameterRange(uMin, uMax, vMin, vMax);
    
    // Generate vertices
    for (int i = 0; i < uSamples; ++i) {
        double u = uMin + (uMax - uMin) * i / (uSamples - 1);
        
        for (int j = 0; j < vSamples; ++j) {
            double v = vMin + (vMax - vMin) * j / (vSamples - 1);
            
            Point3D point = surface.pointAt(u, v);
            Vector3D normal = surface.normalAt(u, v);
            
            mesh.addVertex(point, normal);
        }
    }
    
    // Generate triangles
    for (int i = 0; i < uSamples - 1; ++i) {
        for (int j = 0; j < vSamples - 1; ++j) {
            uint32_t idx0 = i * vSamples + j;
            uint32_t idx1 = i * vSamples + (j + 1);
            uint32_t idx2 = (i + 1) * vSamples + (j + 1);
            uint32_t idx3 = (i + 1) * vSamples + j;
            
            // Two triangles per quad
            mesh.addTriangle(idx0, idx1, idx2);
            mesh.addTriangle(idx0, idx2, idx3);
        }
    }
    
    return mesh;
}

Mesh MeshGenerator::generateLine(const Line& line, bool asLineSegment) {
    Mesh mesh;
    
    if (asLineSegment) {
        // Simple line - just two vertices for GL_LINES rendering
        Vector3D dir = line.direction().normalized();
        mesh.addVertex(line.start, dir);
        mesh.addVertex(line.end, dir);
        // No indices for line segments
    } else {
        // Could generate a thin cylinder here
        // For now, just return line segment
        Vector3D dir = line.direction().normalized();
        mesh.addVertex(line.start, dir);
        mesh.addVertex(line.end, dir);
    }
    
    return mesh;
}

Mesh MeshGenerator::generateCurve(const Curve& curve, int samples) {
    Mesh mesh;
    
    if (samples < 2) samples = 2;
    
    double tMin, tMax;
    curve.getParameterRange(tMin, tMax);
    
    for (int i = 0; i < samples; ++i) {
        double t = tMin + (tMax - tMin) * i / (samples - 1);
        Point3D point = curve.pointAt(t);
        Vector3D tangent = curve.tangentAt(t);
        
        mesh.addVertex(point, tangent);
    }
    
    return mesh;
}

Mesh MeshGenerator::generateCircle(const Circle& circle, int segments, bool filled) {
    Mesh mesh;
    
    if (segments < 3) segments = 3;
    
    const Plane& plane = circle.getPlane();
    double radius = circle.getRadius();
    
    if (filled) {
        // Add center vertex
        mesh.addVertex(circle.getCenter(), circle.getNormal());
        
        // Add perimeter vertices
        for (int i = 0; i < segments; ++i) {
            double angle = 2.0 * M_PI * i / segments;
            Point3D point = circle.pointAtAngle(angle);
            mesh.addVertex(point, circle.getNormal());
        }
        
        // Add triangles from center
        for (int i = 0; i < segments; ++i) {
            uint32_t next = (i + 1) % segments;
            mesh.addTriangle(0, i + 1, next + 1);
        }
    } else {
        // Just the circle outline
        for (int i = 0; i < segments; ++i) {
            double angle = 2.0 * M_PI * i / segments;
            Point3D point = circle.pointAtAngle(angle);
            Vector3D tangent = circle.tangentAtAngle(angle);
            mesh.addVertex(point, tangent);
        }
    }
    
    return mesh;
}

Mesh MeshGenerator::generateArc(const Arc& arc, int segments) {
    Mesh mesh;
    
    if (segments < 2) segments = 2;
    
    for (int i = 0; i < segments; ++i) {
        double t = static_cast<double>(i) / (segments - 1);
        Point3D point = arc.pointAt(t);
        Vector3D tangent = arc.tangentAt(t);
        mesh.addVertex(point, tangent);
    }
    
    return mesh;
}

Mesh MeshGenerator::generateSphere(const Point3D& center, double radius, 
                                   int stacks, int slices) {
    Mesh mesh;
    
    // Generate vertices
    for (int i = 0; i <= stacks; ++i) {
        double phi = M_PI * i / stacks;
        double sinPhi = std::sin(phi);
        double cosPhi = std::cos(phi);
        
        for (int j = 0; j <= slices; ++j) {
            double theta = 2.0 * M_PI * j / slices;
            double sinTheta = std::sin(theta);
            double cosTheta = std::cos(theta);
            
            Vector3D normal(sinPhi * cosTheta, sinPhi * sinTheta, cosPhi);
            Point3D point(
                center.x + radius * normal.x,
                center.y + radius * normal.y,
                center.z + radius * normal.z
            );
            
            mesh.addVertex(point, normal);
        }
    }
    
    // Generate triangles
    for (int i = 0; i < stacks; ++i) {
        for (int j = 0; j < slices; ++j) {
            uint32_t first = i * (slices + 1) + j;
            uint32_t second = first + slices + 1;
            
            mesh.addTriangle(first, second, first + 1);
            mesh.addTriangle(second, second + 1, first + 1);
        }
    }
    
    return mesh;
}

Mesh MeshGenerator::generatePlaneGrid(const Plane& plane, double size, int divisions) {
    Mesh mesh;
    
    double step = size / divisions;
    double halfSize = size / 2.0;
    
    // Generate grid lines
    for (int i = 0; i <= divisions; ++i) {
        double offset = -halfSize + i * step;
        
        // Lines parallel to X axis
        Point3D start1 = plane.planeToWorld(-halfSize, offset);
        Point3D end1 = plane.planeToWorld(halfSize, offset);
        mesh.addVertex(start1, plane.zAxis);
        mesh.addVertex(end1, plane.zAxis);
        
        // Lines parallel to Y axis
        Point3D start2 = plane.planeToWorld(offset, -halfSize);
        Point3D end2 = plane.planeToWorld(offset, halfSize);
        mesh.addVertex(start2, plane.zAxis);
        mesh.addVertex(end2, plane.zAxis);
    }
    
    return mesh;
}

Mesh MeshGenerator::generateAxes(double length) {
    Mesh mesh;
    
    Point3D origin(0, 0, 0);
    
    // X axis (red)
    mesh.addVertex(origin, Vector3D(1, 0, 0));
    mesh.addVertex(Point3D(length, 0, 0), Vector3D(1, 0, 0));
    
    // Y axis (green)
    mesh.addVertex(origin, Vector3D(0, 1, 0));
    mesh.addVertex(Point3D(0, length, 0), Vector3D(0, 1, 0));
    
    // Z axis (blue)
    mesh.addVertex(origin, Vector3D(0, 0, 1));
    mesh.addVertex(Point3D(0, 0, length), Vector3D(0, 0, 1));
    
    return mesh;
}

} // namespace CADLib
