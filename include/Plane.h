#ifndef PLANE_H
#define PLANE_H

#include "Point3D.h"
#include "Vector3D.h"
#include "Matrix4x4.h"
#include <iostream>

namespace CADLib {

/**
 * @brief Represents an infinite construction plane in 3D space
 * 
 * A lightweight, memory-efficient plane defined by an origin point and a normal vector.
 * Unlike PlanarSurface, this represents an infinite geometric plane for construction
 * and intersection operations (similar to Grasshopper's Plane component).
 * 
 * The plane also stores X and Y axis vectors to define a complete coordinate system.
 */
class Plane {
public:
    Point3D origin;
    Vector3D xAxis;
    Vector3D yAxis;
    Vector3D zAxis;  // Normal to the plane
    
    // Constructors
    Plane();
    Plane(const Point3D& origin, const Vector3D& normal);
    Plane(const Point3D& origin, const Vector3D& xAxis, const Vector3D& yAxis);
    Plane(const Point3D& p1, const Point3D& p2, const Point3D& p3);  // From 3 points
    
    // Standard planes
    static Plane worldXY();
    static Plane worldYZ();
    static Plane worldZX();
    
    // Plane equation: ax + by + cz + d = 0
    void getEquation(double& a, double& b, double& c, double& d) const;
    
    // Point operations
    double distanceToPoint(const Point3D& point) const;
    Point3D closestPoint(const Point3D& point) const;
    bool isPointOnPlane(const Point3D& point, double tolerance = 1e-9) const;
    
    // Project point onto plane
    Point3D projectPoint(const Point3D& point) const;
    
    // Get local coordinates (in plane's XY system)
    void worldToPlane(const Point3D& worldPoint, double& u, double& v) const;
    Point3D planeToWorld(double u, double v) const;
    
    // Evaluate point on plane using local coordinates
    Point3D pointAt(double u, double v) const;
    
    // Transform the plane
    void transform(const Matrix4x4& matrix);
    Plane transformed(const Matrix4x4& matrix) const;
    
    // Flip the plane (reverse normal)
    void flip();
    Plane flipped() const;
    
    // Check if plane is valid
    bool isValid() const;
    
    // Stream output
    friend std::ostream& operator<<(std::ostream& os, const Plane& plane);
    
private:
    void updateAxes();  // Ensure axes are orthonormal
    static constexpr double EPSILON = 1e-9;
};

} // namespace CADLib

#endif // PLANE_H
