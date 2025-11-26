#include "Plane.h"
#include <cmath>

namespace CADLib {

// Constructors
Plane::Plane() 
    : origin(0, 0, 0), xAxis(1, 0, 0), yAxis(0, 1, 0), zAxis(0, 0, 1) {}

Plane::Plane(const Point3D& origin, const Vector3D& normal) 
    : origin(origin), zAxis(normal.normalized()) {
    // Create an arbitrary orthonormal basis
    Vector3D arbitrary;
    if (std::abs(zAxis.x) < 0.9) {
        arbitrary = Vector3D(1, 0, 0);
    } else {
        arbitrary = Vector3D(0, 1, 0);
    }
    
    xAxis = arbitrary.cross(zAxis).normalized();
    yAxis = zAxis.cross(xAxis).normalized();
}

Plane::Plane(const Point3D& origin, const Vector3D& xAxis, const Vector3D& yAxis)
    : origin(origin), xAxis(xAxis.normalized()), yAxis(yAxis.normalized()) {
    zAxis = xAxis.cross(yAxis).normalized();
    // Ensure Y axis is perpendicular to X
    this->yAxis = zAxis.cross(this->xAxis).normalized();
}

Plane::Plane(const Point3D& p1, const Point3D& p2, const Point3D& p3) {
    origin = p1;
    Vector3D v1(p1, p2);
    Vector3D v2(p1, p3);
    zAxis = v1.cross(v2).normalized();
    xAxis = v1.normalized();
    yAxis = zAxis.cross(xAxis).normalized();
}

// Standard planes
Plane Plane::worldXY() {
    return Plane(Point3D(0, 0, 0), Vector3D(1, 0, 0), Vector3D(0, 1, 0));
}

Plane Plane::worldYZ() {
    return Plane(Point3D(0, 0, 0), Vector3D(0, 1, 0), Vector3D(0, 0, 1));
}

Plane Plane::worldZX() {
    return Plane(Point3D(0, 0, 0), Vector3D(0, 0, 1), Vector3D(1, 0, 0));
}

// Plane equation: ax + by + cz + d = 0
void Plane::getEquation(double& a, double& b, double& c, double& d) const {
    a = zAxis.x;
    b = zAxis.y;
    c = zAxis.z;
    d = -(a * origin.x + b * origin.y + c * origin.z);
}

// Point operations
double Plane::distanceToPoint(const Point3D& point) const {
    Vector3D toPoint(origin, point);
    return std::abs(toPoint.dot(zAxis));
}

Point3D Plane::closestPoint(const Point3D& point) const {
    return projectPoint(point);
}

bool Plane::isPointOnPlane(const Point3D& point, double tolerance) const {
    return distanceToPoint(point) < tolerance;
}

Point3D Plane::projectPoint(const Point3D& point) const {
    Vector3D toPoint(origin, point);
    double distAlongNormal = toPoint.dot(zAxis);
    return Point3D(
        point.x - distAlongNormal * zAxis.x,
        point.y - distAlongNormal * zAxis.y,
        point.z - distAlongNormal * zAxis.z
    );
}

// Coordinate conversions
void Plane::worldToPlane(const Point3D& worldPoint, double& u, double& v) const {
    Vector3D toPoint(origin, worldPoint);
    u = toPoint.dot(xAxis);
    v = toPoint.dot(yAxis);
}

Point3D Plane::planeToWorld(double u, double v) const {
    return Point3D(
        origin.x + u * xAxis.x + v * yAxis.x,
        origin.y + u * xAxis.y + v * yAxis.y,
        origin.z + u * xAxis.z + v * yAxis.z
    );
}

Point3D Plane::pointAt(double u, double v) const {
    return planeToWorld(u, v);
}

// Transform operations
void Plane::transform(const Matrix4x4& matrix) {
    origin = matrix.transform(origin);
    xAxis = matrix.transform(xAxis, true);
    yAxis = matrix.transform(yAxis, true);
    zAxis = matrix.transform(zAxis, true);
    updateAxes();
}

Plane Plane::transformed(const Matrix4x4& matrix) const {
    Plane result = *this;
    result.transform(matrix);
    return result;
}

// Flip operations
void Plane::flip() {
    zAxis = -zAxis;
    yAxis = -yAxis;
}

Plane Plane::flipped() const {
    Plane result = *this;
    result.flip();
    return result;
}

// Validation
bool Plane::isValid() const {
    return xAxis.length() > EPSILON && 
           yAxis.length() > EPSILON && 
           zAxis.length() > EPSILON;
}

// Update axes to ensure orthonormality
void Plane::updateAxes() {
    xAxis.normalize();
    yAxis.normalize();
    zAxis.normalize();
    
    // Re-orthogonalize using Gram-Schmidt
    yAxis = zAxis.cross(xAxis).normalized();
    xAxis = yAxis.cross(zAxis).normalized();
}

// Stream output
std::ostream& operator<<(std::ostream& os, const Plane& plane) {
    os << "Plane(origin=" << plane.origin 
       << ", normal=" << plane.zAxis << ")";
    return os;
}

} // namespace CADLib
