#include "Point3D.h"

namespace CADLib {

// Constructors
Point3D::Point3D() : x(0.0), y(0.0), z(0.0) {}

Point3D::Point3D(double x, double y, double z) : x(x), y(y), z(z) {}

Point3D::Point3D(const Point3D& other) : x(other.x), y(other.y), z(other.z) {}

// Assignment operator
Point3D& Point3D::operator=(const Point3D& other) {
    if (this != &other) {
        x = other.x;
        y = other.y;
        z = other.z;
    }
    return *this;
}

// Arithmetic operators
Point3D Point3D::operator+(const Point3D& other) const {
    return Point3D(x + other.x, y + other.y, z + other.z);
}

Point3D Point3D::operator-(const Point3D& other) const {
    return Point3D(x - other.x, y - other.y, z - other.z);
}

Point3D Point3D::operator*(double scalar) const {
    return Point3D(x * scalar, y * scalar, z * scalar);
}

Point3D Point3D::operator/(double scalar) const {
    return Point3D(x / scalar, y / scalar, z / scalar);
}

// Compound assignment operators
Point3D& Point3D::operator+=(const Point3D& other) {
    x += other.x;
    y += other.y;
    z += other.z;
    return *this;
}

Point3D& Point3D::operator-=(const Point3D& other) {
    x -= other.x;
    y -= other.y;
    z -= other.z;
    return *this;
}

Point3D& Point3D::operator*=(double scalar) {
    x *= scalar;
    y *= scalar;
    z *= scalar;
    return *this;
}

Point3D& Point3D::operator/=(double scalar) {
    x /= scalar;
    y /= scalar;
    z /= scalar;
    return *this;
}

// Comparison operators
bool Point3D::operator==(const Point3D& other) const {
    return std::abs(x - other.x) < EPSILON &&
           std::abs(y - other.y) < EPSILON &&
           std::abs(z - other.z) < EPSILON;
}

bool Point3D::operator!=(const Point3D& other) const {
    return !(*this == other);
}

// Distance calculations
double Point3D::distanceTo(const Point3D& other) const {
    double dx = x - other.x;
    double dy = y - other.y;
    double dz = z - other.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double Point3D::distanceSquaredTo(const Point3D& other) const {
    double dx = x - other.x;
    double dy = y - other.y;
    double dz = z - other.z;
    return dx * dx + dy * dy + dz * dz;
}

double Point3D::distanceFromOrigin() const {
    return std::sqrt(x * x + y * y + z * z);
}

// Utility methods
void Point3D::set(double x, double y, double z) {
    this->x = x;
    this->y = y;
    this->z = z;
}

Point3D Point3D::midpoint(const Point3D& other) const {
    return Point3D((x + other.x) / 2.0,
                   (y + other.y) / 2.0,
                   (z + other.z) / 2.0);
}

// Stream output
std::ostream& operator<<(std::ostream& os, const Point3D& point) {
    os << "Point3D(" << point.x << ", " << point.y << ", " << point.z << ")";
    return os;
}

// Non-member operator
Point3D operator*(double scalar, const Point3D& point) {
    return point * scalar;
}

} // namespace CADLib
