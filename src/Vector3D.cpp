#include "Vector3D.h"

namespace CADLib {

// Constructors
Vector3D::Vector3D() : x(0.0), y(0.0), z(0.0) {}

Vector3D::Vector3D(double x, double y, double z) : x(x), y(y), z(z) {}

Vector3D::Vector3D(const Vector3D& other) : x(other.x), y(other.y), z(other.z) {}

Vector3D::Vector3D(const Point3D& from, const Point3D& to) 
    : x(to.x - from.x), y(to.y - from.y), z(to.z - from.z) {}

// Assignment operator
Vector3D& Vector3D::operator=(const Vector3D& other) {
    if (this != &other) {
        x = other.x;
        y = other.y;
        z = other.z;
    }
    return *this;
}

// Arithmetic operators
Vector3D Vector3D::operator+(const Vector3D& other) const {
    return Vector3D(x + other.x, y + other.y, z + other.z);
}

Vector3D Vector3D::operator-(const Vector3D& other) const {
    return Vector3D(x - other.x, y - other.y, z - other.z);
}

Vector3D Vector3D::operator*(double scalar) const {
    return Vector3D(x * scalar, y * scalar, z * scalar);
}

Vector3D Vector3D::operator/(double scalar) const {
    return Vector3D(x / scalar, y / scalar, z / scalar);
}

Vector3D Vector3D::operator-() const {
    return Vector3D(-x, -y, -z);
}

// Compound assignment operators
Vector3D& Vector3D::operator+=(const Vector3D& other) {
    x += other.x;
    y += other.y;
    z += other.z;
    return *this;
}

Vector3D& Vector3D::operator-=(const Vector3D& other) {
    x -= other.x;
    y -= other.y;
    z -= other.z;
    return *this;
}

Vector3D& Vector3D::operator*=(double scalar) {
    x *= scalar;
    y *= scalar;
    z *= scalar;
    return *this;
}

Vector3D& Vector3D::operator/=(double scalar) {
    x /= scalar;
    y /= scalar;
    z /= scalar;
    return *this;
}

// Comparison operators
bool Vector3D::operator==(const Vector3D& other) const {
    return std::abs(x - other.x) < EPSILON &&
           std::abs(y - other.y) < EPSILON &&
           std::abs(z - other.z) < EPSILON;
}

bool Vector3D::operator!=(const Vector3D& other) const {
    return !(*this == other);
}

// Vector operations
double Vector3D::dot(const Vector3D& other) const {
    return x * other.x + y * other.y + z * other.z;
}

Vector3D Vector3D::cross(const Vector3D& other) const {
    return Vector3D(
        y * other.z - z * other.y,
        z * other.x - x * other.z,
        x * other.y - y * other.x
    );
}

double Vector3D::length() const {
    return std::sqrt(x * x + y * y + z * z);
}

double Vector3D::lengthSquared() const {
    return x * x + y * y + z * z;
}

Vector3D Vector3D::normalized() const {
    double len = length();
    if (len < EPSILON) {
        return Vector3D();
    }
    return Vector3D(x / len, y / len, z / len);
}

void Vector3D::normalize() {
    double len = length();
    if (len > EPSILON) {
        x /= len;
        y /= len;
        z /= len;
    }
}

// Angle calculations
double Vector3D::angleTo(const Vector3D& other) const {
    double lenProduct = length() * other.length();
    if (lenProduct < EPSILON) {
        return 0.0;
    }
    double cosAngle = dot(other) / lenProduct;
    // Clamp to [-1, 1] to avoid numerical issues with acos
    cosAngle = std::max(-1.0, std::min(1.0, cosAngle));
    return std::acos(cosAngle);
}

double Vector3D::angleToInDegrees(const Vector3D& other) const {
    return angleTo(other) * 180.0 / M_PI;
}

// Projection and reflection
Vector3D Vector3D::projectOnto(const Vector3D& other) const {
    double otherLengthSq = other.lengthSquared();
    if (otherLengthSq < EPSILON) {
        return Vector3D();
    }
    double scalar = dot(other) / otherLengthSq;
    return other * scalar;
}

Vector3D Vector3D::reflect(const Vector3D& normal) const {
    return *this - normal * (2.0 * dot(normal));
}

// Utility methods
void Vector3D::set(double x, double y, double z) {
    this->x = x;
    this->y = y;
    this->z = z;
}

bool Vector3D::isZero() const {
    return std::abs(x) < EPSILON && std::abs(y) < EPSILON && std::abs(z) < EPSILON;
}

bool Vector3D::isUnit() const {
    return std::abs(lengthSquared() - 1.0) < EPSILON;
}

Point3D Vector3D::toPoint() const {
    return Point3D(x, y, z);
}

// Static utility vectors
Vector3D Vector3D::zero() {
    return Vector3D(0.0, 0.0, 0.0);
}

Vector3D Vector3D::unitX() {
    return Vector3D(1.0, 0.0, 0.0);
}

Vector3D Vector3D::unitY() {
    return Vector3D(0.0, 1.0, 0.0);
}

Vector3D Vector3D::unitZ() {
    return Vector3D(0.0, 0.0, 1.0);
}

// Stream output
std::ostream& operator<<(std::ostream& os, const Vector3D& vec) {
    os << "Vector3D(" << vec.x << ", " << vec.y << ", " << vec.z << ")";
    return os;
}

// Non-member operator
Vector3D operator*(double scalar, const Vector3D& vec) {
    return vec * scalar;
}

} // namespace CADLib
