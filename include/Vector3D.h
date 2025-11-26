#ifndef VECTOR3D_H
#define VECTOR3D_H

#include "Point3D.h"
#include <cmath>
#include <iostream>

namespace CADLib {

/**
 * @brief Represents a vector in 3D space
 * 
 * This class represents a vector with x, y, z components.
 * Used for directions, normals, and mathematical operations.
 */
class Vector3D {
public:
    double x, y, z;
    
    // Constructors
    Vector3D();
    Vector3D(double x, double y, double z);
    Vector3D(const Vector3D& other);
    Vector3D(const Point3D& from, const Point3D& to);
    
    // Assignment operator
    Vector3D& operator=(const Vector3D& other);
    
    // Arithmetic operators
    Vector3D operator+(const Vector3D& other) const;
    Vector3D operator-(const Vector3D& other) const;
    Vector3D operator*(double scalar) const;
    Vector3D operator/(double scalar) const;
    Vector3D operator-() const; // Unary negation
    
    // Compound assignment operators
    Vector3D& operator+=(const Vector3D& other);
    Vector3D& operator-=(const Vector3D& other);
    Vector3D& operator*=(double scalar);
    Vector3D& operator/=(double scalar);
    
    // Comparison operators
    bool operator==(const Vector3D& other) const;
    bool operator!=(const Vector3D& other) const;
    
    // Vector operations
    double dot(const Vector3D& other) const;
    Vector3D cross(const Vector3D& other) const;
    double length() const;
    double lengthSquared() const;
    Vector3D normalized() const;
    void normalize();
    
    // Angle calculations
    double angleTo(const Vector3D& other) const; // Returns angle in radians
    double angleToInDegrees(const Vector3D& other) const;
    
    // Projection and reflection
    Vector3D projectOnto(const Vector3D& other) const;
    Vector3D reflect(const Vector3D& normal) const;
    
    // Utility methods
    void set(double x, double y, double z);
    bool isZero() const;
    bool isUnit() const;
    Point3D toPoint() const;
    
    // Static utility vectors
    static Vector3D zero();
    static Vector3D unitX();
    static Vector3D unitY();
    static Vector3D unitZ();
    
    // Stream output
    friend std::ostream& operator<<(std::ostream& os, const Vector3D& vec);
    
private:
    static constexpr double EPSILON = 1e-9;
};

// Non-member operator for scalar * Vector3D
Vector3D operator*(double scalar, const Vector3D& vec);

} // namespace CADLib

#endif // VECTOR3D_H
