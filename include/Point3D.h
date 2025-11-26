#ifndef POINT3D_H
#define POINT3D_H

#include <cmath>
#include <iostream>

namespace CADLib {

/**
 * @brief Represents a point in 3D space
 * 
 * This class represents a point with x, y, z coordinates in 3D space.
 * Compatible with Qt and C++ for visualization purposes.
 */
class Point3D {
public:
    double x, y, z;
    
    // Constructors
    Point3D();
    Point3D(double x, double y, double z);
    Point3D(const Point3D& other);
    
    // Assignment operator
    Point3D& operator=(const Point3D& other);
    
    // Arithmetic operators
    Point3D operator+(const Point3D& other) const;
    Point3D operator-(const Point3D& other) const;
    Point3D operator*(double scalar) const;
    Point3D operator/(double scalar) const;
    
    // Compound assignment operators
    Point3D& operator+=(const Point3D& other);
    Point3D& operator-=(const Point3D& other);
    Point3D& operator*=(double scalar);
    Point3D& operator/=(double scalar);
    
    // Comparison operators
    bool operator==(const Point3D& other) const;
    bool operator!=(const Point3D& other) const;
    
    // Distance calculations
    double distanceTo(const Point3D& other) const;
    double distanceSquaredTo(const Point3D& other) const;
    double distanceFromOrigin() const;
    
    // Utility methods
    void set(double x, double y, double z);
    Point3D midpoint(const Point3D& other) const;
    
    // Stream output
    friend std::ostream& operator<<(std::ostream& os, const Point3D& point);
    
private:
    static constexpr double EPSILON = 1e-9;
};

// Non-member operator for scalar * Point3D
Point3D operator*(double scalar, const Point3D& point);

} // namespace CADLib

#endif // POINT3D_H
