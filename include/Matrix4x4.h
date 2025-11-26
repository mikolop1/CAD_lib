#ifndef MATRIX4X4_H
#define MATRIX4X4_H

#include "Point3D.h"
#include "Vector3D.h"
#include <iostream>
#include <array>

namespace CADLib {

/**
 * @brief Represents a 4x4 transformation matrix
 * 
 * This class is used for 3D transformations including translation,
 * rotation, scaling, and their combinations. Uses column-major order
 * for compatibility with OpenGL and Qt.
 */
class Matrix4x4 {
public:
    // Data stored in column-major order
    std::array<double, 16> data;
    
    // Constructors
    Matrix4x4();
    Matrix4x4(const Matrix4x4& other);
    Matrix4x4(const std::array<double, 16>& values);
    
    // Assignment operator
    Matrix4x4& operator=(const Matrix4x4& other);
    
    // Element access (row, col)
    double& operator()(int row, int col);
    const double& operator()(int row, int col) const;
    
    // Matrix operations
    Matrix4x4 operator*(const Matrix4x4& other) const;
    Matrix4x4& operator*=(const Matrix4x4& other);
    Matrix4x4 operator+(const Matrix4x4& other) const;
    Matrix4x4 operator-(const Matrix4x4& other) const;
    Matrix4x4 operator*(double scalar) const;
    
    // Comparison operators
    bool operator==(const Matrix4x4& other) const;
    bool operator!=(const Matrix4x4& other) const;
    
    // Transform points and vectors
    Point3D transform(const Point3D& point) const;
    Vector3D transform(const Vector3D& vec, bool isDirection = true) const;
    
    // Matrix operations
    Matrix4x4 transposed() const;
    Matrix4x4 inverted() const;
    double determinant() const;
    
    // Static factory methods for transformations
    static Matrix4x4 identity();
    static Matrix4x4 translation(double x, double y, double z);
    static Matrix4x4 translation(const Vector3D& vec);
    static Matrix4x4 scale(double x, double y, double z);
    static Matrix4x4 scale(double uniform);
    static Matrix4x4 rotationX(double angle); // angle in radians
    static Matrix4x4 rotationY(double angle);
    static Matrix4x4 rotationZ(double angle);
    static Matrix4x4 rotation(const Vector3D& axis, double angle);
    
    // Utility methods
    void setIdentity();
    bool isIdentity() const;
    
    // Stream output
    friend std::ostream& operator<<(std::ostream& os, const Matrix4x4& mat);
    
private:
    static constexpr double EPSILON = 1e-9;
    
    // Helper for matrix inversion
    Matrix4x4 cofactor() const;
};

// Non-member operator
Matrix4x4 operator*(double scalar, const Matrix4x4& mat);

} // namespace CADLib

#endif // MATRIX4X4_H
