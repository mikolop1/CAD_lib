#include "Matrix4x4.h"
#include <cmath>
#include <stdexcept>

namespace CADLib {

// Constructors
Matrix4x4::Matrix4x4() {
    setIdentity();
}

Matrix4x4::Matrix4x4(const Matrix4x4& other) : data(other.data) {}

Matrix4x4::Matrix4x4(const std::array<double, 16>& values) : data(values) {}

// Assignment operator
Matrix4x4& Matrix4x4::operator=(const Matrix4x4& other) {
    if (this != &other) {
        data = other.data;
    }
    return *this;
}

// Element access (row, col) - column-major order
double& Matrix4x4::operator()(int row, int col) {
    return data[col * 4 + row];
}

const double& Matrix4x4::operator()(int row, int col) const {
    return data[col * 4 + row];
}

// Matrix operations
Matrix4x4 Matrix4x4::operator*(const Matrix4x4& other) const {
    Matrix4x4 result;
    for (int row = 0; row < 4; ++row) {
        for (int col = 0; col < 4; ++col) {
            double sum = 0.0;
            for (int k = 0; k < 4; ++k) {
                sum += (*this)(row, k) * other(k, col);
            }
            result(row, col) = sum;
        }
    }
    return result;
}

Matrix4x4& Matrix4x4::operator*=(const Matrix4x4& other) {
    *this = *this * other;
    return *this;
}

Matrix4x4 Matrix4x4::operator+(const Matrix4x4& other) const {
    Matrix4x4 result;
    for (int i = 0; i < 16; ++i) {
        result.data[i] = data[i] + other.data[i];
    }
    return result;
}

Matrix4x4 Matrix4x4::operator-(const Matrix4x4& other) const {
    Matrix4x4 result;
    for (int i = 0; i < 16; ++i) {
        result.data[i] = data[i] - other.data[i];
    }
    return result;
}

Matrix4x4 Matrix4x4::operator*(double scalar) const {
    Matrix4x4 result;
    for (int i = 0; i < 16; ++i) {
        result.data[i] = data[i] * scalar;
    }
    return result;
}

// Comparison operators
bool Matrix4x4::operator==(const Matrix4x4& other) const {
    for (int i = 0; i < 16; ++i) {
        if (std::abs(data[i] - other.data[i]) > EPSILON) {
            return false;
        }
    }
    return true;
}

bool Matrix4x4::operator!=(const Matrix4x4& other) const {
    return !(*this == other);
}

// Transform points and vectors
Point3D Matrix4x4::transform(const Point3D& point) const {
    double x = (*this)(0, 0) * point.x + (*this)(0, 1) * point.y + 
               (*this)(0, 2) * point.z + (*this)(0, 3);
    double y = (*this)(1, 0) * point.x + (*this)(1, 1) * point.y + 
               (*this)(1, 2) * point.z + (*this)(1, 3);
    double z = (*this)(2, 0) * point.x + (*this)(2, 1) * point.y + 
               (*this)(2, 2) * point.z + (*this)(2, 3);
    double w = (*this)(3, 0) * point.x + (*this)(3, 1) * point.y + 
               (*this)(3, 2) * point.z + (*this)(3, 3);
    
    if (std::abs(w) > EPSILON) {
        return Point3D(x / w, y / w, z / w);
    }
    return Point3D(x, y, z);
}

Vector3D Matrix4x4::transform(const Vector3D& vec, bool isDirection) const {
    double w = isDirection ? 0.0 : 1.0;
    
    double x = (*this)(0, 0) * vec.x + (*this)(0, 1) * vec.y + 
               (*this)(0, 2) * vec.z + (*this)(0, 3) * w;
    double y = (*this)(1, 0) * vec.x + (*this)(1, 1) * vec.y + 
               (*this)(1, 2) * vec.z + (*this)(1, 3) * w;
    double z = (*this)(2, 0) * vec.x + (*this)(2, 1) * vec.y + 
               (*this)(2, 2) * vec.z + (*this)(2, 3) * w;
    
    return Vector3D(x, y, z);
}

// Matrix operations
Matrix4x4 Matrix4x4::transposed() const {
    Matrix4x4 result;
    for (int row = 0; row < 4; ++row) {
        for (int col = 0; col < 4; ++col) {
            result(row, col) = (*this)(col, row);
        }
    }
    return result;
}

double Matrix4x4::determinant() const {
    double det = 0.0;
    
    // Calculate determinant using cofactor expansion on first row
    for (int col = 0; col < 4; ++col) {
        // Create 3x3 minor matrix
        double minor[9];
        int minorIdx = 0;
        
        for (int i = 1; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                if (j != col) {
                    minor[minorIdx++] = (*this)(i, j);
                }
            }
        }
        
        // Calculate 3x3 determinant
        double minor_det = minor[0] * (minor[4] * minor[8] - minor[5] * minor[7])
                         - minor[1] * (minor[3] * minor[8] - minor[5] * minor[6])
                         + minor[2] * (minor[3] * minor[7] - minor[4] * minor[6]);
        
        // Add to determinant with alternating signs
        det += (col % 2 == 0 ? 1 : -1) * (*this)(0, col) * minor_det;
    }
    
    return det;
}

Matrix4x4 Matrix4x4::inverted() const {
    double det = determinant();
    
    if (std::abs(det) < EPSILON) {
        throw std::runtime_error("Matrix is not invertible (determinant is zero)");
    }
    
    Matrix4x4 result;
    
    // Calculate cofactor matrix
    for (int row = 0; row < 4; ++row) {
        for (int col = 0; col < 4; ++col) {
            // Create 3x3 minor matrix
            double minor[9];
            int minorIdx = 0;
            
            for (int i = 0; i < 4; ++i) {
                if (i == row) continue;
                for (int j = 0; j < 4; ++j) {
                    if (j == col) continue;
                    minor[minorIdx++] = (*this)(i, j);
                }
            }
            
            // Calculate 3x3 determinant
            double minor_det = minor[0] * (minor[4] * minor[8] - minor[5] * minor[7])
                             - minor[1] * (minor[3] * minor[8] - minor[5] * minor[6])
                             + minor[2] * (minor[3] * minor[7] - minor[4] * minor[6]);
            
            // Calculate cofactor with alternating signs and transpose
            result(col, row) = ((row + col) % 2 == 0 ? 1 : -1) * minor_det / det;
        }
    }
    
    return result;
}

// Static factory methods
Matrix4x4 Matrix4x4::identity() {
    return Matrix4x4();
}

Matrix4x4 Matrix4x4::translation(double x, double y, double z) {
    Matrix4x4 result;
    result(0, 3) = x;
    result(1, 3) = y;
    result(2, 3) = z;
    return result;
}

Matrix4x4 Matrix4x4::translation(const Vector3D& vec) {
    return translation(vec.x, vec.y, vec.z);
}

Matrix4x4 Matrix4x4::scale(double x, double y, double z) {
    Matrix4x4 result;
    result(0, 0) = x;
    result(1, 1) = y;
    result(2, 2) = z;
    return result;
}

Matrix4x4 Matrix4x4::scale(double uniform) {
    return scale(uniform, uniform, uniform);
}

Matrix4x4 Matrix4x4::rotationX(double angle) {
    Matrix4x4 result;
    double c = std::cos(angle);
    double s = std::sin(angle);
    
    result(1, 1) = c;
    result(1, 2) = -s;
    result(2, 1) = s;
    result(2, 2) = c;
    
    return result;
}

Matrix4x4 Matrix4x4::rotationY(double angle) {
    Matrix4x4 result;
    double c = std::cos(angle);
    double s = std::sin(angle);
    
    result(0, 0) = c;
    result(0, 2) = s;
    result(2, 0) = -s;
    result(2, 2) = c;
    
    return result;
}

Matrix4x4 Matrix4x4::rotationZ(double angle) {
    Matrix4x4 result;
    double c = std::cos(angle);
    double s = std::sin(angle);
    
    result(0, 0) = c;
    result(0, 1) = -s;
    result(1, 0) = s;
    result(1, 1) = c;
    
    return result;
}

Matrix4x4 Matrix4x4::rotation(const Vector3D& axis, double angle) {
    Vector3D a = axis.normalized();
    double c = std::cos(angle);
    double s = std::sin(angle);
    double t = 1.0 - c;
    
    Matrix4x4 result;
    
    result(0, 0) = t * a.x * a.x + c;
    result(0, 1) = t * a.x * a.y - s * a.z;
    result(0, 2) = t * a.x * a.z + s * a.y;
    
    result(1, 0) = t * a.x * a.y + s * a.z;
    result(1, 1) = t * a.y * a.y + c;
    result(1, 2) = t * a.y * a.z - s * a.x;
    
    result(2, 0) = t * a.x * a.z - s * a.y;
    result(2, 1) = t * a.y * a.z + s * a.x;
    result(2, 2) = t * a.z * a.z + c;
    
    return result;
}

// Utility methods
void Matrix4x4::setIdentity() {
    data.fill(0.0);
    data[0] = data[5] = data[10] = data[15] = 1.0;
}

bool Matrix4x4::isIdentity() const {
    Matrix4x4 id = identity();
    return *this == id;
}

// Stream output
std::ostream& operator<<(std::ostream& os, const Matrix4x4& mat) {
    os << "Matrix4x4(\n";
    for (int row = 0; row < 4; ++row) {
        os << "  [";
        for (int col = 0; col < 4; ++col) {
            os << mat(row, col);
            if (col < 3) os << ", ";
        }
        os << "]";
        if (row < 3) os << ",";
        os << "\n";
    }
    os << ")";
    return os;
}

// Non-member operator
Matrix4x4 operator*(double scalar, const Matrix4x4& mat) {
    return mat * scalar;
}

} // namespace CADLib
