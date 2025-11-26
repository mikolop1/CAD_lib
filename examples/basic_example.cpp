#include "Point3D.h"
#include "Vector3D.h"
#include "Matrix4x4.h"
#include "Geometry.h"
#include <iostream>

using namespace CADLib;

int main() {
    std::cout << "=== CADLib Basic Example ===" << std::endl << std::endl;
    
    // ========================================
    // Working with Points
    // ========================================
    std::cout << "1. Point Operations:" << std::endl;
    Point3D p1(1.0, 2.0, 3.0);
    Point3D p2(4.0, 5.0, 6.0);
    
    std::cout << "Point 1: " << p1 << std::endl;
    std::cout << "Point 2: " << p2 << std::endl;
    std::cout << "Distance: " << p1.distanceTo(p2) << std::endl;
    std::cout << "Midpoint: " << p1.midpoint(p2) << std::endl;
    std::cout << std::endl;
    
    // ========================================
    // Working with Vectors
    // ========================================
    std::cout << "2. Vector Operations:" << std::endl;
    Vector3D v1(1.0, 0.0, 0.0);
    Vector3D v2(0.0, 1.0, 0.0);
    
    std::cout << "Vector 1: " << v1 << std::endl;
    std::cout << "Vector 2: " << v2 << std::endl;
    std::cout << "Dot product: " << v1.dot(v2) << std::endl;
    std::cout << "Cross product: " << v1.cross(v2) << std::endl;
    std::cout << "Angle (degrees): " << v1.angleToInDegrees(v2) << std::endl;
    std::cout << std::endl;
    
    // ========================================
    // Working with Lines
    // ========================================
    std::cout << "3. Line Operations:" << std::endl;
    Line line(Point3D(0, 0, 0), Point3D(10, 0, 0));
    
    std::cout << "Line from " << line.start << " to " << line.end << std::endl;
    std::cout << "Length: " << line.length() << std::endl;
    std::cout << "Direction: " << line.direction() << std::endl;
    std::cout << "Point at t=0.5: " << line.pointAt(0.5) << std::endl;
    
    Point3D testPoint(5.0, 3.0, 0.0);
    std::cout << "Distance from " << testPoint << " to line: " 
              << line.distanceToPoint(testPoint) << std::endl;
    std::cout << std::endl;
    
    // ========================================
    // Working with Transformations
    // ========================================
    std::cout << "4. Transformation Operations:" << std::endl;
    Point3D original(1.0, 0.0, 0.0);
    std::cout << "Original point: " << original << std::endl;
    
    // Translation
    Matrix4x4 translation = Matrix4x4::translation(5.0, 3.0, 2.0);
    Point3D translated = translation.transform(original);
    std::cout << "After translation (5,3,2): " << translated << std::endl;
    
    // Rotation around Z axis (90 degrees)
    Matrix4x4 rotation = Matrix4x4::rotationZ(M_PI / 2);
    Point3D rotated = rotation.transform(original);
    std::cout << "After 90° rotation around Z: " << rotated << std::endl;
    
    // Scaling
    Matrix4x4 scale = Matrix4x4::scale(2.0);
    Point3D scaled = scale.transform(original);
    std::cout << "After uniform scale by 2: " << scaled << std::endl;
    
    // Combined transformation
    Matrix4x4 combined = translation * rotation * scale;
    Point3D transformed = combined.transform(original);
    std::cout << "After combined transformation: " << transformed << std::endl;
    std::cout << std::endl;
    
    std::cout << "Example completed successfully!" << std::endl;
    
    return 0;
}
