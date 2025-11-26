#include "Point3D.h"
#include "Vector3D.h"
#include "Matrix4x4.h"
#include "Geometry.h"
#include <iostream>
#include <iomanip>

using namespace CADLib;

void printSeparator() {
    std::cout << "----------------------------------------" << std::endl;
}

int main() {
    std::cout << "=== CADLib Transformations Example ===" << std::endl << std::endl;
    std::cout << std::fixed << std::setprecision(3);
    
    // ========================================
    // Basic Translation
    // ========================================
    std::cout << "1. Translation:" << std::endl;
    Point3D p1(1, 2, 3);
    std::cout << "Original point: " << p1 << std::endl;
    
    Matrix4x4 trans = Matrix4x4::translation(5, 0, 0);
    Point3D translated = trans.transform(p1);
    std::cout << "After translation (5,0,0): " << translated << std::endl;
    printSeparator();
    std::cout << std::endl;
    
    // ========================================
    // Basic Rotation
    // ========================================
    std::cout << "2. Rotation:" << std::endl;
    Point3D p2(1, 0, 0);
    std::cout << "Original point: " << p2 << std::endl;
    
    Matrix4x4 rotZ90 = Matrix4x4::rotationZ(M_PI / 2);
    Point3D rotated = rotZ90.transform(p2);
    std::cout << "After 90° rotation around Z: " << rotated << std::endl;
    
    Matrix4x4 rotY90 = Matrix4x4::rotationY(M_PI / 2);
    rotated = rotY90.transform(p2);
    std::cout << "After 90° rotation around Y: " << rotated << std::endl;
    
    Matrix4x4 rotX90 = Matrix4x4::rotationX(M_PI / 2);
    Point3D p3(0, 1, 0);
    rotated = rotX90.transform(p3);
    std::cout << "Point (0,1,0) after 90° rotation around X: " << rotated << std::endl;
    printSeparator();
    std::cout << std::endl;
    
    // ========================================
    // Arbitrary Axis Rotation
    // ========================================
    std::cout << "3. Rotation around Arbitrary Axis:" << std::endl;
    Vector3D axis(1, 1, 0);
    axis.normalize();
    std::cout << "Rotation axis: " << axis << std::endl;
    
    Point3D p4(1, 0, 0);
    Matrix4x4 rotArbitrary = Matrix4x4::rotation(axis, M_PI / 4);
    rotated = rotArbitrary.transform(p4);
    std::cout << "Point " << p4 << " after 45° rotation: " << rotated << std::endl;
    printSeparator();
    std::cout << std::endl;
    
    // ========================================
    // Scaling
    // ========================================
    std::cout << "4. Scaling:" << std::endl;
    Point3D p5(2, 3, 4);
    std::cout << "Original point: " << p5 << std::endl;
    
    Matrix4x4 scaleUniform = Matrix4x4::scale(2.0);
    Point3D scaled = scaleUniform.transform(p5);
    std::cout << "After uniform scale by 2: " << scaled << std::endl;
    
    Matrix4x4 scaleNonUniform = Matrix4x4::scale(2.0, 0.5, 1.0);
    scaled = scaleNonUniform.transform(p5);
    std::cout << "After non-uniform scale (2, 0.5, 1): " << scaled << std::endl;
    printSeparator();
    std::cout << std::endl;
    
    // ========================================
    // Combined Transformations
    // ========================================
    std::cout << "5. Combined Transformations:" << std::endl;
    Point3D p6(1, 0, 0);
    std::cout << "Original point: " << p6 << std::endl;
    
    // Combine: Scale -> Rotate -> Translate
    Matrix4x4 scale = Matrix4x4::scale(2.0);
    Matrix4x4 rotate = Matrix4x4::rotationZ(M_PI / 4);
    Matrix4x4 translate = Matrix4x4::translation(5, 5, 0);
    
    // Note: Matrix multiplication order is right-to-left
    Matrix4x4 combined = translate * rotate * scale;
    Point3D transformed = combined.transform(p6);
    
    std::cout << "After scale(2) * rotate(45°) * translate(5,5,0): " 
              << transformed << std::endl;
    printSeparator();
    std::cout << std::endl;
    
    // ========================================
    // Matrix Inversion
    // ========================================
    std::cout << "6. Matrix Inversion:" << std::endl;
    Matrix4x4 transform = Matrix4x4::translation(10, 20, 30);
    Point3D original(1, 2, 3);
    
    std::cout << "Original point: " << original << std::endl;
    Point3D transformed2 = transform.transform(original);
    std::cout << "After transformation: " << transformed2 << std::endl;
    
    Matrix4x4 inverse = transform.inverted();
    Point3D restored = inverse.transform(transformed2);
    std::cout << "After inverse transformation: " << restored << std::endl;
    printSeparator();
    std::cout << std::endl;
    
    // ========================================
    // Transforming Geometry Objects
    // ========================================
    std::cout << "7. Transforming Geometry Objects:" << std::endl;
    
    // Create a line
    Line line(Point3D(0, 0, 0), Point3D(5, 0, 0));
    std::cout << "Original line: " << line.start << " to " << line.end << std::endl;
    
    // Rotate and translate the line
    Matrix4x4 lineTransform = Matrix4x4::translation(2, 2, 0) * 
                              Matrix4x4::rotationZ(M_PI / 3);
    line.transform(lineTransform);
    std::cout << "Transformed line: " << line.start << " to " << line.end << std::endl;
    printSeparator();
    std::cout << std::endl;
    
    // ========================================
    // Transforming Bezier Curve
    // ========================================
    std::cout << "8. Transforming Bezier Curve:" << std::endl;
    
    std::vector<Point3D> controlPoints = {
        Point3D(0, 0, 0),
        Point3D(5, 5, 0),
        Point3D(10, 0, 0)
    };
    
    BezierCurve curve(controlPoints);
    std::cout << "Original curve at t=0.5: " << curve.pointAt(0.5) << std::endl;
    
    Matrix4x4 curveTransform = Matrix4x4::rotationZ(M_PI / 2) * 
                                Matrix4x4::scale(2.0);
    curve.transform(curveTransform);
    std::cout << "After scale(2) and rotate(90°) at t=0.5: " 
              << curve.pointAt(0.5) << std::endl;
    printSeparator();
    std::cout << std::endl;
    
    // ========================================
    // Transforming Surface
    // ========================================
    std::cout << "9. Transforming Surface:" << std::endl;
    
    PlanarSurface surface(Point3D(0, 0, 0), Vector3D(1, 0, 0), 
                          Vector3D(0, 1, 0), 10, 10);
    std::cout << "Original surface center: " << surface.pointAt(0.5, 0.5) << std::endl;
    std::cout << "Original normal: " << surface.normalAt(0.5, 0.5) << std::endl;
    
    Matrix4x4 surfaceTransform = Matrix4x4::translation(0, 0, 5) * 
                                  Matrix4x4::rotationX(M_PI / 4);
    surface.transform(surfaceTransform);
    std::cout << "After rotate(45° X) and translate(0,0,5):" << std::endl;
    std::cout << "  Center: " << surface.pointAt(0.5, 0.5) << std::endl;
    std::cout << "  Normal: " << surface.normalAt(0.5, 0.5) << std::endl;
    printSeparator();
    std::cout << std::endl;
    
    // ========================================
    // Chaining Multiple Transformations
    // ========================================
    std::cout << "10. Chaining Multiple Transformations:" << std::endl;
    
    Point3D cube[8] = {
        Point3D(0, 0, 0), Point3D(1, 0, 0), Point3D(1, 1, 0), Point3D(0, 1, 0),
        Point3D(0, 0, 1), Point3D(1, 0, 1), Point3D(1, 1, 1), Point3D(0, 1, 1)
    };
    
    std::cout << "Original cube vertex [0]: " << cube[0] << std::endl;
    std::cout << "Original cube vertex [7]: " << cube[7] << std::endl;
    
    // Build a complex transformation
    Matrix4x4 complex = Matrix4x4::translation(10, 10, 10);
    complex *= Matrix4x4::rotationZ(M_PI / 6);
    complex *= Matrix4x4::rotationY(M_PI / 6);
    complex *= Matrix4x4::scale(3.0);
    complex *= Matrix4x4::translation(-0.5, -0.5, -0.5);  // Center first
    
    for (int i = 0; i < 8; ++i) {
        cube[i] = complex.transform(cube[i]);
    }
    
    std::cout << "Transformed cube vertex [0]: " << cube[0] << std::endl;
    std::cout << "Transformed cube vertex [7]: " << cube[7] << std::endl;
    printSeparator();
    std::cout << std::endl;
    
    std::cout << "Example completed successfully!" << std::endl;
    
    return 0;
}
