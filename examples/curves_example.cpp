#include "Geometry.h"
#include <iostream>
#include <iomanip>

using namespace CADLib;

int main() {
    std::cout << "=== CADLib Curves Example ===" << std::endl << std::endl;
    std::cout << std::fixed << std::setprecision(3);
    
    // ========================================
    // Linear Bezier Curve (2 control points - a line)
    // ========================================
    std::cout << "1. Linear Bezier Curve:" << std::endl;
    std::vector<Point3D> linearPoints = {
        Point3D(0, 0, 0),
        Point3D(10, 0, 0)
    };
    BezierCurve linearCurve(linearPoints);
    
    std::cout << "Control points: " << linearPoints.size() << std::endl;
    std::cout << "Degree: " << linearCurve.degree() << std::endl;
    std::cout << "Point at t=0.0: " << linearCurve.pointAt(0.0) << std::endl;
    std::cout << "Point at t=0.5: " << linearCurve.pointAt(0.5) << std::endl;
    std::cout << "Point at t=1.0: " << linearCurve.pointAt(1.0) << std::endl;
    std::cout << std::endl;
    
    // ========================================
    // Quadratic Bezier Curve (3 control points)
    // ========================================
    std::cout << "2. Quadratic Bezier Curve:" << std::endl;
    std::vector<Point3D> quadraticPoints = {
        Point3D(0, 0, 0),
        Point3D(5, 10, 0),
        Point3D(10, 0, 0)
    };
    BezierCurve quadraticCurve(quadraticPoints);
    
    std::cout << "Control points: " << quadraticPoints.size() << std::endl;
    std::cout << "Degree: " << quadraticCurve.degree() << std::endl;
    
    std::cout << "Sampling curve at t=0.0 to 1.0:" << std::endl;
    for (double t = 0.0; t <= 1.0; t += 0.25) {
        Point3D pt = quadraticCurve.pointAt(t);
        Vector3D tangent = quadraticCurve.tangentAt(t);
        std::cout << "  t=" << t << ": point=" << pt 
                  << ", tangent=" << tangent << std::endl;
    }
    std::cout << std::endl;
    
    // ========================================
    // Cubic Bezier Curve (4 control points)
    // ========================================
    std::cout << "3. Cubic Bezier Curve:" << std::endl;
    std::vector<Point3D> cubicPoints = {
        Point3D(0, 0, 0),
        Point3D(2, 8, 0),
        Point3D(8, 8, 0),
        Point3D(10, 0, 0)
    };
    BezierCurve cubicCurve(cubicPoints);
    
    std::cout << "Control points: " << cubicPoints.size() << std::endl;
    std::cout << "Degree: " << cubicCurve.degree() << std::endl;
    std::cout << std::endl;
    
    // ========================================
    // 3D Bezier Curve
    // ========================================
    std::cout << "4. 3D Bezier Curve (helix-like):" << std::endl;
    std::vector<Point3D> helix3DPoints = {
        Point3D(0, 0, 0),
        Point3D(5, 5, 5),
        Point3D(10, 0, 10),
        Point3D(15, -5, 15),
        Point3D(20, 0, 20)
    };
    BezierCurve helix3D(helix3DPoints);
    
    std::cout << "Control points: " << helix3DPoints.size() << std::endl;
    std::cout << "Degree: " << helix3D.degree() << std::endl;
    
    // Sample the curve
    std::vector<Point3D> samples = helix3D.sample(11);
    std::cout << "Sampled points:" << std::endl;
    for (size_t i = 0; i < samples.size(); ++i) {
        std::cout << "  [" << i << "] " << samples[i] << std::endl;
    }
    std::cout << std::endl;
    
    // ========================================
    // Transform a Curve
    // ========================================
    std::cout << "5. Transforming a Curve:" << std::endl;
    BezierCurve transformCurve(cubicPoints);
    
    std::cout << "Original curve at t=0.5: " << transformCurve.pointAt(0.5) << std::endl;
    
    // Apply rotation around Z axis (45 degrees)
    Matrix4x4 rotation = Matrix4x4::rotationZ(M_PI / 4);
    transformCurve.transform(rotation);
    
    std::cout << "After 45° rotation around Z at t=0.5: " 
              << transformCurve.pointAt(0.5) << std::endl;
    
    // Apply translation
    Matrix4x4 translation = Matrix4x4::translation(10, 10, 5);
    transformCurve.transform(translation);
    
    std::cout << "After translation (10,10,5) at t=0.5: " 
              << transformCurve.pointAt(0.5) << std::endl;
    std::cout << std::endl;
    
    // ========================================
    // Bounding Box
    // ========================================
    std::cout << "6. Curve Bounding Box:" << std::endl;
    Point3D minBox, maxBox;
    cubicCurve.getBoundingBox(minBox, maxBox);
    std::cout << "Cubic curve bounding box:" << std::endl;
    std::cout << "  Min: " << minBox << std::endl;
    std::cout << "  Max: " << maxBox << std::endl;
    std::cout << std::endl;
    
    std::cout << "Example completed successfully!" << std::endl;
    
    return 0;
}
