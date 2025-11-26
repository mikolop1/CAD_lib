#include "Plane.h"
#include "Circle.h"
#include "Loft.h"
#include <iostream>
#include <iomanip>

using namespace CADLib;

void printSeparator() {
    std::cout << "========================================" << std::endl;
}

int main() {
    std::cout << "=== CADLib Construction Planes & Lofting Example ===" << std::endl << std::endl;
    std::cout << std::fixed << std::setprecision(3);
    
    // ========================================
    // Construction Planes
    // ========================================
    std::cout << "1. Construction Planes (Memory Efficient):" << std::endl;
    
    // Standard planes
    Plane xyPlane = Plane::worldXY();
    Plane yzPlane = Plane::worldYZ();
    Plane zxPlane = Plane::worldZX();
    
    std::cout << "World XY Plane: " << xyPlane << std::endl;
    std::cout << "World YZ Plane: " << yzPlane << std::endl;
    std::cout << "World ZX Plane: " << zxPlane << std::endl;
    std::cout << "Memory per plane: " << sizeof(Plane) << " bytes" << std::endl;
    printSeparator();
    std::cout << std::endl;
    
    // ========================================
    // Custom Planes
    // ========================================
    std::cout << "2. Creating Custom Construction Planes:" << std::endl;
    
    // Plane from origin and normal
    Plane customPlane1(Point3D(5, 5, 5), Vector3D(0, 0, 1));
    std::cout << "Plane at (5,5,5) with Z-normal: " << customPlane1 << std::endl;
    
    // Plane from 3 points
    Plane customPlane2(Point3D(0, 0, 0), Point3D(10, 0, 0), Point3D(0, 10, 5));
    std::cout << "Plane from 3 points: " << customPlane2 << std::endl;
    
    // Tilted plane
    Vector3D tiltedNormal(1, 1, 1);
    tiltedNormal.normalize();
    Plane tiltedPlane(Point3D(0, 0, 0), tiltedNormal);
    std::cout << "Tilted plane (45° to all axes): " << tiltedPlane << std::endl;
    printSeparator();
    std::cout << std::endl;
    
    // ========================================
    // Circles on Planes
    // ========================================
    std::cout << "3. Creating Circles on Construction Planes:" << std::endl;
    
    // Circle in XY plane
    Circle circle1(xyPlane, 5.0);
    std::cout << "Circle in XY plane, radius 5:" << std::endl;
    std::cout << "  Center: " << circle1.getCenter() << std::endl;
    std::cout << "  Point at 0°: " << circle1.pointAtAngle(0) << std::endl;
    std::cout << "  Point at 90°: " << circle1.pointAtAngle(M_PI/2) << std::endl;
    std::cout << "  Circumference: " << circle1.circumference() << std::endl;
    std::cout << "  Area: " << circle1.area() << std::endl;
    
    // Circle on tilted plane
    Plane tiltedPlane2(Point3D(0, 0, 10), Vector3D(1, 0, 1).normalized());
    Circle circle2(tiltedPlane2, 3.0);
    std::cout << "\nCircle on tilted plane, radius 3:" << std::endl;
    std::cout << "  Center: " << circle2.getCenter() << std::endl;
    std::cout << "  Normal: " << circle2.getNormal() << std::endl;
    std::cout << "  Point at 0°: " << circle2.pointAtAngle(0) << std::endl;
    
    // Circle from 3 points
    Point3D p1(1, 0, 0), p2(0, 1, 0), p3(-1, 0, 0);
    Circle circle3 = Circle::from3Points(p1, p2, p3);
    std::cout << "\nCircle from 3 points:" << std::endl;
    std::cout << "  Center: " << circle3.getCenter() << std::endl;
    std::cout << "  Radius: " << circle3.getRadius() << std::endl;
    printSeparator();
    std::cout << std::endl;
    
    // ========================================
    // Plane Operations
    // ========================================
    std::cout << "4. Plane Operations:" << std::endl;
    
    Point3D testPoint(5, 5, 3);
    std::cout << "Test point: " << testPoint << std::endl;
    
    Plane horzPlane = Plane::worldXY();
    std::cout << "Distance to XY plane: " << horzPlane.distanceToPoint(testPoint) << std::endl;
    
    Point3D projected = horzPlane.projectPoint(testPoint);
    std::cout << "Projected onto XY plane: " << projected << std::endl;
    
    double u, v;
    horzPlane.worldToPlane(testPoint, u, v);
    std::cout << "Local coordinates on plane: (u=" << u << ", v=" << v << ")" << std::endl;
    
    Point3D backToWorld = horzPlane.planeToWorld(u, v);
    std::cout << "Convert back to world: " << backToWorld << std::endl;
    printSeparator();
    std::cout << std::endl;
    
    // ========================================
    // Straight Loft Between Lines
    // ========================================
    std::cout << "5. Straight Loft Between Two Lines:" << std::endl;
    
    Line line1(Point3D(0, 0, 0), Point3D(10, 0, 0));
    Line line2(Point3D(0, 5, 10), Point3D(10, 5, 10));
    
    std::cout << "Line 1: " << line1.start << " to " << line1.end << std::endl;
    std::cout << "Line 2: " << line2.start << " to " << line2.end << std::endl;
    
    PlanarSurface loftedSurface = Loft::straightLoft(line1, line2);
    std::cout << "Lofted surface corners:" << std::endl;
    auto corners = loftedSurface.getCorners();
    for (size_t i = 0; i < corners.size(); ++i) {
        std::cout << "  [" << i << "] " << corners[i] << std::endl;
    }
    std::cout << "Center: " << loftedSurface.pointAt(0.5, 0.5) << std::endl;
    printSeparator();
    std::cout << std::endl;
    
    // ========================================
    // Straight Loft Between Circles
    // ========================================
    std::cout << "6. Straight Loft Between Two Circles (Cylinder/Cone):" << std::endl;
    
    Plane bottomPlane(Point3D(0, 0, 0), Vector3D(0, 0, 1));
    Plane topPlane(Point3D(0, 0, 10), Vector3D(0, 0, 1));
    
    Circle bottomCircle(bottomPlane, 5.0);
    Circle topCircle(topPlane, 5.0);
    
    std::cout << "Bottom circle: center=" << bottomCircle.getCenter() 
              << ", radius=" << bottomCircle.getRadius() << std::endl;
    std::cout << "Top circle: center=" << topCircle.getCenter() 
              << ", radius=" << topCircle.getRadius() << std::endl;
    
    BezierSurface cylinderSurface = Loft::straightLoft(bottomCircle, topCircle, 8);
    std::cout << "Created cylindrical surface" << std::endl;
    std::cout << "U degree: " << cylinderSurface.getUDegree() << std::endl;
    std::cout << "V degree: " << cylinderSurface.getVDegree() << std::endl;
    std::cout << "Sample point at (0.5, 0.5): " << cylinderSurface.pointAt(0.5, 0.5) << std::endl;
    
    // Conical loft (different radii)
    Circle topCircleSmall(topPlane, 2.0);
    BezierSurface coneSurface = Loft::straightLoft(bottomCircle, topCircleSmall, 8);
    std::cout << "\nCreated conical surface (radius 5 to 2)" << std::endl;
    std::cout << "Sample point at (0.5, 0.5): " << coneSurface.pointAt(0.5, 0.5) << std::endl;
    printSeparator();
    std::cout << std::endl;
    
    // ========================================
    // Loft Between Curves
    // ========================================
    std::cout << "7. Loft Between Bezier Curves:" << std::endl;
    
    std::vector<Point3D> curve1Points = {
        Point3D(0, 0, 0), Point3D(5, 2, 0), Point3D(10, 0, 0)
    };
    std::vector<Point3D> curve2Points = {
        Point3D(0, 0, 10), Point3D(5, 2, 10), Point3D(10, 0, 10)
    };
    
    BezierCurve bezier1(curve1Points);
    BezierCurve bezier2(curve2Points);
    
    std::cout << "Curve 1 at t=0.5: " << bezier1.pointAt(0.5) << std::endl;
    std::cout << "Curve 2 at t=0.5: " << bezier2.pointAt(0.5) << std::endl;
    
    BezierSurface loftedBezier = Loft::straightLoft(bezier1, bezier2, 10);
    std::cout << "Lofted surface at (0.5, 0.5): " << loftedBezier.pointAt(0.5, 0.5) << std::endl;
    printSeparator();
    std::cout << std::endl;
    
    // ========================================
    // Plane-Surface Intersection
    // ========================================
    std::cout << "8. Intersecting Surfaces with Planes:" << std::endl;
    
    PlanarSurface testSurface(Point3D(0, 0, 0), Vector3D(1, 0, 0),
                              Vector3D(0, 1, 0), 10, 10);
    
    Plane cuttingPlane(Point3D(5, 5, 5), Vector3D(0, 0, 1));
    
    std::cout << "Surface: 10x10 in XY plane at z=0" << std::endl;
    std::cout << "Cutting plane: at z=5" << std::endl;
    
    // Test point-plane side
    Point3D testPt1(5, 5, 0);
    Point3D testPt2(5, 5, 10);
    
    double side1 = Intersection::pointPlaneSide(testPt1, cuttingPlane);
    double side2 = Intersection::pointPlaneSide(testPt2, cuttingPlane);
    
    std::cout << "Point " << testPt1 << " is " 
              << (side1 < 0 ? "below" : "above") << " plane (value: " << side1 << ")" << std::endl;
    std::cout << "Point " << testPt2 << " is " 
              << (side2 < 0 ? "below" : "above") << " plane (value: " << side2 << ")" << std::endl;
    
    // Line-plane intersection
    Line testLine(Point3D(5, 5, 0), Point3D(5, 5, 10));
    Point3D intersection;
    if (Intersection::linePlaneIntersection(testLine, cuttingPlane, intersection)) {
        std::cout << "Line intersects plane at: " << intersection << std::endl;
    }
    printSeparator();
    std::cout << std::endl;
    
    // ========================================
    // Plane Transformations
    // ========================================
    std::cout << "9. Transforming Construction Planes:" << std::endl;
    
    Plane movingPlane = Plane::worldXY();
    std::cout << "Original plane: " << movingPlane << std::endl;
    
    Matrix4x4 rotation = Matrix4x4::rotationY(M_PI / 4);
    movingPlane.transform(rotation);
    std::cout << "After 45° rotation around Y: " << movingPlane << std::endl;
    
    Matrix4x4 translation = Matrix4x4::translation(10, 0, 5);
    movingPlane.transform(translation);
    std::cout << "After translation (10,0,5): " << movingPlane << std::endl;
    printSeparator();
    std::cout << std::endl;
    
    std::cout << "Example completed successfully!" << std::endl;
    std::cout << "\nKey Features Demonstrated:" << std::endl;
    std::cout << "- Lightweight construction planes (" << sizeof(Plane) << " bytes)" << std::endl;
    std::cout << "- Circles on arbitrary planes" << std::endl;
    std::cout << "- Straight lofting between lines, circles, and curves" << std::endl;
    std::cout << "- Plane-surface intersection testing" << std::endl;
    std::cout << "- Efficient geometric operations" << std::endl;
    
    return 0;
}
