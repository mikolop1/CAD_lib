#ifndef CIRCLE_H
#define CIRCLE_H

#include "Point3D.h"
#include "Vector3D.h"
#include "Plane.h"
#include "Geometry.h"
#include <vector>

namespace CADLib {

/**
 * @brief Represents a circle in 3D space
 * 
 * A circle is defined by a plane (center and normal) and a radius.
 * The circle lies on the plane and can be evaluated parametrically.
 */
class Circle : public Curve {
private:
    Plane plane;
    double radius;
    
public:
    // Constructors
    Circle();
    Circle(const Plane& plane, double radius);
    Circle(const Point3D& center, const Vector3D& normal, double radius);
    Circle(const Point3D& center, double radius);  // Circle in XY plane
    
    // Create circle from 3 points
    static Circle from3Points(const Point3D& p1, const Point3D& p2, const Point3D& p3);
    
    // Getters
    const Plane& getPlane() const { return plane; }
    Point3D getCenter() const { return plane.origin; }
    Vector3D getNormal() const { return plane.zAxis; }
    double getRadius() const { return radius; }
    
    // Setters
    void setPlane(const Plane& p) { plane = p; }
    void setRadius(double r) { radius = r; }
    
    // Curve interface - parameter t goes from 0 to 2*PI
    Point3D pointAt(double t) const override;
    Vector3D tangentAt(double t) const override;
    void getParameterRange(double& tMin, double& tMax) const override;
    
    // Additional circle-specific methods
    double circumference() const;
    double area() const;
    Point3D pointAtAngle(double angleRadians) const;
    Vector3D tangentAtAngle(double angleRadians) const;
    
    // Get point at normalized parameter (0 to 1 instead of 0 to 2*PI)
    Point3D pointAtNormalized(double t) const;
    
    // Distance to point
    double distanceToPoint(const Point3D& point) const;
    Point3D closestPoint(const Point3D& point) const;
    
    // Check if point is on circle
    bool isPointOnCircle(const Point3D& point, double tolerance = 1e-9) const;
    
    // Geometry interface
    void transform(const Matrix4x4& matrix) override;
    void getBoundingBox(Point3D& min, Point3D& max) const override;
    std::unique_ptr<Geometry> clone() const override;
    
private:
    static constexpr double EPSILON = 1e-9;
};

/**
 * @brief Represents an arc (partial circle) in 3D space
 */
class Arc : public Curve {
private:
    Plane plane;
    double radius;
    double startAngle;  // In radians
    double endAngle;    // In radians
    
public:
    // Constructors
    Arc();
    Arc(const Plane& plane, double radius, double startAngle, double endAngle);
    Arc(const Point3D& center, const Vector3D& normal, double radius, 
        double startAngle, double endAngle);
    
    // Create arc from 3 points
    static Arc from3Points(const Point3D& start, const Point3D& mid, const Point3D& end);
    
    // Getters
    const Plane& getPlane() const { return plane; }
    Point3D getCenter() const { return plane.origin; }
    Vector3D getNormal() const { return plane.zAxis; }
    double getRadius() const { return radius; }
    double getStartAngle() const { return startAngle; }
    double getEndAngle() const { return endAngle; }
    double getAngleSpan() const;
    
    // Get start and end points
    Point3D getStartPoint() const;
    Point3D getEndPoint() const;
    Point3D getMidPoint() const;
    
    // Curve interface - parameter t is normalized from 0 to 1
    Point3D pointAt(double t) const override;
    Vector3D tangentAt(double t) const override;
    void getParameterRange(double& tMin, double& tMax) const override;
    
    // Arc-specific methods
    double arcLength() const;
    Point3D pointAtAngle(double angle) const;
    
    // Geometry interface
    void transform(const Matrix4x4& matrix) override;
    void getBoundingBox(Point3D& min, Point3D& max) const override;
    std::unique_ptr<Geometry> clone() const override;
};

} // namespace CADLib

#endif // CIRCLE_H
