#include "Circle.h"
#include <cmath>
#include <algorithm>
#include <stdexcept>

namespace CADLib {

// ============================================================================
// Circle Implementation
// ============================================================================

Circle::Circle() : plane(Plane::worldXY()), radius(1.0) {}

Circle::Circle(const Plane& plane, double radius) 
    : plane(plane), radius(radius) {
    if (radius <= 0) {
        throw std::invalid_argument("Circle radius must be positive");
    }
}

Circle::Circle(const Point3D& center, const Vector3D& normal, double radius)
    : plane(center, normal), radius(radius) {
    if (radius <= 0) {
        throw std::invalid_argument("Circle radius must be positive");
    }
}

Circle::Circle(const Point3D& center, double radius)
    : plane(Plane::worldXY()), radius(radius) {
    if (radius <= 0) {
        throw std::invalid_argument("Circle radius must be positive");
    }
    plane.origin = center;
}

Circle Circle::from3Points(const Point3D& p1, const Point3D& p2, const Point3D& p3) {
    // Create plane from 3 points
    Plane circlePlane(p1, p2, p3);
    
    // Project points onto plane to work in 2D
    double u1, v1, u2, v2, u3, v3;
    circlePlane.worldToPlane(p1, u1, v1);
    circlePlane.worldToPlane(p2, u2, v2);
    circlePlane.worldToPlane(p3, u3, v3);
    
    // Find circle center in 2D using perpendicular bisectors
    double d = 2.0 * (u1 * (v2 - v3) + u2 * (v3 - v1) + u3 * (v1 - v2));
    
    if (std::abs(d) < EPSILON) {
        throw std::invalid_argument("Points are collinear, cannot create circle");
    }
    
    double ux = ((u1*u1 + v1*v1) * (v2 - v3) + 
                 (u2*u2 + v2*v2) * (v3 - v1) + 
                 (u3*u3 + v3*v3) * (v1 - v2)) / d;
    double uy = ((u1*u1 + v1*v1) * (u3 - u2) + 
                 (u2*u2 + v2*v2) * (u1 - u3) + 
                 (u3*u3 + v3*v3) * (u2 - u1)) / d;
    
    // Convert back to world coordinates
    Point3D center = circlePlane.planeToWorld(ux, uy);
    double radius = center.distanceTo(p1);
    
    circlePlane.origin = center;
    return Circle(circlePlane, radius);
}

Point3D Circle::pointAt(double t) const {
    return pointAtAngle(t);
}

Vector3D Circle::tangentAt(double t) const {
    return tangentAtAngle(t);
}

void Circle::getParameterRange(double& tMin, double& tMax) const {
    tMin = 0.0;
    tMax = 2.0 * M_PI;
}

double Circle::circumference() const {
    return 2.0 * M_PI * radius;
}

double Circle::area() const {
    return M_PI * radius * radius;
}

Point3D Circle::pointAtAngle(double angleRadians) const {
    double x = radius * std::cos(angleRadians);
    double y = radius * std::sin(angleRadians);
    return plane.planeToWorld(x, y);
}

Vector3D Circle::tangentAtAngle(double angleRadians) const {
    double tx = -radius * std::sin(angleRadians);
    double ty = radius * std::cos(angleRadians);
    
    Vector3D tangent(
        tx * plane.xAxis.x + ty * plane.yAxis.x,
        tx * plane.xAxis.y + ty * plane.yAxis.y,
        tx * plane.xAxis.z + ty * plane.yAxis.z
    );
    
    return tangent.normalized();
}

Point3D Circle::pointAtNormalized(double t) const {
    return pointAtAngle(t * 2.0 * M_PI);
}

double Circle::distanceToPoint(const Point3D& point) const {
    Point3D closest = closestPoint(point);
    return point.distanceTo(closest);
}

Point3D Circle::closestPoint(const Point3D& point) const {
    // Project point onto plane
    Point3D projected = plane.projectPoint(point);
    
    // Get vector from center to projected point
    Vector3D toPoint(plane.origin, projected);
    double dist = toPoint.length();
    
    if (dist < EPSILON) {
        // Point is at center, return any point on circle
        return pointAtAngle(0);
    }
    
    // Normalize and scale to radius
    toPoint.normalize();
    return Point3D(
        plane.origin.x + toPoint.x * radius,
        plane.origin.y + toPoint.y * radius,
        plane.origin.z + toPoint.z * radius
    );
}

bool Circle::isPointOnCircle(const Point3D& point, double tolerance) const {
    if (!plane.isPointOnPlane(point, tolerance)) {
        return false;
    }
    double dist = plane.origin.distanceTo(point);
    return std::abs(dist - radius) < tolerance;
}

void Circle::transform(const Matrix4x4& matrix) {
    plane.transform(matrix);
    // Note: For non-uniform scaling, this will distort the circle into an ellipse
    // For now, we scale radius by the average scale factor
    Point3D scaledPoint = matrix.transform(Point3D(radius, 0, 0));
    radius = scaledPoint.distanceFromOrigin();
}

void Circle::getBoundingBox(Point3D& min, Point3D& max) const {
    // Sample points around the circle
    std::vector<Point3D> samples = sample(16);
    
    if (samples.empty()) {
        min = max = plane.origin;
        return;
    }
    
    min = max = samples[0];
    for (const auto& pt : samples) {
        min.x = std::min(min.x, pt.x);
        min.y = std::min(min.y, pt.y);
        min.z = std::min(min.z, pt.z);
        max.x = std::max(max.x, pt.x);
        max.y = std::max(max.y, pt.y);
        max.z = std::max(max.z, pt.z);
    }
}

std::unique_ptr<Geometry> Circle::clone() const {
    return std::make_unique<Circle>(*this);
}

// ============================================================================
// Arc Implementation
// ============================================================================

Arc::Arc() : plane(Plane::worldXY()), radius(1.0), startAngle(0.0), endAngle(M_PI) {}

Arc::Arc(const Plane& plane, double radius, double startAngle, double endAngle)
    : plane(plane), radius(radius), startAngle(startAngle), endAngle(endAngle) {
    if (radius <= 0) {
        throw std::invalid_argument("Arc radius must be positive");
    }
}

Arc::Arc(const Point3D& center, const Vector3D& normal, double radius,
         double startAngle, double endAngle)
    : plane(center, normal), radius(radius), startAngle(startAngle), endAngle(endAngle) {
    if (radius <= 0) {
        throw std::invalid_argument("Arc radius must be positive");
    }
}

Arc Arc::from3Points(const Point3D& start, const Point3D& mid, const Point3D& end) {
    // First create a circle from the 3 points
    Circle circle = Circle::from3Points(start, mid, end);
    
    // Get angles for start and end points
    const Plane& p = circle.getPlane();
    
    double u1, v1, u2, v2;
    p.worldToPlane(start, u1, v1);
    p.worldToPlane(end, u2, v2);
    
    double startAngle = std::atan2(v1, u1);
    double endAngle = std::atan2(v2, u2);
    
    // Ensure the arc goes through the middle point
    double uMid, vMid;
    p.worldToPlane(mid, uMid, vMid);
    double midAngle = std::atan2(vMid, uMid);
    
    // Adjust angles to ensure proper arc direction
    if (endAngle < startAngle) {
        endAngle += 2.0 * M_PI;
    }
    
    double testAngle = startAngle + (endAngle - startAngle) * 0.5;
    double diff1 = std::abs(testAngle - midAngle);
    double diff2 = std::abs(testAngle - (midAngle + 2.0 * M_PI));
    
    if (diff1 > M_PI && diff2 < diff1) {
        endAngle = endAngle - 2.0 * M_PI + startAngle;
    }
    
    return Arc(circle.getPlane(), circle.getRadius(), startAngle, endAngle);
}

double Arc::getAngleSpan() const {
    double span = endAngle - startAngle;
    if (span < 0) span += 2.0 * M_PI;
    return span;
}

Point3D Arc::getStartPoint() const {
    return pointAtAngle(startAngle);
}

Point3D Arc::getEndPoint() const {
    return pointAtAngle(endAngle);
}

Point3D Arc::getMidPoint() const {
    return pointAtAngle((startAngle + endAngle) / 2.0);
}

Point3D Arc::pointAt(double t) const {
    double angle = startAngle + t * (endAngle - startAngle);
    return pointAtAngle(angle);
}

Vector3D Arc::tangentAt(double t) const {
    double angle = startAngle + t * (endAngle - startAngle);
    double tx = -radius * std::sin(angle);
    double ty = radius * std::cos(angle);
    
    Vector3D tangent(
        tx * plane.xAxis.x + ty * plane.yAxis.x,
        tx * plane.xAxis.y + ty * plane.yAxis.y,
        tx * plane.xAxis.z + ty * plane.yAxis.z
    );
    
    return tangent.normalized();
}

void Arc::getParameterRange(double& tMin, double& tMax) const {
    tMin = 0.0;
    tMax = 1.0;
}

double Arc::arcLength() const {
    return radius * getAngleSpan();
}

Point3D Arc::pointAtAngle(double angle) const {
    double x = radius * std::cos(angle);
    double y = radius * std::sin(angle);
    return plane.planeToWorld(x, y);
}

void Arc::transform(const Matrix4x4& matrix) {
    plane.transform(matrix);
    Point3D scaledPoint = matrix.transform(Point3D(radius, 0, 0));
    radius = scaledPoint.distanceFromOrigin();
}

void Arc::getBoundingBox(Point3D& min, Point3D& max) const {
    std::vector<Point3D> samples = sample(16);
    
    if (samples.empty()) {
        min = max = plane.origin;
        return;
    }
    
    min = max = samples[0];
    for (const auto& pt : samples) {
        min.x = std::min(min.x, pt.x);
        min.y = std::min(min.y, pt.y);
        min.z = std::min(min.z, pt.z);
        max.x = std::max(max.x, pt.x);
        max.y = std::max(max.y, pt.y);
        max.z = std::max(max.z, pt.z);
    }
}

std::unique_ptr<Geometry> Arc::clone() const {
    return std::make_unique<Arc>(*this);
}

} // namespace CADLib
