#include "Geometry.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace CADLib {

// ============================================================================
// Line Implementation
// ============================================================================

Line::Line() : start(), end() {}

Line::Line(const Point3D& start, const Point3D& end) : start(start), end(end) {}

Vector3D Line::direction() const {
    return Vector3D(start, end);
}

double Line::length() const {
    return start.distanceTo(end);
}

Point3D Line::pointAt(double t) const {
    return Point3D(
        start.x + t * (end.x - start.x),
        start.y + t * (end.y - start.y),
        start.z + t * (end.z - start.z)
    );
}

Point3D Line::closestPoint(const Point3D& point) const {
    Vector3D lineDir = direction();
    double lineLength = lineDir.length();
    
    if (lineLength < 1e-9) {
        return start;
    }
    
    Vector3D toPoint(start, point);
    double t = toPoint.dot(lineDir) / (lineLength * lineLength);
    
    // Clamp to [0, 1] to stay on the line segment
    t = std::max(0.0, std::min(1.0, t));
    
    return pointAt(t);
}

double Line::distanceToPoint(const Point3D& point) const {
    Point3D closest = closestPoint(point);
    return point.distanceTo(closest);
}

void Line::transform(const Matrix4x4& matrix) {
    start = matrix.transform(start);
    end = matrix.transform(end);
}

void Line::getBoundingBox(Point3D& min, Point3D& max) const {
    min.x = std::min(start.x, end.x);
    min.y = std::min(start.y, end.y);
    min.z = std::min(start.z, end.z);
    
    max.x = std::max(start.x, end.x);
    max.y = std::max(start.y, end.y);
    max.z = std::max(start.z, end.z);
}

std::unique_ptr<Geometry> Line::clone() const {
    return std::make_unique<Line>(*this);
}

// ============================================================================
// Curve Implementation
// ============================================================================

std::vector<Point3D> Curve::sample(int numPoints) const {
    std::vector<Point3D> points;
    if (numPoints < 2) return points;
    
    double tMin, tMax;
    getParameterRange(tMin, tMax);
    
    for (int i = 0; i < numPoints; ++i) {
        double t = tMin + (tMax - tMin) * i / (numPoints - 1);
        points.push_back(pointAt(t));
    }
    
    return points;
}

// ============================================================================
// BezierCurve Implementation
// ============================================================================

BezierCurve::BezierCurve() {}

BezierCurve::BezierCurve(const std::vector<Point3D>& points) 
    : controlPoints(points) {}

void BezierCurve::setControlPoints(const std::vector<Point3D>& points) {
    controlPoints = points;
}

const std::vector<Point3D>& BezierCurve::getControlPoints() const {
    return controlPoints;
}

void BezierCurve::addControlPoint(const Point3D& point) {
    controlPoints.push_back(point);
}

int BezierCurve::binomial(int n, int k) const {
    if (k > n) return 0;
    if (k == 0 || k == n) return 1;
    
    int result = 1;
    for (int i = 1; i <= k; ++i) {
        result *= (n - i + 1);
        result /= i;
    }
    return result;
}

double BezierCurve::bernstein(int n, int i, double t) const {
    return binomial(n, i) * std::pow(1 - t, n - i) * std::pow(t, i);
}

Point3D BezierCurve::pointAt(double t) const {
    if (controlPoints.empty()) {
        return Point3D();
    }
    
    if (controlPoints.size() == 1) {
        return controlPoints[0];
    }
    
    int n = controlPoints.size() - 1;
    Point3D result;
    
    for (int i = 0; i <= n; ++i) {
        double b = bernstein(n, i, t);
        result.x += b * controlPoints[i].x;
        result.y += b * controlPoints[i].y;
        result.z += b * controlPoints[i].z;
    }
    
    return result;
}

Vector3D BezierCurve::tangentAt(double t) const {
    if (controlPoints.size() < 2) {
        return Vector3D();
    }
    
    int n = controlPoints.size() - 1;
    Vector3D result;
    
    for (int i = 0; i < n; ++i) {
        double b = bernstein(n - 1, i, t);
        Vector3D diff(controlPoints[i], controlPoints[i + 1]);
        result += diff * b;
    }
    
    result *= n;
    return result.normalized();
}

void BezierCurve::getParameterRange(double& tMin, double& tMax) const {
    tMin = 0.0;
    tMax = 1.0;
}

void BezierCurve::transform(const Matrix4x4& matrix) {
    for (auto& point : controlPoints) {
        point = matrix.transform(point);
    }
}

void BezierCurve::getBoundingBox(Point3D& min, Point3D& max) const {
    if (controlPoints.empty()) {
        min = max = Point3D();
        return;
    }
    
    min = max = controlPoints[0];
    
    for (const auto& point : controlPoints) {
        min.x = std::min(min.x, point.x);
        min.y = std::min(min.y, point.y);
        min.z = std::min(min.z, point.z);
        
        max.x = std::max(max.x, point.x);
        max.y = std::max(max.y, point.y);
        max.z = std::max(max.z, point.z);
    }
}

std::unique_ptr<Geometry> BezierCurve::clone() const {
    return std::make_unique<BezierCurve>(*this);
}

int BezierCurve::degree() const {
    return controlPoints.empty() ? 0 : controlPoints.size() - 1;
}

// ============================================================================
// Surface Implementation
// ============================================================================

std::vector<std::vector<Point3D>> Surface::sample(int uSamples, int vSamples) const {
    std::vector<std::vector<Point3D>> points;
    if (uSamples < 2 || vSamples < 2) return points;
    
    double uMin, uMax, vMin, vMax;
    getParameterRange(uMin, uMax, vMin, vMax);
    
    points.resize(uSamples);
    for (int i = 0; i < uSamples; ++i) {
        double u = uMin + (uMax - uMin) * i / (uSamples - 1);
        points[i].resize(vSamples);
        
        for (int j = 0; j < vSamples; ++j) {
            double v = vMin + (vMax - vMin) * j / (vSamples - 1);
            points[i][j] = pointAt(u, v);
        }
    }
    
    return points;
}

// ============================================================================
// PlanarSurface Implementation
// ============================================================================

PlanarSurface::PlanarSurface() 
    : origin(), uAxis(1, 0, 0), vAxis(0, 1, 0), width(1.0), height(1.0) {}

PlanarSurface::PlanarSurface(const Point3D& origin, const Vector3D& uAxis, 
                             const Vector3D& vAxis, double width, double height)
    : origin(origin), uAxis(uAxis.normalized()), vAxis(vAxis.normalized()), 
      width(width), height(height) {}

PlanarSurface::PlanarSurface(const Point3D& p1, const Point3D& p2, 
                             const Point3D& p3, const Point3D& p4) {
    origin = p1;
    Vector3D u(p1, p2);
    Vector3D v(p1, p4);
    
    width = u.length();
    height = v.length();
    
    uAxis = u.normalized();
    vAxis = v.normalized();
}

std::vector<Point3D> PlanarSurface::getCorners() const {
    std::vector<Point3D> corners;
    corners.push_back(pointAt(0, 0));
    corners.push_back(pointAt(1, 0));
    corners.push_back(pointAt(1, 1));
    corners.push_back(pointAt(0, 1));
    return corners;
}

Point3D PlanarSurface::pointAt(double u, double v) const {
    return Point3D(
        origin.x + u * width * uAxis.x + v * height * vAxis.x,
        origin.y + u * width * uAxis.y + v * height * vAxis.y,
        origin.z + u * width * uAxis.z + v * height * vAxis.z
    );
}

Vector3D PlanarSurface::normalAt(double u, double v) const {
    return uAxis.cross(vAxis).normalized();
}

void PlanarSurface::getParameterRange(double& uMin, double& uMax, 
                                      double& vMin, double& vMax) const {
    uMin = vMin = 0.0;
    uMax = vMax = 1.0;
}

void PlanarSurface::transform(const Matrix4x4& matrix) {
    origin = matrix.transform(origin);
    uAxis = matrix.transform(uAxis, true);
    vAxis = matrix.transform(vAxis, true);
}

void PlanarSurface::getBoundingBox(Point3D& min, Point3D& max) const {
    std::vector<Point3D> corners = getCorners();
    
    min = max = corners[0];
    for (const auto& corner : corners) {
        min.x = std::min(min.x, corner.x);
        min.y = std::min(min.y, corner.y);
        min.z = std::min(min.z, corner.z);
        
        max.x = std::max(max.x, corner.x);
        max.y = std::max(max.y, corner.y);
        max.z = std::max(max.z, corner.z);
    }
}

std::unique_ptr<Geometry> PlanarSurface::clone() const {
    return std::make_unique<PlanarSurface>(*this);
}

// ============================================================================
// BezierSurface Implementation
// ============================================================================

BezierSurface::BezierSurface() : uDegree(0), vDegree(0) {}

BezierSurface::BezierSurface(const std::vector<std::vector<Point3D>>& points) 
    : controlPoints(points) {
    if (!points.empty() && !points[0].empty()) {
        uDegree = points.size() - 1;
        vDegree = points[0].size() - 1;
    } else {
        uDegree = vDegree = 0;
    }
}

void BezierSurface::setControlPoints(const std::vector<std::vector<Point3D>>& points) {
    controlPoints = points;
    if (!points.empty() && !points[0].empty()) {
        uDegree = points.size() - 1;
        vDegree = points[0].size() - 1;
    } else {
        uDegree = vDegree = 0;
    }
}

const std::vector<std::vector<Point3D>>& BezierSurface::getControlPoints() const {
    return controlPoints;
}

int BezierSurface::binomial(int n, int k) const {
    if (k > n) return 0;
    if (k == 0 || k == n) return 1;
    
    int result = 1;
    for (int i = 1; i <= k; ++i) {
        result *= (n - i + 1);
        result /= i;
    }
    return result;
}

double BezierSurface::bernstein(int n, int i, double t) const {
    return binomial(n, i) * std::pow(1 - t, n - i) * std::pow(t, i);
}

Point3D BezierSurface::pointAt(double u, double v) const {
    if (controlPoints.empty() || controlPoints[0].empty()) {
        return Point3D();
    }
    
    Point3D result;
    
    for (int i = 0; i <= uDegree; ++i) {
        double bu = bernstein(uDegree, i, u);
        for (int j = 0; j <= vDegree; ++j) {
            double bv = bernstein(vDegree, j, v);
            double basis = bu * bv;
            
            result.x += basis * controlPoints[i][j].x;
            result.y += basis * controlPoints[i][j].y;
            result.z += basis * controlPoints[i][j].z;
        }
    }
    
    return result;
}

Vector3D BezierSurface::normalAt(double u, double v) const {
    if (controlPoints.empty() || controlPoints[0].empty()) {
        return Vector3D(0, 0, 1);
    }
    
    // Compute partial derivatives
    Vector3D du, dv;
    
    // Partial derivative with respect to u
    for (int i = 0; i < uDegree; ++i) {
        double bu = bernstein(uDegree - 1, i, u);
        for (int j = 0; j <= vDegree; ++j) {
            double bv = bernstein(vDegree, j, v);
            Vector3D diff(controlPoints[i][j], controlPoints[i + 1][j]);
            du += diff * (bu * bv);
        }
    }
    du *= uDegree;
    
    // Partial derivative with respect to v
    for (int i = 0; i <= uDegree; ++i) {
        double bu = bernstein(uDegree, i, u);
        for (int j = 0; j < vDegree; ++j) {
            double bv = bernstein(vDegree - 1, j, v);
            Vector3D diff(controlPoints[i][j], controlPoints[i][j + 1]);
            dv += diff * (bu * bv);
        }
    }
    dv *= vDegree;
    
    return du.cross(dv).normalized();
}

void BezierSurface::getParameterRange(double& uMin, double& uMax, 
                                      double& vMin, double& vMax) const {
    uMin = vMin = 0.0;
    uMax = vMax = 1.0;
}

void BezierSurface::transform(const Matrix4x4& matrix) {
    for (auto& row : controlPoints) {
        for (auto& point : row) {
            point = matrix.transform(point);
        }
    }
}

void BezierSurface::getBoundingBox(Point3D& min, Point3D& max) const {
    if (controlPoints.empty() || controlPoints[0].empty()) {
        min = max = Point3D();
        return;
    }
    
    min = max = controlPoints[0][0];
    
    for (const auto& row : controlPoints) {
        for (const auto& point : row) {
            min.x = std::min(min.x, point.x);
            min.y = std::min(min.y, point.y);
            min.z = std::min(min.z, point.z);
            
            max.x = std::max(max.x, point.x);
            max.y = std::max(max.y, point.y);
            max.z = std::max(max.z, point.z);
        }
    }
}

std::unique_ptr<Geometry> BezierSurface::clone() const {
    return std::make_unique<BezierSurface>(*this);
}

} // namespace CADLib
