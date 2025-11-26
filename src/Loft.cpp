#include "Loft.h"
#include <algorithm>
#include <cmath>

namespace CADLib {

// ============================================================================
// Loft Implementation
// ============================================================================

BezierSurface Loft::straightLoft(const Curve& curve1, const Curve& curve2, int uSamples) {
    if (uSamples < 2) uSamples = 2;
    
    // Sample both curves
    std::vector<Point3D> samples1 = curve1.sample(uSamples);
    std::vector<Point3D> samples2 = curve2.sample(uSamples);
    
    // Create control point grid for linear interpolation (degree 1 in v direction)
    std::vector<std::vector<Point3D>> controlPoints(uSamples);
    
    for (int i = 0; i < uSamples; ++i) {
        controlPoints[i].resize(2);
        controlPoints[i][0] = samples1[i];
        controlPoints[i][1] = samples2[i];
    }
    
    return BezierSurface(controlPoints);
}

PlanarSurface Loft::straightLoft(const Line& line1, const Line& line2) {
    // Create a ruled surface between two lines
    // The four corners are the endpoints of the two lines
    Point3D p1 = line1.start;
    Point3D p2 = line1.end;
    Point3D p3 = line2.end;
    Point3D p4 = line2.start;
    
    return PlanarSurface(p1, p2, p3, p4);
}

BezierSurface Loft::straightLoft(const Circle& circle1, const Circle& circle2, int samples) {
    if (samples < 3) samples = 3;
    
    std::vector<std::vector<Point3D>> controlPoints(samples);
    
    for (int i = 0; i < samples; ++i) {
        double t = static_cast<double>(i) / (samples - 1);
        double angle = t * 2.0 * M_PI;
        
        controlPoints[i].resize(2);
        controlPoints[i][0] = circle1.pointAtAngle(angle);
        controlPoints[i][1] = circle2.pointAtAngle(angle);
    }
    
    return BezierSurface(controlPoints);
}

BezierSurface Loft::multiCurveLoft(const std::vector<const Curve*>& curves, int vSamples) {
    if (curves.empty()) {
        throw std::invalid_argument("Cannot loft with no curves");
    }
    
    if (curves.size() == 1) {
        throw std::invalid_argument("Cannot loft with single curve");
    }
    
    if (vSamples < 2) vSamples = 2;
    
    // Sample all curves
    std::vector<std::vector<Point3D>> allSamples;
    for (const auto* curve : curves) {
        allSamples.push_back(curve->sample(vSamples));
    }
    
    // Create control point grid
    std::vector<std::vector<Point3D>> controlPoints(vSamples);
    
    for (int i = 0; i < vSamples; ++i) {
        controlPoints[i].resize(curves.size());
        for (size_t j = 0; j < curves.size(); ++j) {
            controlPoints[i][j] = allSamples[j][i];
        }
    }
    
    return BezierSurface(controlPoints);
}

// ============================================================================
// SurfaceSplit Implementation
// ============================================================================

std::vector<PlanarSurface> SurfaceSplit::splitPlanarSurface(
    const PlanarSurface& surface,
    const Plane& cuttingPlane,
    double tolerance) {
    
    std::vector<PlanarSurface> result;
    
    // Get the four corners of the surface
    std::vector<Point3D> corners = surface.getCorners();
    
    // Check which side of the plane each corner is on
    std::vector<double> sides;
    for (const auto& corner : corners) {
        sides.push_back(Intersection::pointPlaneSide(corner, cuttingPlane));
    }
    
    // Count corners on each side
    int positive = 0, negative = 0, onPlane = 0;
    for (double side : sides) {
        if (std::abs(side) < tolerance) onPlane++;
        else if (side > 0) positive++;
        else negative++;
    }
    
    // If all corners on same side or on plane, no split occurs
    if (positive == 0 || negative == 0) {
        // Surface is entirely on one side or touching the plane
        result.push_back(surface);
        return result;
    }
    
    // Surface is split by the plane
    // For now, return the original surface (full implementation would compute intersection)
    // This is a simplified version - full implementation would need to:
    // 1. Find intersection points where edges cross the plane
    // 2. Create new surfaces from the split geometry
    
    result.push_back(surface);
    return result;
}

std::vector<Line> SurfaceSplit::intersectSurfaceWithPlane(
    const Surface& surface,
    const Plane& plane,
    int samples) {
    
    std::vector<Line> intersectionLines;
    
    // Sample the surface and find where it crosses the plane
    auto grid = surface.sample(samples, samples);
    
    std::vector<Point3D> intersectionPoints;
    
    // Check horizontal edges
    for (size_t i = 0; i < grid.size(); ++i) {
        for (size_t j = 0; j < grid[i].size() - 1; ++j) {
            Line edge(grid[i][j], grid[i][j + 1]);
            Point3D intersect;
            if (Intersection::linePlaneIntersection(edge, plane, intersect)) {
                intersectionPoints.push_back(intersect);
            }
        }
    }
    
    // Check vertical edges
    for (size_t i = 0; i < grid.size() - 1; ++i) {
        for (size_t j = 0; j < grid[i].size(); ++j) {
            Line edge(grid[i][j], grid[i + 1][j]);
            Point3D intersect;
            if (Intersection::linePlaneIntersection(edge, plane, intersect)) {
                intersectionPoints.push_back(intersect);
            }
        }
    }
    
    // Connect intersection points into lines
    // This is a simplified approach
    if (intersectionPoints.size() >= 2) {
        intersectionLines.push_back(Line(intersectionPoints[0], 
                                        intersectionPoints[intersectionPoints.size() - 1]));
    }
    
    return intersectionLines;
}

std::vector<std::vector<Point3D>> SurfaceSplit::trimBezierSurface(
    const BezierSurface& surface,
    const Plane& plane,
    int uSamples,
    int vSamples) {
    
    std::vector<std::vector<Point3D>> result;
    
    // Sample the surface
    auto grid = surface.sample(uSamples, vSamples);
    
    // Filter points on positive side of plane
    for (const auto& row : grid) {
        std::vector<Point3D> filteredRow;
        for (const auto& point : row) {
            if (Intersection::pointPlaneSide(point, plane) >= 0) {
                filteredRow.push_back(point);
            }
        }
        if (!filteredRow.empty()) {
            result.push_back(filteredRow);
        }
    }
    
    return result;
}

// ============================================================================
// Intersection Implementation
// ============================================================================

bool Intersection::linePlaneIntersection(const Line& line, const Plane& plane, Point3D& point) {
    Vector3D lineDir = line.direction();
    Vector3D planeNormal = plane.zAxis;
    
    double denominator = lineDir.dot(planeNormal);
    
    // Check if line is parallel to plane
    if (std::abs(denominator) < 1e-9) {
        return false;
    }
    
    Vector3D toPlane(line.start, plane.origin);
    double t = toPlane.dot(planeNormal) / denominator;
    
    // Check if intersection is within line segment
    if (t < 0.0 || t > 1.0) {
        return false;
    }
    
    point = line.pointAt(t);
    return true;
}

bool Intersection::planePlaneIntersection(const Plane& plane1, const Plane& plane2, 
                                         Line& intersectionLine) {
    Vector3D n1 = plane1.zAxis;
    Vector3D n2 = plane2.zAxis;
    
    // Line direction is perpendicular to both normals
    Vector3D direction = n1.cross(n2);
    
    // Check if planes are parallel
    if (direction.lengthSquared() < 1e-9) {
        return false;
    }
    
    direction.normalize();
    
    // Find a point on the intersection line
    // Use the plane equations to solve for a point
    double a1, b1, c1, d1, a2, b2, c2, d2;
    plane1.getEquation(a1, b1, c1, d1);
    plane2.getEquation(a2, b2, c2, d2);
    
    // Try to find a point by setting one coordinate to 0
    Point3D point;
    
    // Try z = 0
    double det = a1 * b2 - a2 * b1;
    if (std::abs(det) > 1e-9) {
        point.x = (b1 * d2 - b2 * d1) / det;
        point.y = (a2 * d1 - a1 * d2) / det;
        point.z = 0;
    }
    // Try y = 0
    else {
        det = a1 * c2 - a2 * c1;
        if (std::abs(det) > 1e-9) {
            point.x = (c1 * d2 - c2 * d1) / det;
            point.y = 0;
            point.z = (a2 * d1 - a1 * d2) / det;
        }
        // Try x = 0
        else {
            det = b1 * c2 - b2 * c1;
            if (std::abs(det) > 1e-9) {
                point.x = 0;
                point.y = (c1 * d2 - c2 * d1) / det;
                point.z = (b2 * d1 - b1 * d2) / det;
            } else {
                return false;
            }
        }
    }
    
    // Create the intersection line
    Point3D endPoint(
        point.x + direction.x,
        point.y + direction.y,
        point.z + direction.z
    );
    
    intersectionLine = Line(point, endPoint);
    return true;
}

double Intersection::pointPlaneSide(const Point3D& point, const Plane& plane) {
    Vector3D toPoint(plane.origin, point);
    return toPoint.dot(plane.zAxis);
}

} // namespace CADLib
