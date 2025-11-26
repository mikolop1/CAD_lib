#ifndef LOFT_H
#define LOFT_H

#include "Geometry.h"
#include "Circle.h"
#include <vector>
#include <memory>

namespace CADLib {

/**
 * @brief Creates lofted surfaces between curves
 * 
 * Lofting creates a surface that smoothly transitions between two or more curves.
 * This is similar to Grasshopper's Loft component.
 */
class Loft {
public:
    /**
     * @brief Create a ruled surface (straight loft) between two curves
     * 
     * Creates a surface by linearly interpolating between corresponding points
     * on two curves. This is the simplest form of loft.
     * 
     * @param curve1 First curve
     * @param curve2 Second curve
     * @param uSamples Number of samples along curves (minimum 2)
     * @return BezierSurface representing the lofted surface
     */
    static BezierSurface straightLoft(const Curve& curve1, const Curve& curve2, 
                                      int uSamples = 10);
    
    /**
     * @brief Create a straight loft between two lines
     * 
     * Optimized version for lofting between two line segments.
     * Creates a ruled surface (planar or warped quadrilateral).
     */
    static PlanarSurface straightLoft(const Line& line1, const Line& line2);
    
    /**
     * @brief Create a straight loft between two circles
     * 
     * Creates a cylindrical or conical surface between two circles.
     * Circles must be on parallel planes for a true cylinder.
     */
    static BezierSurface straightLoft(const Circle& circle1, const Circle& circle2,
                                      int samples = 16);
    
    /**
     * @brief Create a loft between multiple curves
     * 
     * Creates a surface that passes through all provided curves.
     * Uses Bezier interpolation for smooth transitions.
     * 
     * @param curves Vector of curves to loft through
     * @param vSamples Number of samples along each curve
     * @return BezierSurface representing the multi-curve loft
     */
    static BezierSurface multiCurveLoft(const std::vector<const Curve*>& curves,
                                        int vSamples = 10);
};

/**
 * @brief Surface splitting and trimming operations
 */
class SurfaceSplit {
public:
    /**
     * @brief Split a planar surface with a plane
     * 
     * Divides a planar surface into two parts along the intersection
     * with a plane. Returns the portions on both sides of the cutting plane.
     * 
     * @param surface The planar surface to split
     * @param cuttingPlane The plane to split with
     * @param tolerance Numerical tolerance for intersection
     * @return Vector of resulting surface pieces (0, 1, or 2 surfaces)
     */
    static std::vector<PlanarSurface> splitPlanarSurface(
        const PlanarSurface& surface,
        const Plane& cuttingPlane,
        double tolerance = 1e-9);
    
    /**
     * @brief Find intersection curve between a surface and a plane
     * 
     * Computes the curve(s) where a surface intersects with a plane.
     * For planar surfaces, this will be a line or empty.
     * For curved surfaces, this may be a curve.
     * 
     * @param surface The surface to intersect
     * @param plane The intersection plane
     * @param samples Number of samples to use for curved surfaces
     * @return Vector of intersection curves (as lines or point sequences)
     */
    static std::vector<Line> intersectSurfaceWithPlane(
        const Surface& surface,
        const Plane& plane,
        int samples = 20);
    
    /**
     * @brief Trim a Bezier surface with a plane
     * 
     * Cuts a Bezier surface along its intersection with a plane.
     * Returns the portion(s) of the surface on the positive side of the plane.
     */
    static std::vector<std::vector<Point3D>> trimBezierSurface(
        const BezierSurface& surface,
        const Plane& plane,
        int uSamples = 20,
        int vSamples = 20);
};

/**
 * @brief Helper class for computing intersections
 */
class Intersection {
public:
    /**
     * @brief Find intersection point between a line and a plane
     * 
     * @param line The line to intersect
     * @param plane The plane to intersect with
     * @param point Output parameter for intersection point
     * @return true if intersection exists, false if line is parallel to plane
     */
    static bool linePlaneIntersection(const Line& line, const Plane& plane,
                                      Point3D& point);
    
    /**
     * @brief Find intersection between two planes
     * 
     * Two planes intersect in a line (unless they are parallel or coincident).
     * 
     * @param plane1 First plane
     * @param plane2 Second plane
     * @param intersectionLine Output line of intersection
     * @return true if planes intersect in a line, false if parallel
     */
    static bool planePlaneIntersection(const Plane& plane1, const Plane& plane2,
                                       Line& intersectionLine);
    
    /**
     * @brief Check which side of a plane a point is on
     * 
     * @param point The point to test
     * @param plane The plane to test against
     * @return Positive if on positive side, negative if on negative side, 0 if on plane
     */
    static double pointPlaneSide(const Point3D& point, const Plane& plane);
};

} // namespace CADLib

#endif // LOFT_H
