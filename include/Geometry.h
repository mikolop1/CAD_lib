#ifndef GEOMETRY_H
#define GEOMETRY_H

#include "Point3D.h"
#include "Vector3D.h"
#include "Matrix4x4.h"
#include <vector>
#include <memory>

namespace CADLib {

/**
 * @brief Base class for all geometric entities
 */
class Geometry {
public:
    virtual ~Geometry() = default;
    
    // Transform the geometry
    virtual void transform(const Matrix4x4& matrix) = 0;
    
    // Get bounding box (min and max points)
    virtual void getBoundingBox(Point3D& min, Point3D& max) const = 0;
    
    // Clone the geometry
    virtual std::unique_ptr<Geometry> clone() const = 0;
};

/**
 * @brief Represents a line segment in 3D space
 */
class Line : public Geometry {
public:
    Point3D start;
    Point3D end;
    
    // Constructors
    Line();
    Line(const Point3D& start, const Point3D& end);
    
    // Get direction vector
    Vector3D direction() const;
    
    // Get length
    double length() const;
    
    // Get point at parameter t (0 to 1)
    Point3D pointAt(double t) const;
    
    // Get closest point on line to a given point
    Point3D closestPoint(const Point3D& point) const;
    
    // Get distance from point to line
    double distanceToPoint(const Point3D& point) const;
    
    // Geometry interface
    void transform(const Matrix4x4& matrix) override;
    void getBoundingBox(Point3D& min, Point3D& max) const override;
    std::unique_ptr<Geometry> clone() const override;
};

/**
 * @brief Represents a parametric curve in 3D space
 */
class Curve : public Geometry {
public:
    virtual ~Curve() = default;
    
    // Get point at parameter t
    virtual Point3D pointAt(double t) const = 0;
    
    // Get tangent vector at parameter t
    virtual Vector3D tangentAt(double t) const = 0;
    
    // Get parameter range
    virtual void getParameterRange(double& tMin, double& tMax) const = 0;
    
    // Sample the curve into line segments
    virtual std::vector<Point3D> sample(int numPoints) const;
};

/**
 * @brief Represents a Bezier curve
 */
class BezierCurve : public Curve {
private:
    std::vector<Point3D> controlPoints;
    
    // Binomial coefficient calculation
    int binomial(int n, int k) const;
    
    // Bernstein polynomial
    double bernstein(int n, int i, double t) const;
    
public:
    // Constructors
    BezierCurve();
    BezierCurve(const std::vector<Point3D>& points);
    
    // Control points management
    void setControlPoints(const std::vector<Point3D>& points);
    const std::vector<Point3D>& getControlPoints() const;
    void addControlPoint(const Point3D& point);
    
    // Curve interface
    Point3D pointAt(double t) const override;
    Vector3D tangentAt(double t) const override;
    void getParameterRange(double& tMin, double& tMax) const override;
    
    // Geometry interface
    void transform(const Matrix4x4& matrix) override;
    void getBoundingBox(Point3D& min, Point3D& max) const override;
    std::unique_ptr<Geometry> clone() const override;
    
    // Degree of the curve
    int degree() const;
};

/**
 * @brief Represents a surface in 3D space
 */
class Surface : public Geometry {
public:
    virtual ~Surface() = default;
    
    // Get point at parameters (u, v)
    virtual Point3D pointAt(double u, double v) const = 0;
    
    // Get normal vector at parameters (u, v)
    virtual Vector3D normalAt(double u, double v) const = 0;
    
    // Get parameter range
    virtual void getParameterRange(double& uMin, double& uMax, 
                                   double& vMin, double& vMax) const = 0;
    
    // Sample the surface into a grid of points
    virtual std::vector<std::vector<Point3D>> sample(int uSamples, int vSamples) const;
};

/**
 * @brief Represents a planar surface
 */
class PlanarSurface : public Surface {
private:
    Point3D origin;
    Vector3D uAxis;
    Vector3D vAxis;
    double width;
    double height;
    
public:
    // Constructors
    PlanarSurface();
    PlanarSurface(const Point3D& origin, const Vector3D& uAxis, 
                  const Vector3D& vAxis, double width, double height);
    PlanarSurface(const Point3D& p1, const Point3D& p2, 
                  const Point3D& p3, const Point3D& p4);
    
    // Getters
    Point3D getOrigin() const { return origin; }
    Vector3D getUAxis() const { return uAxis; }
    Vector3D getVAxis() const { return vAxis; }
    double getWidth() const { return width; }
    double getHeight() const { return height; }
    
    // Get the four corners
    std::vector<Point3D> getCorners() const;
    
    // Surface interface
    Point3D pointAt(double u, double v) const override;
    Vector3D normalAt(double u, double v) const override;
    void getParameterRange(double& uMin, double& uMax, 
                          double& vMin, double& vMax) const override;
    
    // Geometry interface
    void transform(const Matrix4x4& matrix) override;
    void getBoundingBox(Point3D& min, Point3D& max) const override;
    std::unique_ptr<Geometry> clone() const override;
};

/**
 * @brief Represents a Bezier surface (tensor product surface)
 */
class BezierSurface : public Surface {
private:
    std::vector<std::vector<Point3D>> controlPoints;
    int uDegree;
    int vDegree;
    
    // Binomial coefficient calculation
    int binomial(int n, int k) const;
    
    // Bernstein polynomial
    double bernstein(int n, int i, double t) const;
    
public:
    // Constructors
    BezierSurface();
    BezierSurface(const std::vector<std::vector<Point3D>>& points);
    
    // Control points management
    void setControlPoints(const std::vector<std::vector<Point3D>>& points);
    const std::vector<std::vector<Point3D>>& getControlPoints() const;
    
    // Surface interface
    Point3D pointAt(double u, double v) const override;
    Vector3D normalAt(double u, double v) const override;
    void getParameterRange(double& uMin, double& uMax, 
                          double& vMin, double& vMax) const override;
    
    // Geometry interface
    void transform(const Matrix4x4& matrix) override;
    void getBoundingBox(Point3D& min, Point3D& max) const override;
    std::unique_ptr<Geometry> clone() const override;
    
    // Get degrees
    int getUDegree() const { return uDegree; }
    int getVDegree() const { return vDegree; }
};

} // namespace CADLib

#endif // GEOMETRY_H
