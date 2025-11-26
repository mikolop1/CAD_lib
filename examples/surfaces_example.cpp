#include "Geometry.h"
#include <iostream>
#include <iomanip>

using namespace CADLib;

int main() {
    std::cout << "=== CADLib Surfaces Example ===" << std::endl << std::endl;
    std::cout << std::fixed << std::setprecision(3);
    
    // ========================================
    // Planar Surface (Rectangle)
    // ========================================
    std::cout << "1. Planar Surface (Rectangle):" << std::endl;
    
    Point3D origin(0, 0, 0);
    Vector3D uAxis(1, 0, 0);
    Vector3D vAxis(0, 1, 0);
    double width = 10.0;
    double height = 5.0;
    
    PlanarSurface rect(origin, uAxis, vAxis, width, height);
    
    std::cout << "Origin: " << rect.getOrigin() << std::endl;
    std::cout << "Width: " << rect.getWidth() << ", Height: " << rect.getHeight() << std::endl;
    
    // Get the four corners
    std::vector<Point3D> corners = rect.getCorners();
    std::cout << "Corners:" << std::endl;
    for (size_t i = 0; i < corners.size(); ++i) {
        std::cout << "  [" << i << "] " << corners[i] << std::endl;
    }
    
    // Sample points on surface
    std::cout << "Center point (u=0.5, v=0.5): " << rect.pointAt(0.5, 0.5) << std::endl;
    std::cout << "Normal at center: " << rect.normalAt(0.5, 0.5) << std::endl;
    std::cout << std::endl;
    
    // ========================================
    // Planar Surface from 4 Points
    // ========================================
    std::cout << "2. Planar Surface from Four Points:" << std::endl;
    
    Point3D p1(0, 0, 0);
    Point3D p2(8, 2, 0);
    Point3D p3(10, 10, 2);
    Point3D p4(2, 8, 2);
    
    PlanarSurface quadSurface(p1, p2, p3, p4);
    
    std::cout << "Created surface from 4 corner points" << std::endl;
    std::cout << "Point at (0.5, 0.5): " << quadSurface.pointAt(0.5, 0.5) << std::endl;
    std::cout << "Normal: " << quadSurface.normalAt(0.5, 0.5) << std::endl;
    std::cout << std::endl;
    
    // ========================================
    // Bezier Surface (Bilinear Patch)
    // ========================================
    std::cout << "3. Bezier Surface (Bilinear Patch):" << std::endl;
    
    std::vector<std::vector<Point3D>> bilinearCP = {
        {Point3D(0, 0, 0), Point3D(0, 10, 0)},
        {Point3D(10, 0, 0), Point3D(10, 10, 2)}
    };
    
    BezierSurface bilinearPatch(bilinearCP);
    
    std::cout << "U degree: " << bilinearPatch.getUDegree() << std::endl;
    std::cout << "V degree: " << bilinearPatch.getVDegree() << std::endl;
    std::cout << "Corner points:" << std::endl;
    std::cout << "  (0,0): " << bilinearPatch.pointAt(0, 0) << std::endl;
    std::cout << "  (1,0): " << bilinearPatch.pointAt(1, 0) << std::endl;
    std::cout << "  (0,1): " << bilinearPatch.pointAt(0, 1) << std::endl;
    std::cout << "  (1,1): " << bilinearPatch.pointAt(1, 1) << std::endl;
    std::cout << "Center (0.5,0.5): " << bilinearPatch.pointAt(0.5, 0.5) << std::endl;
    std::cout << std::endl;
    
    // ========================================
    // Bezier Surface (Cubic Patch)
    // ========================================
    std::cout << "4. Bezier Surface (Cubic Patch - Curved):" << std::endl;
    
    std::vector<std::vector<Point3D>> cubicCP = {
        {Point3D(0, 0, 0), Point3D(0, 3, 1), Point3D(0, 7, 1), Point3D(0, 10, 0)},
        {Point3D(3, 0, 1), Point3D(3, 3, 3), Point3D(3, 7, 3), Point3D(3, 10, 1)},
        {Point3D(7, 0, 1), Point3D(7, 3, 3), Point3D(7, 7, 3), Point3D(7, 10, 1)},
        {Point3D(10, 0, 0), Point3D(10, 3, 1), Point3D(10, 7, 1), Point3D(10, 10, 0)}
    };
    
    BezierSurface cubicPatch(cubicCP);
    
    std::cout << "U degree: " << cubicPatch.getUDegree() << std::endl;
    std::cout << "V degree: " << cubicPatch.getVDegree() << std::endl;
    std::cout << "Center point (0.5, 0.5): " << cubicPatch.pointAt(0.5, 0.5) << std::endl;
    std::cout << "Normal at center: " << cubicPatch.normalAt(0.5, 0.5) << std::endl;
    std::cout << std::endl;
    
    // ========================================
    // Sample Surface into Grid
    // ========================================
    std::cout << "5. Sampling Surface into Grid:" << std::endl;
    
    auto grid = bilinearPatch.sample(5, 5);
    std::cout << "Sampled " << grid.size() << "x" << grid[0].size() << " grid:" << std::endl;
    
    for (size_t i = 0; i < grid.size(); ++i) {
        for (size_t j = 0; j < grid[i].size(); ++j) {
            if (j == 0) std::cout << "  ";
            std::cout << "(" << grid[i][j].x << "," << grid[i][j].y << "," 
                      << grid[i][j].z << ")";
            if (j < grid[i].size() - 1) std::cout << " ";
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;
    
    // ========================================
    // Transform Surface
    // ========================================
    std::cout << "6. Transforming a Surface:" << std::endl;
    
    PlanarSurface transformSurface(Point3D(0, 0, 0), Vector3D(1, 0, 0), 
                                   Vector3D(0, 1, 0), 5.0, 5.0);
    
    std::cout << "Original center: " << transformSurface.pointAt(0.5, 0.5) << std::endl;
    
    // Rotate 45 degrees around Z
    Matrix4x4 rotation = Matrix4x4::rotationZ(M_PI / 4);
    transformSurface.transform(rotation);
    
    std::cout << "After 45° rotation: " << transformSurface.pointAt(0.5, 0.5) << std::endl;
    
    // Translate
    Matrix4x4 translation = Matrix4x4::translation(10, 10, 5);
    transformSurface.transform(translation);
    
    std::cout << "After translation: " << transformSurface.pointAt(0.5, 0.5) << std::endl;
    std::cout << std::endl;
    
    // ========================================
    // Bounding Box
    // ========================================
    std::cout << "7. Surface Bounding Box:" << std::endl;
    Point3D minBox, maxBox;
    cubicPatch.getBoundingBox(minBox, maxBox);
    std::cout << "Cubic patch bounding box:" << std::endl;
    std::cout << "  Min: " << minBox << std::endl;
    std::cout << "  Max: " << maxBox << std::endl;
    std::cout << "  Size: (" << (maxBox.x - minBox.x) << ", " 
              << (maxBox.y - minBox.y) << ", " 
              << (maxBox.z - minBox.z) << ")" << std::endl;
    std::cout << std::endl;
    
    std::cout << "Example completed successfully!" << std::endl;
    
    return 0;
}
