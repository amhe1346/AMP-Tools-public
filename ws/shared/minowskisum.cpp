#include <Eigen/Dense>
#include <vector>
#include <algorithm>

// Forward declaration for convexHull
std::vector<Eigen::Vector2d> convexHull(std::vector<Eigen::Vector2d> pts);

// Buffer (offset) a convex polygon by a given distance (Eigen version)
std::vector<Eigen::Vector2d> bufferConvexPolygonEigen(const std::vector<Eigen::Vector2d>& polygon, double buffer) {
    size_t n = polygon.size();
    if (n < 3) return polygon;
    std::vector<Eigen::Vector2d> offset_pts;
    for (size_t i = 0; i < n; ++i) {
        const Eigen::Vector2d& prev = polygon[(i + n - 1) % n];
        const Eigen::Vector2d& curr = polygon[i];
        const Eigen::Vector2d& next = polygon[(i + 1) % n];
        // Compute edge normals
        Eigen::Vector2d edge1 = (curr - prev).normalized();
        Eigen::Vector2d normal1(-edge1.y(), edge1.x());
        Eigen::Vector2d edge2 = (next - curr).normalized();
        Eigen::Vector2d normal2(-edge2.y(), edge2.x());
        // Average the normals
        Eigen::Vector2d avg_normal = (normal1 + normal2).normalized();
        // Offset the vertex
        offset_pts.push_back(curr + buffer * avg_normal);
    }
    return convexHull(offset_pts);
}
#include <Eigen/Dense>
#include <vector>
#include <utility>
#include <cmath>


   
// Helper function: 2D cross product
static double cross(const Eigen::Vector2d& O, const Eigen::Vector2d& A, const Eigen::Vector2d& B) {
    return (A.x() - O.x()) * (B.y() - O.y()) - (A.y() - O.y()) * (B.x() - O.x());
}

// Compute convex hull using Andrew's monotone chain algorithm
std::vector<Eigen::Vector2d> convexHull(std::vector<Eigen::Vector2d> pts) {
    size_t n = pts.size(), k = 0;
    if (n <= 3) return pts;
    std::sort(pts.begin(), pts.end(), [](const Eigen::Vector2d& a, const Eigen::Vector2d& b) {
        return a.x() < b.x() || (a.x() == b.x() && a.y() < b.y());
    });
    std::vector<Eigen::Vector2d> hull(2 * n);
    // Lower hull
    for (size_t i = 0; i < n; ++i) {
        while (k >= 2 && cross(hull[k - 2], hull[k - 1], pts[i]) <= 0) k--;
        hull[k++] = pts[i];
    }
    // Upper hull
    for (size_t i = n - 1, t = k + 1; i > 0; --i) {
        while (k >= t && cross(hull[k - 2], hull[k - 1], pts[i - 1]) <= 0) k--;
        hull[k++] = pts[i - 1];
    }
    hull.resize(k - 1);
    return hull;
}

// Classic Minkowski sum for any robot polygon (robot should be centered at origin)
std::vector<Eigen::Vector2d> minkowskiSumEigen(
    const std::vector<Eigen::Vector2d>& obstacle,
    const std::vector<Eigen::Vector2d>& robot)
{
    std::vector<Eigen::Vector2d> convex_obstacle = convexHull(obstacle);
    std::vector<Eigen::Vector2d> convex_robot = convexHull(robot);
    std::vector<Eigen::Vector2d> cspace_vertices;
    for (const auto& obs_vertex : convex_obstacle) {
        for (const auto& robot_vertex : convex_robot) {
            cspace_vertices.push_back(obs_vertex - robot_vertex); // reflect robot about origin
        }
    }
    return convexHull(cspace_vertices);
}