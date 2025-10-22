#pragma once

#include "AMPCore.h"

namespace amp {

// Basic collision checker class
class MyCollisionChecker {
public:
    MyCollisionChecker() = default;
    virtual ~MyCollisionChecker() = default;
    
    // Static method for path validation
    static bool isValidPath(const Eigen::Vector2d& start, const Eigen::Vector2d& end,
                           const std::vector<amp::Obstacle2D>& obstacles,
                           double x_min, double x_max, double y_min, double y_max) {
        
        // Check if start or end points are out of bounds
        if (start.x() < x_min || start.x() > x_max || start.y() < y_min || start.y() > y_max ||
            end.x() < x_min || end.x() > x_max || end.y() < y_min || end.y() > y_max) {
            return false;
        }
        
        // Check if start or end points are in obstacles
        if (pointInObstacles(start, obstacles) || pointInObstacles(end, obstacles)) {
            return false;
        }
        
        // Check line segment intersection with obstacles
        for (const auto& obstacle : obstacles) {
            if (lineIntersectsPolygon(start, end, obstacle)) {
                return false;
            }
        }
        
        return true;
    }
    
    // Static method for point in obstacles check
    static bool pointInObstacles(const Eigen::Vector2d& point, const std::vector<amp::Obstacle2D>& obstacles) {
        for (const auto& obstacle : obstacles) {
            if (pointInPolygon(point, obstacle)) {
                return true;
            }
        }
        return false;
    }

private:
    // Helper method: Check if point is inside a polygon using ray casting algorithm
    static bool pointInPolygon(const Eigen::Vector2d& point, const amp::Obstacle2D& polygon) {
        const auto& vertices = polygon.verticesCCW();
        int n = vertices.size();
        bool inside = false;
        
        for (int i = 0, j = n - 1; i < n; j = i++) {
            const auto& vi = vertices[i];
            const auto& vj = vertices[j];
            
            if (((vi.y() > point.y()) != (vj.y() > point.y())) &&
                (point.x() < (vj.x() - vi.x()) * (point.y() - vi.y()) / (vj.y() - vi.y()) + vi.x())) {
                inside = !inside;
            }
        }
        return inside;
    }
    
    // Helper method: Check if line segment intersects with polygon
    static bool lineIntersectsPolygon(const Eigen::Vector2d& start, const Eigen::Vector2d& end, 
                                     const amp::Obstacle2D& polygon) {
        const auto& vertices = polygon.verticesCCW();
        int n = vertices.size();
        
        // Check intersection with each edge of the polygon
        for (int i = 0; i < n; i++) {
            const auto& p1 = vertices[i];
            const auto& p2 = vertices[(i + 1) % n];
            
            if (lineSegmentsIntersect(start, end, p1, p2)) {
                return true;
            }
        }
        return false;
    }
    
    // Helper method: Check if two line segments intersect
    static bool lineSegmentsIntersect(const Eigen::Vector2d& p1, const Eigen::Vector2d& q1,
                                     const Eigen::Vector2d& p2, const Eigen::Vector2d& q2) {
        auto orientation = [](const Eigen::Vector2d& p, const Eigen::Vector2d& q, const Eigen::Vector2d& r) {
            double val = (q.y() - p.y()) * (r.x() - q.x()) - (q.x() - p.x()) * (r.y() - q.y());
            if (std::abs(val) < 1e-10) return 0;  // collinear
            return (val > 0) ? 1 : 2;  // clock or counterclock wise
        };
        
        auto onSegment = [](const Eigen::Vector2d& p, const Eigen::Vector2d& q, const Eigen::Vector2d& r) {
            return q.x() <= std::max(p.x(), r.x()) && q.x() >= std::min(p.x(), r.x()) &&
                   q.y() <= std::max(p.y(), r.y()) && q.y() >= std::min(p.y(), r.y());
        };
        
        int o1 = orientation(p1, q1, p2);
        int o2 = orientation(p1, q1, q2);
        int o3 = orientation(p2, q2, p1);
        int o4 = orientation(p2, q2, q1);
        
        // General case
        if (o1 != o2 && o3 != o4) return true;
        
        // Special cases
        if (o1 == 0 && onSegment(p1, p2, q1)) return true;
        if (o2 == 0 && onSegment(p1, q2, q1)) return true;
        if (o3 == 0 && onSegment(p2, p1, q2)) return true;
        if (o4 == 0 && onSegment(p2, q1, q2)) return true;
        
        return false;
    }
    
public:
    // Basic collision checking methods
    virtual bool isInCollision(const amp::Environment2D& env, const amp::Path2D& path) const {
        // Check each segment in the path
        for (size_t i = 0; i < path.waypoints.size() - 1; i++) {
            if (!isValidPath(path.waypoints[i], path.waypoints[i + 1], 
                           env.obstacles, env.x_min, env.x_max, env.y_min, env.y_max)) {
                return true; // collision found
            }
        }
        return false; // no collision
    }
    
    virtual bool isInCollision(const amp::Environment2D& env, const Eigen::Vector2d& point) const {
        // Check if point is out of bounds
        if (point.x() < env.x_min || point.x() > env.x_max || 
            point.y() < env.y_min || point.y() > env.y_max) {
            return true;
        }
        
        // Check if point is in obstacles
        return pointInObstacles(point, env.obstacles);
    }
};

// C-space collision checker for grid-based collision detection
class MyCSpaceCollisionChecker {
public:
    MyCSpaceCollisionChecker(const amp::GridCSpace2D* cspace) : m_cspace(cspace) {}
    
    // Check if a point is in collision using the C-space grid
    bool inCollision(double x, double y) const {
        auto cell = m_cspace->getCellFromPoint(x, y);
        return (*m_cspace)(cell.first, cell.second);
    }
    
    // Check if a path is valid by sampling points along it
    bool isValidPath(const Eigen::Vector2d& start, const Eigen::Vector2d& end) const {
        const int num_checks = 20;
        for (int i = 0; i <= num_checks; ++i) {
            double t = static_cast<double>(i) / num_checks;
            Eigen::Vector2d point = start + t * (end - start);
            if (inCollision(point.x(), point.y())) {
                return false;
            }
        }
        return true;
    }
    
    // Check if a point is valid (not in collision and within bounds)
    bool isValidPoint(const Eigen::Vector2d& point) const {
        auto x_bounds = m_cspace->x0Bounds();
        auto y_bounds = m_cspace->x1Bounds();
        
        if (point.x() < x_bounds.first || point.x() > x_bounds.second ||
            point.y() < y_bounds.first || point.y() > y_bounds.second) {
            return false;
        }
        
        return !inCollision(point.x(), point.y());
    }
    
private:
    const amp::GridCSpace2D* m_cspace;
};

} // namespace amp