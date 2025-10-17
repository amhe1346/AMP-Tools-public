#include "PathSmoothing.h"
#include "../shared/MyCollisionChecker.h"

// Check if a straight line between two points is collision-free
bool PathSmoothing::isCollisionFree(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2, const amp::Problem2D& problem) {
    // Use the existing collision checker
    return amp::MyCollisionChecker::isValidPath(
        p1, p2,
        problem.obstacles,
        problem.x_min, problem.x_max,
        problem.y_min, problem.y_max
    );
}

// Shortcut smoothing: Try to connect non-adjacent waypoints directly
amp::Path2D PathSmoothing::shortcutSmoothing(const amp::Path2D& path, const amp::Problem2D& problem, int max_iterations) {
    if (path.waypoints.size() <= 2) {
        return path; // Already optimal
    }
    
    amp::Path2D smoothed_path = path;
    
    for (int iter = 0; iter < max_iterations; ++iter) {
        bool improved = false;
        
        // Try to shortcut between waypoints (only try adjacent shortcuts, not far ones)
        for (size_t i = 0; i < smoothed_path.waypoints.size() && i + 2 < smoothed_path.waypoints.size(); ++i) {
            // Only try to skip one waypoint at a time (connect i to i+2)
            size_t j = i + 2;
            if (j < smoothed_path.waypoints.size()) {
                // Try to connect waypoint i to waypoint j directly
                if (isCollisionFree(smoothed_path.waypoints[i], smoothed_path.waypoints[j], problem)) {
                    // Remove the waypoint between i and j
                    smoothed_path.waypoints.erase(smoothed_path.waypoints.begin() + i + 1);
                    improved = true;
                    break;
                }
            }
        }
        
        // If no improvement, stop
        if (!improved) break;
    }
    
    return smoothed_path;
}

// Gradient smoothing: Move waypoints to reduce path length while avoiding obstacles
amp::Path2D PathSmoothing::gradientSmoothing(const amp::Path2D& path, const amp::Problem2D& problem, int max_iterations, double step_size) {
    if (path.waypoints.size() <= 2) {
        return path; // Cannot smooth
    }
    
    amp::Path2D smoothed_path = path;
    
    for (int iter = 0; iter < max_iterations; ++iter) {
        bool improved = false;
        
        // For each interior waypoint (not start or goal)
        for (size_t i = 1; i < smoothed_path.waypoints.size() - 1; ++i) {
            Eigen::Vector2d& current = smoothed_path.waypoints[i];
            const Eigen::Vector2d& prev = smoothed_path.waypoints[i - 1];
            const Eigen::Vector2d& next = smoothed_path.waypoints[i + 1];
            
            // Compute gradient: direction that reduces path length
            Eigen::Vector2d gradient = (prev + next) / 2.0 - current;
            
            // Try to move in gradient direction
            Eigen::Vector2d new_point = current + step_size * gradient;
            
            // Check if new point is valid (not in obstacles)
            bool valid = !amp::MyCollisionChecker::pointInObstacles(new_point, problem.obstacles);
            
            // Also check edges are collision-free
            if (valid) {
                valid = isCollisionFree(prev, new_point, problem) && 
                        isCollisionFree(new_point, next, problem);
            }
            
            if (valid) {
                current = new_point;
                improved = true;
            }
        }
        
        // If no improvement, stop
        if (!improved) break;
    }
    
    return smoothed_path;
}
