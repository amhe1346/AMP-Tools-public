#pragma once

// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Path smoothing algorithms for improving PRM/RRT paths

class PathSmoothing {
public:
    // Smooth a path using shortcut method
    // Attempts to connect non-adjacent waypoints directly if collision-free
    static amp::Path2D shortcutSmoothing(const amp::Path2D& path, const amp::Problem2D& problem, int max_iterations = 100);
    
    // Smooth a path using gradient descent on path length
    static amp::Path2D gradientSmoothing(const amp::Path2D& path, const amp::Problem2D& problem, int max_iterations = 100, double step_size = 0.1);
    
private:
    // Check if a straight line between two points is collision-free
    static bool isCollisionFree(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2, const amp::Problem2D& problem);
};
