#pragma once

// ============================================================================
// MY CORE ALGORITHMS - Quick Access Header
// ============================================================================

#include "AMPCore.h"

// Core algorithm implementations
#include "MyCollisionChecker.h"
#include "MySamplingBasedPlanners.h"
#include "MyTranslationalCSpace.h"

// ============================================================================
// ALGORITHM SHORTCUTS
// ============================================================================

// Quick PRM planning
inline amp::Path2D planPRM(const amp::Problem2D& problem, int n = 200, double r = 1.0) {
    MyPRM prm;
    prm.setParams(n, r);
    return prm.plan(problem);
}

// Quick RRT planning  
inline amp::Path2D planRRT(const amp::Problem2D& problem) {
    MyRRT rrt;
    return rrt.plan(problem);
}

// Quick RRT with visualization
inline amp::Path2D planRRTWithViz(const amp::Problem2D& problem) {
    MyRRTWithVisualization rrt;
    return rrt.planWithVisualization(problem);
}

// Quick collision checking
inline bool checkCollision(const amp::Environment2D& env, const Eigen::Vector2d& point) {
    amp::MyCollisionChecker checker;
    return checker.isInCollision(env, point);
}

// Quick path validation
inline bool validatePath(const Eigen::Vector2d& start, const Eigen::Vector2d& end, 
                        const std::vector<amp::Obstacle2D>& obstacles,
                        double x_min, double x_max, double y_min, double y_max) {
    return amp::MyCollisionChecker::isValidPath(start, end, obstacles, x_min, x_max, y_min, y_max);
}