#pragma once

// ============================================================================
// SHARED FOLDER - ALL IMPLEMENTATIONS
// Quick access header for all shared folder algorithms and utilities
// ============================================================================

// Core AMP includes
#include "AMPCore.h"

// ============================================================================
// COLLISION DETECTION & GEOMETRIC UTILITIES
// ============================================================================
#include "MyCollisionChecker.h"
#include "MyTranslationalCSpace.h"
#include "ObstacleExpansion.h"
#include "tools/Obstacle.h"
#include "minowskisum.h"

// ============================================================================
// SAMPLING-BASED PLANNERS (HW7)
// ============================================================================
#include "MySamplingBasedPlanners.h"
// Available classes:
// - MyPRM : public amp::PRM2D
// - MyRRT : public amp::GoalBiasRRT2D  
// - MyRRTWithVisualization : public MyRRT

// ============================================================================
// MANIPULATOR PLANNING
// ============================================================================
#include "ManipulatorSkeleton.h"

// ============================================================================
// BENCHMARKING & UTILITIES
// ============================================================================
#include "BenchmarkUtils.h"
// Note: ConvertUtils.cpp excluded to avoid function name collisions
#include "HelpfulClass.h"

// ============================================================================
// NAMESPACE ALIASES FOR CONVENIENCE
// ============================================================================
namespace shared {
    // Collision detection
    using CollisionChecker = amp::MyCollisionChecker;
    using CSpace = amp::MyTranslationalCSpace;
    
    // Sampling-based planners
    using PRM = MyPRM;
    using RRT = MyRRT;
    using RRTWithViz = MyRRTWithVisualization;
    
    // Manipulator (commented out - class not implemented yet)
    // using Manipulator = MyManipulator2Link;
}

// ============================================================================
// QUICK ACCESS FACTORY FUNCTIONS
// ============================================================================
namespace SharedFactory {
    
    // Create collision checker
    inline std::unique_ptr<amp::MyCollisionChecker> createCollisionChecker() {
        return std::make_unique<amp::MyCollisionChecker>();
    }
    
    // Create sampling-based planners
    inline std::unique_ptr<MyPRM> createPRM(int n = 200, double r = 1.0) {
        auto prm = std::make_unique<MyPRM>();
        prm->setParams(n, r);
        return prm;
    }
    
    inline std::unique_ptr<MyRRT> createRRT() {
        return std::make_unique<MyRRT>();
    }
    
    inline std::unique_ptr<MyRRTWithVisualization> createRRTWithViz() {
        return std::make_unique<MyRRTWithVisualization>();
    }
}

// ============================================================================
// COMMON PROBLEM SOLVING UTILITIES
// ============================================================================
namespace SharedUtils {
    
    // Quick problem solving with different algorithms
    inline amp::Path2D solvePRM(const amp::Problem2D& problem, int n = 200, double r = 1.0) {
        auto prm = SharedFactory::createPRM(n, r);
        return prm->plan(problem);
    }
    
    inline amp::Path2D solveRRT(const amp::Problem2D& problem) {
        auto rrt = SharedFactory::createRRT();
        return rrt->plan(problem);
    }
    
    inline amp::Path2D solveRRTWithViz(const amp::Problem2D& problem) {
        auto rrt = SharedFactory::createRRTWithViz();
        return rrt->planWithVisualization(problem);
    }
}

// ============================================================================
// USAGE EXAMPLES IN COMMENTS
// ============================================================================
/*
QUICK USAGE EXAMPLES:

// 1. Use collision checker
auto checker = SharedFactory::createCollisionChecker();
bool collision = checker->isInCollision(env, point);

// 2. Solve problem with PRM
amp::Path2D path = SharedUtils::solvePRM(problem, 500, 1.5);

// 3. Solve with RRT and visualization
amp::Path2D path = SharedUtils::solveRRTWithViz(problem);

// 4. Direct class usage with aliases
shared::PRM prm;
prm.setParams(300, 2.0);
amp::Path2D path = prm.plan(problem);

// 5. Access specific implementations
MyRRTWithVisualization rrt_viz;
amp::Path2D path = rrt_viz.planWithVisualization(problem);
*/