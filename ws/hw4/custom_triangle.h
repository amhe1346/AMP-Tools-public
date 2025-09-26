#pragma once

#include "AMPCore.h"
#include "tools/Environment.h"
#include "tools/Obstacle.h"

namespace amp {
    // Custom triangle obstacle functions
    Obstacle2D getCustomTriangle1();
    Obstacle2D getCustomTriangle2(); 
    Obstacle2D getLargeTriangle();
    
    // Custom environment with triangles
    Environment2D getCustomTriangleEnvironment(int triangle_type = 1);
    
    // Debug and analysis functions
    void printTriangleVertices();
    std::vector<Eigen::Vector2d> computeCSpaceObstacle(const std::vector<Eigen::Vector2d>& obstacle_vertices);
    
    // Minkowski sum visualization
    Environment2D getMinkowskiSumVisualization();
    
    // 3D C-space with rotation (SE(2))
    Eigen::Vector2d rotatePoint(const Eigen::Vector2d& point, double theta);
    std::vector<std::tuple<double, double, double>> compute3DCSpaceObstacle(const std::vector<Eigen::Vector2d>& obstacle_vertices);
    std::vector<Environment2D> get3DCSpaceSlices();
    Environment2D get3DStackedVisualization();
    void export3DCSpaceData(const std::string& filename);
}