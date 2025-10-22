#pragma once

#include "AMPCore.h"
#include <iostream>

namespace amp {

/**
 * @brief Utility class for expanding obstacles by a given radius
 * Used for converting point robot problems to problems with circular agents
 */
class ObstacleExpansion {
public:
    /**
     * @brief Expand obstacles by agent radius using Minkowski sum approximation
     * @param obstacles Original obstacles
     * @param radius Agent radius to expand by
     * @return Expanded obstacles that account for agent size
     */
    static std::vector<amp::Obstacle2D> expandObstacles(const std::vector<amp::Obstacle2D>& obstacles, 
                                                        double radius) {
        std::vector<amp::Obstacle2D> expanded_obstacles;
        
        std::cout << "Expanding " << obstacles.size() << " obstacles by radius: " << radius << std::endl;
        
        for (const auto& obstacle : obstacles) {
            const auto& vertices = obstacle.verticesCCW();
            std::vector<Eigen::Vector2d> expanded_vertices;
            
            // Simple approach: offset each vertex outward by radius
            // This is an approximation - a proper Minkowski sum would be more complex
            for (size_t i = 0; i < vertices.size(); ++i) {
                const auto& curr = vertices[i];
                const auto& prev = vertices[(i - 1 + vertices.size()) % vertices.size()];
                const auto& next = vertices[(i + 1) % vertices.size()];
                
                // Calculate normal vectors from adjacent edges
                Eigen::Vector2d edge1 = (curr - prev).normalized();
                Eigen::Vector2d edge2 = (next - curr).normalized();
                
                // Calculate outward normal (average of edge normals, rotated 90 degrees)
                Eigen::Vector2d normal1(-edge1.y(), edge1.x());
                Eigen::Vector2d normal2(-edge2.y(), edge2.x());
                Eigen::Vector2d avg_normal = (normal1 + normal2).normalized();
                
                // Offset vertex by radius in normal direction
                Eigen::Vector2d expanded_vertex = curr + radius * avg_normal;
                expanded_vertices.push_back(expanded_vertex);
            }
            
            // Create new obstacle with expanded vertices
            amp::Obstacle2D expanded_obstacle(expanded_vertices);
            expanded_obstacles.push_back(expanded_obstacle);
        }
        
        return expanded_obstacles;
    }
    
    /**
     * @brief Shrink workspace boundaries by agent radius
     * @param x_min Original x minimum
     * @param x_max Original x maximum  
     * @param y_min Original y minimum
     * @param y_max Original y maximum
     * @param radius Agent radius
     */
    static void shrinkWorkspace(double& x_min, double& x_max, 
                               double& y_min, double& y_max, 
                               double radius) {
        x_min += radius;
        x_max -= radius;
        y_min += radius;
        y_max -= radius;
    }
};

} // namespace amp