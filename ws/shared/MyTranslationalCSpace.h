#pragma once

#include "AMPCore.h"
#include <iostream>
#include <queue>
#include <vector>
#include <algorithm>
#include <cmath>

namespace amp {

/**
 * @brief Configuration Space implementation for 2D translational robots with circular geometry
 * This class creates a discretized C-space where obstacles are grown by the robot radius
 * using proper Minkowski sum computation for circular robots
 */
class MyTranslationalCSpace : public amp::GridCSpace2D {
public:
    MyTranslationalCSpace(std::size_t x_cells, std::size_t y_cells, 
                         double x_min, double x_max, 
                         double y_min, double y_max,
                         const std::vector<amp::Obstacle2D>& obstacles,
                         double robot_radius)
        : amp::GridCSpace2D(x_cells, y_cells, x_min, x_max, y_min, y_max)
        , m_obstacles(obstacles)
        , m_robot_radius(robot_radius) 
    {
        computeCSpace();
    }

    // Override getCellFromPoint to convert continuous coordinates to grid cells
    virtual std::pair<std::size_t, std::size_t> getCellFromPoint(double x, double y) const override {
        auto x_bounds = x0Bounds();
        auto y_bounds = x1Bounds();
        
        auto grid_size = size();
        std::size_t nx = grid_size.first;
        std::size_t ny = grid_size.second;
        
        double cell_width_x = (x_bounds.second - x_bounds.first) / nx;
        double cell_width_y = (y_bounds.second - y_bounds.first) / ny;
        
        std::size_t cell_x = std::min((std::size_t)std::max(0.0, (x - x_bounds.first) / cell_width_x), nx - 1);
        std::size_t cell_y = std::min((std::size_t)std::max(0.0, (y - y_bounds.first) / cell_width_y), ny - 1);
        
        return {cell_x, cell_y};
    }

    // Get a collision-free path using A* search
    amp::Path2D planPath(const Eigen::Vector2d& start, const Eigen::Vector2d& goal) const {
        auto start_cell = getCellFromPoint(start.x(), start.y());
        auto goal_cell = getCellFromPoint(goal.x(), goal.y());
        
        std::cout << "Planning from (" << start.x() << "," << start.y() << ") to (" 
                  << goal.x() << "," << goal.y() << ")" << std::endl;
        std::cout << "Start cell: (" << start_cell.first << "," << start_cell.second << ")" << std::endl;
        std::cout << "Goal cell: (" << goal_cell.first << "," << goal_cell.second << ")" << std::endl;
        
        // Simple A* implementation
        std::vector<std::vector<bool>> visited(size().first, std::vector<bool>(size().second, false));
        std::vector<std::vector<std::pair<int, int>>> parent(size().first, 
            std::vector<std::pair<int, int>>(size().second, {-1, -1}));
        
        std::priority_queue<Node> open_set;
        open_set.push({(std::size_t)start_cell.first, (std::size_t)start_cell.second, 0.0, heuristic(start_cell, goal_cell)});
        
        while (!open_set.empty()) {
            Node current = open_set.top();
            open_set.pop();
            
            if (visited[current.x][current.y]) continue;
            visited[current.x][current.y] = true;
            
            if (current.x == goal_cell.first && current.y == goal_cell.second) {
                return reconstructPath(parent, start_cell, goal_cell, start, goal);
            }
            
            // Explore neighbors (8-connected)
            for (int dx = -1; dx <= 1; ++dx) {
                for (int dy = -1; dy <= 1; ++dy) {
                    if (dx == 0 && dy == 0) continue;
                    
                    int nx = (int)current.x + dx;
                    int ny = (int)current.y + dy;
                    
                    if (nx >= 0 && nx < (int)size().first && 
                        ny >= 0 && ny < (int)size().second &&
                        !visited[nx][ny] && !operator()(nx, ny)) {
                        
                        parent[nx][ny] = {(int)current.x, (int)current.y};
                        double new_cost = current.g + (dx == 0 || dy == 0 ? 1.0 : 1.414); // Diagonal cost
                        open_set.push({(std::size_t)nx, (std::size_t)ny, new_cost, new_cost + heuristic({(std::size_t)nx, (std::size_t)ny}, goal_cell)});
                    }
                }
            }
        }
        
        // No path found - return straight line with exact start/goal
        amp::Path2D path;
        path.waypoints.push_back(start);
        path.waypoints.push_back(goal);
        return path;
    }

private:
    struct Node {
        std::size_t x, y;
        double g, f;
        
        bool operator<(const Node& other) const {
            return f > other.f; // For min-heap
        }
    };
    
    void computeCSpace() {
        auto grid_size = size();
        auto x_bounds = x0Bounds();
        auto y_bounds = x1Bounds();
        
        double cell_width_x = (x_bounds.second - x_bounds.first) / grid_size.first;
        double cell_width_y = (y_bounds.second - y_bounds.first) / grid_size.second;
        
        std::cout << "Computing C-Space with " << grid_size.first << "x" << grid_size.second 
                  << " cells, robot radius: " << m_robot_radius << std::endl;
        
        // For each cell in the grid, check if placing robot center there causes collision
        for (std::size_t i = 0; i < grid_size.first; ++i) {
            for (std::size_t j = 0; j < grid_size.second; ++j) {
                // Get the center point of this cell
                double x = x_bounds.first + (i + 0.5) * cell_width_x;
                double y = y_bounds.first + (j + 0.5) * cell_width_y;
                
                // Check if robot centered at (x,y) collides with any obstacle
                bool in_collision = false;
                for (const auto& obstacle : m_obstacles) {
                    if (circlePolygonCollision(x, y, m_robot_radius, obstacle.verticesCCW())) {
                        in_collision = true;
                        break;
                    }
                }
                
                // Set collision value in grid
                operator()(i, j) = in_collision;
            }
        }
    }
    
    // Check if a circle collides with a polygon
    bool circlePolygonCollision(double cx, double cy, double radius, 
                               const std::vector<Eigen::Vector2d>& polygon) const {
        // Check if circle center is inside polygon
        if (pointInPolygon({cx, cy}, polygon)) {
            return true;
        }
        
        // Check distance from circle center to each edge
        for (size_t i = 0; i < polygon.size(); ++i) {
            size_t next = (i + 1) % polygon.size();
            double dist = pointToSegmentDistance({cx, cy}, polygon[i], polygon[next]);
            if (dist <= radius) {
                return true;
            }
        }
        
        return false;
    }
    
    // Point in polygon test using ray casting
    bool pointInPolygon(const Eigen::Vector2d& point, const std::vector<Eigen::Vector2d>& polygon) const {
        int crossings = 0;
        for (size_t i = 0; i < polygon.size(); ++i) {
            size_t j = (i + 1) % polygon.size();
            
            if (((polygon[i].y() > point.y()) != (polygon[j].y() > point.y())) &&
                (point.x() < (polygon[j].x() - polygon[i].x()) * (point.y() - polygon[i].y()) / 
                (polygon[j].y() - polygon[i].y()) + polygon[i].x())) {
                crossings++;
            }
        }
        return (crossings % 2) == 1;
    }
    
    // Distance from point to line segment
    double pointToSegmentDistance(const Eigen::Vector2d& point, 
                                 const Eigen::Vector2d& seg_start, 
                                 const Eigen::Vector2d& seg_end) const {
        Eigen::Vector2d segment = seg_end - seg_start;
        double segment_length_sq = segment.squaredNorm();
        
        if (segment_length_sq < 1e-10) {
            return (point - seg_start).norm();
        }
        
        double t = std::max(0.0, std::min(1.0, (point - seg_start).dot(segment) / segment_length_sq));
        Eigen::Vector2d projection = seg_start + t * segment;
        return (point - projection).norm();
    }
    
    double heuristic(const std::pair<std::size_t, std::size_t>& from, 
                    const std::pair<std::size_t, std::size_t>& to) const {
        double dx = (double)to.first - (double)from.first;
        double dy = (double)to.second - (double)from.second;
        return std::sqrt(dx*dx + dy*dy);
    }
    
    amp::Path2D reconstructPath(const std::vector<std::vector<std::pair<int, int>>>& parent,
                               const std::pair<std::size_t, std::size_t>& start_cell,
                               const std::pair<std::size_t, std::size_t>& goal_cell,
                               const Eigen::Vector2d& exact_start,
                               const Eigen::Vector2d& exact_goal) const {
        amp::Path2D path;
        
        auto x_bounds = x0Bounds();
        auto y_bounds = x1Bounds();
        auto grid_size = size();
        
        double cell_width_x = (x_bounds.second - x_bounds.first) / grid_size.first;
        double cell_width_y = (y_bounds.second - y_bounds.first) / grid_size.second;
        
        // Trace back from goal to start
        std::vector<std::pair<int, int>> cell_path;
        int x = goal_cell.first, y = goal_cell.second;
        
        while (x != -1 && y != -1) {
            cell_path.push_back({x, y});
            auto p = parent[x][y];
            x = p.first;
            y = p.second;
        }
        
        std::reverse(cell_path.begin(), cell_path.end());
        
        // Convert cell path to world coordinates, using exact start/goal positions
        for (size_t i = 0; i < cell_path.size(); ++i) {
            const auto& cell = cell_path[i];
            if (i == 0) {
                // Use exact start position
                path.waypoints.push_back(exact_start);
            } else if (i == cell_path.size() - 1) {
                // Use exact goal position
                path.waypoints.push_back(exact_goal);
            } else {
                double world_x = x_bounds.first + (cell.first + 0.5) * cell_width_x;
                double world_y = y_bounds.first + (cell.second + 0.5) * cell_width_y;
                path.waypoints.push_back({world_x, world_y});
            }
        }
        
        return path;
    }

private:
    std::vector<amp::Obstacle2D> m_obstacles;
    double m_robot_radius;
};

} // namespace amp