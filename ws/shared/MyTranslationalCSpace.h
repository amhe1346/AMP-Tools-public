#include "HelpfulClass.h"
#pragma once

#include "AMPCore.h"
#include "minowskisum.h"
#include "MySamplingBasedPlanners.h"
#include "MyCollisionChecker.h"
#include <iostream>
#include <queue>
#include <vector>
#include <algorithm>
#include <cmath>

namespace amp {

// Forward declaration
class MyTranslationalCSpace;

/**
 * @brief RRT planner that uses C-space grid for collision checking
 */
class MyCSpaceRRT {
public:
    MyCSpaceRRT(const MyTranslationalCSpace* cspace) : m_cspace(cspace) {}
    
    amp::Path2D planInCSpace(const amp::Problem2D& problem);
    
private:
    const MyTranslationalCSpace* m_cspace;
    bool isValidPoint(const Eigen::Vector2d& point) const;
    bool isValidPath(const Eigen::Vector2d& start, const Eigen::Vector2d& end) const;
};

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

    // Check if a point is in collision using the C-space grid
    bool inCollision(double x, double y) const {
        auto cell = getCellFromPoint(x, y);
        return operator()(cell.first, cell.second);
    }

    // Get a collision-free path using MyGoalBiasRRT with visualization
    amp::Path2D planPath(const Eigen::Vector2d& start, const Eigen::Vector2d& goal) const {
        std::cout << "C-Space planning from (" << start.x() << "," << start.y() << ") to (" 
                  << goal.x() << "," << goal.y() << ") using MyCSpaceRRT" << std::endl;

        // Create a problem for the point robot in the expanded obstacle space
        amp::Problem2D point_robot_problem;
        point_robot_problem.q_init = start;
        point_robot_problem.q_goal = goal;
        point_robot_problem.x_min = x0Bounds().first;
        point_robot_problem.x_max = x0Bounds().second;
        point_robot_problem.y_min = x1Bounds().first;
        point_robot_problem.y_max = x1Bounds().second;
        point_robot_problem.obstacles.clear(); // Obstacles are not used in C-space grid

        // Use MyCSpaceRRT for planning with C-space collision checking
        MyCSpaceRRT cspace_rrt(this);
        amp::Path2D path = cspace_rrt.planInCSpace(point_robot_problem);

        // Create visualizations as figures
        createVisualizationFigures(start, goal, path);

        // Ensure path starts and ends at exact positions
        if (!path.waypoints.empty()) {
            path.waypoints.front() = start;
            path.waypoints.back() = goal;
        }

        return path;
    }
    
    // Create visualization figures showing C-space and RRT path
    void createVisualizationFigures(const Eigen::Vector2d& start, const Eigen::Vector2d& goal, const amp::Path2D& path) const {
        // Only create figures for the first few planning instances to avoid too many files
        static int figure_count = 0;
        if (figure_count < 3) {
            // Create C-space visualization
            amp::Visualizer::makeFigure(*this);
            
            // Create C-space with path visualization
            if (!path.waypoints.empty()) {
                amp::Visualizer::makeFigure(*this, path);
            }
            figure_count++;
        }
        
        // Print summary information
        auto grid_size = size();
        auto x_bounds = x0Bounds();
        auto y_bounds = x1Bounds();
        
        std::cout << "\n=== C-SPACE ANALYSIS ===" << std::endl;
        std::cout << "Grid size: " << grid_size.first << "x" << grid_size.second << std::endl;
        std::cout << "Bounds: X[" << x_bounds.first << "," << x_bounds.second << "] Y[" 
                  << y_bounds.first << "," << y_bounds.second << "]" << std::endl;
        std::cout << "Robot radius: " << m_robot_radius << std::endl;
        std::cout << "Start: (" << start.x() << "," << start.y() << ")" << std::endl;
        std::cout << "Goal: (" << goal.x() << "," << goal.y() << ")" << std::endl;
        
        // Check if start/goal are in collision
        auto start_cell = getCellFromPoint(start.x(), start.y());
        auto goal_cell = getCellFromPoint(goal.x(), goal.y());
        bool start_collision = operator()(start_cell.first, start_cell.second);
        bool goal_collision = operator()(goal_cell.first, goal_cell.second);
        
        std::cout << "Start cell (" << start_cell.first << "," << start_cell.second << ") collision: " 
                  << (start_collision ? "YES" : "NO") << std::endl;
        std::cout << "Goal cell (" << goal_cell.first << "," << goal_cell.second << ") collision: " 
                  << (goal_collision ? "YES" : "NO") << std::endl;
        
        // Count free vs occupied cells
        int free_cells = 0, occupied_cells = 0;
        for (std::size_t i = 0; i < grid_size.first; ++i) {
            for (std::size_t j = 0; j < grid_size.second; ++j) {
                if (operator()(i, j)) occupied_cells++;
                else free_cells++;
            }
        }
        
        double free_percentage = 100.0 * free_cells / (free_cells + occupied_cells);
        std::cout << "Free cells: " << free_cells << " (" << free_percentage << "%)" << std::endl;
        std::cout << "Occupied cells: " << occupied_cells << std::endl;
        
        if (!path.waypoints.empty()) {
            std::cout << "Path found with " << path.waypoints.size() << " waypoints" << std::endl;
        } else {
            std::cout << "No path found!" << std::endl;
        }
        
        std::cout << "========================\n" << std::endl;
    }
    
    // Print a section of the grid for visual debugging
    void printGridSection(const Eigen::Vector2d& start, const Eigen::Vector2d& goal, int radius) const {
        auto grid_size = size();
        auto start_cell = getCellFromPoint(start.x(), start.y());
        auto goal_cell = getCellFromPoint(goal.x(), goal.y());
        
        std::cout << "\nGrid section (. = free, # = occupied, S = start, G = goal):" << std::endl;
        
        int min_i = std::max(0, (int)std::min(start_cell.first, goal_cell.first) - radius);
        int max_i = std::min((int)grid_size.first - 1, (int)std::max(start_cell.first, goal_cell.first) + radius);
        int min_j = std::max(0, (int)std::min(start_cell.second, goal_cell.second) - radius);
        int max_j = std::min((int)grid_size.second - 1, (int)std::max(start_cell.second, goal_cell.second) + radius);
        
        for (int j = max_j; j >= min_j; j--) { // Print from top to bottom
            for (int i = min_i; i <= max_i; i++) {
                if (i == start_cell.first && j == start_cell.second) {
                    std::cout << "S";
                } else if (i == goal_cell.first && j == goal_cell.second) {
                    std::cout << "G";
                } else if (operator()(i, j)) {
                    std::cout << "#";
                } else {
                    std::cout << ".";
                }
            }
            std::cout << std::endl;
        }
    }

private:
    void computeCSpace() {
        auto grid_size = size();
        auto x_bounds = x0Bounds();
        auto y_bounds = x1Bounds();
        double cell_width_x = (x_bounds.second - x_bounds.first) / grid_size.first;
        double cell_width_y = (y_bounds.second - y_bounds.first) / grid_size.second;
        std::cout << "Computing C-Space with " << grid_size.first << "x" << grid_size.second
                  << " cells, robot radius: " << m_robot_radius << std::endl;

        // --- Minkowski sum expansion ---
        // Approximate the robot as a circle polygon
        std::vector<Eigen::Vector2d> robot_poly = amp::makeCirclePolygon(Eigen::Vector2d(0,0), m_robot_radius, 20);
        // Expand each obstacle using Minkowski sum
        std::vector<std::vector<Eigen::Vector2d>> expanded_obstacles;
        for (const auto& obs : m_obstacles) {
            expanded_obstacles.push_back(amp::computeCSpaceObstacle(obs.verticesCCW(), robot_poly));
        }

        // For each cell in the grid, check if the cell center is inside any expanded obstacle
        for (std::size_t i = 0; i < grid_size.first; ++i) {
            for (std::size_t j = 0; j < grid_size.second; ++j) {
                double x = x_bounds.first + (i + 0.5) * cell_width_x;
                double y = y_bounds.first + (j + 0.5) * cell_width_y;
                Eigen::Vector2d pt(x, y);
                bool in_collision = false;
                for (const auto& expanded_obs : expanded_obstacles) {
                    if (pointInPolygon(pt, expanded_obs)) {
                        in_collision = true;
                        break;
                    }
                }
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

private:
    std::vector<amp::Obstacle2D> m_obstacles;
    double m_robot_radius;
};

// Implementation of C-space RRT planner
inline bool MyCSpaceRRT::isValidPoint(const Eigen::Vector2d& point) const {
    // Use the C-space collision checker
    amp::MyCSpaceCollisionChecker checker(m_cspace);
    return checker.isValidPoint(point);
}

inline bool MyCSpaceRRT::isValidPath(const Eigen::Vector2d& start, const Eigen::Vector2d& end) const {
    // Use the C-space collision checker
    amp::MyCSpaceCollisionChecker checker(m_cspace);
    return checker.isValidPath(start, end);
}

inline amp::Path2D MyCSpaceRRT::planInCSpace(const amp::Problem2D& problem) {
    std::cout << "\n=== C-SPACE RRT PLANNING (using MyCSpaceRRT wrapper) ===" << std::endl;
    // Custom RRT loop using C-space collision checker
    const int max_iterations = 10000;
    const double step_size = 0.1;
    const double goal_bias = 0.3;
    const double goal_threshold = 0.2;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> x_dist(problem.x_min, problem.x_max);
    std::uniform_real_distribution<double> y_dist(problem.y_min, problem.y_max);
    std::uniform_real_distribution<double> uniform(0.0, 1.0);

    struct TreeNode {
        Eigen::Vector2d position;
        int parent_index;
        TreeNode(const Eigen::Vector2d& pos, int parent) : position(pos), parent_index(parent) {}
    };

    std::vector<TreeNode> tree;
    tree.emplace_back(problem.q_init, -1);

    for (int iter = 0; iter < max_iterations; ++iter) {
        Eigen::Vector2d q_rand;
        if (uniform(gen) < goal_bias) {
            q_rand = problem.q_goal;
        } else {
            q_rand = Eigen::Vector2d(x_dist(gen), y_dist(gen));
        }

        // Find nearest node
        int nearest_index = 0;
        double min_distance = (tree[0].position - q_rand).norm();
        for (size_t i = 1; i < tree.size(); ++i) {
            double distance = (tree[i].position - q_rand).norm();
            if (distance < min_distance) {
                min_distance = distance;
                nearest_index = i;
            }
        }

        Eigen::Vector2d q_near = tree[nearest_index].position;
        Eigen::Vector2d direction = q_rand - q_near;
        double distance = direction.norm();
        Eigen::Vector2d q_new;
        if (distance <= step_size) {
            q_new = q_rand;
        } else {
            q_new = q_near + (direction / distance) * step_size;
        }

        // Use C-space collision checker
        if (isValidPoint(q_new) && isValidPath(q_near, q_new)) {
            tree.emplace_back(q_new, nearest_index);
            if ((q_new - problem.q_goal).norm() < goal_threshold) {
                // Reconstruct path
                std::vector<Eigen::Vector2d> waypoints;
                int current_index = tree.size() - 1;
                while (current_index != -1) {
                    waypoints.push_back(tree[current_index].position);
                    current_index = tree[current_index].parent_index;
                }
                std::reverse(waypoints.begin(), waypoints.end());
                amp::Path2D path;
                path.waypoints = waypoints;
                return path;
            }
        }
    }
    // No path found
    amp::Path2D path;
    path.waypoints.push_back(problem.q_init);
    return path;
}

} // namespace amp