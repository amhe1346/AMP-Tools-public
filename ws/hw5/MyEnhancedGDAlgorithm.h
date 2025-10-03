#pragma once

// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW5.h"

class MyEnhancedPotentialFunction : public amp::PotentialFunction2D {
private:
    Eigen::Vector2d goal;
    std::vector<amp::Obstacle2D> obstacles;
    double d_star;    // Repulsive influence distance
    double zeta;      // Attractive potential gain
    double Q_star;    // Repulsive potential strength
    double eta;       // Step size (not used in potential function but kept for consistency)
    
public:
    // Constructor with goal, obstacles, and algorithm parameters
    MyEnhancedPotentialFunction(const Eigen::Vector2d& g, const std::vector<amp::Obstacle2D>& obs, 
                       double d_star_param, double zeta_param, double Q_star_param, double eta_param) 
        : goal(g), obstacles(obs), d_star(d_star_param), zeta(zeta_param), 
          Q_star(Q_star_param), eta(eta_param) {}
    
    // Constructor to set just the goal (no repulsive forces)
    MyEnhancedPotentialFunction(const Eigen::Vector2d& g) 
        : goal(g), d_star(1.0), zeta(1.0), Q_star(1.0), eta(1.0) {}
    
    // Default constructor (points to origin)
    MyEnhancedPotentialFunction() 
        : goal(Eigen::Vector2d::Zero()), d_star(1.0), zeta(1.0), Q_star(1.0), eta(1.0) {}
    
    virtual double operator()(const Eigen::Vector2d& q) const override {
        // U_att = 0.5 * zeta * ||q - goal||²
        Eigen::Vector2d diff = q - goal;
        double U_att = 0.5 * zeta * (diff[0] * diff[0] + diff[1] * diff[1]);
        
        // U_rep = sum over obstacles of 0.5 * Q_star * (1/d - 1/d_star)²
        double U_rep = 0.0;
        for (const auto& obstacle : obstacles) {
            double d = distanceToObstacle(q, obstacle);
            if (d > 0 && d <= d_star) {  // Only apply repulsive force within influence distance
                double term = (1.0/d - 1.0/d_star);
                U_rep += 0.5 * Q_star * term * term;
            }
        }
        
        return U_att + U_rep;  // Total potential = attractive + repulsive
    }

    virtual Eigen::Vector2d getGradient(const Eigen::Vector2d& q) const override {
        // ∇U_att = -zeta * (q - goal) (movement toward goal)
        Eigen::Vector2d grad_att = -zeta * (q - goal);
        
        // ∇U_rep = sum over obstacles of repulsive gradient
        Eigen::Vector2d grad_rep = Eigen::Vector2d::Zero();
        for (const auto& obstacle : obstacles) {
            double d = distanceToObstacle(q, obstacle);
            if (d > 0 && d <= d_star) {
                // Direction away from obstacle
                Eigen::Vector2d closest = findClosestPointOnObstacle(q, obstacle);
                Eigen::Vector2d direction = (q - closest);
                if (direction.norm() > 1e-10) {
                    direction.normalize();
                } else {
                    // If exactly on obstacle, push in random direction
                    direction = Eigen::Vector2d::Random().normalized();
                }
                
                // Find furthest and closest points on obstacle relative to goal
                auto [furthest_point, closest_point_to_goal] = findExtremePointsOnObstacle(obstacle, goal);
                
                // Create tangential flow from furthest to closest point
                Eigen::Vector2d flow_direction = (closest_point_to_goal - furthest_point);
                if (flow_direction.norm() > 1e-10) {
                    flow_direction.normalize();
                } else {
                    // Fallback to traditional repulsive direction if points are too close
                    flow_direction = direction;
                }
                
                // Blend traditional repulsive force with tangential flow
                double blend_factor = 0.7;  // How much tangential flow vs repulsive force
                Eigen::Vector2d combined_direction = (blend_factor * flow_direction + (1.0 - blend_factor) * direction).normalized();
                
                // Magnitude of repulsive gradient
                double term = (1.0/d - 1.0/d_star);
                double magnitude = Q_star * term / (d * d);  // Positive magnitude
                grad_rep += magnitude * combined_direction;
            }
        }
        
        return grad_att + grad_rep;  // Total gradient = attractive + repulsive
    }
    
private:
    // Calculate minimum distance from point to obstacle boundary
    double distanceToObstacle(const Eigen::Vector2d& q, const amp::Obstacle2D& obstacle) const {
        std::vector<Eigen::Vector2d> vertices = obstacle.verticesCCW();
        double min_dist = std::numeric_limits<double>::max();
        
        for (size_t i = 0; i < vertices.size(); ++i) {
            size_t j = (i + 1) % vertices.size();
            
            // Distance to line segment
            Eigen::Vector2d edge = vertices[j] - vertices[i];
            Eigen::Vector2d to_point = q - vertices[i];
            
            double edge_length_sq = edge.squaredNorm();
            if (edge_length_sq < 1e-10) {
                // Degenerate edge
                min_dist = std::min(min_dist, to_point.norm());
            } else {
                double t = to_point.dot(edge) / edge_length_sq;
                t = std::max(0.0, std::min(1.0, t));
                
                Eigen::Vector2d closest = vertices[i] + t * edge;
                min_dist = std::min(min_dist, (q - closest).norm());
            }
        }
        
        return min_dist;
    }
    
    // Find closest point on obstacle for gradient direction
    Eigen::Vector2d findClosestPointOnObstacle(const Eigen::Vector2d& q, const amp::Obstacle2D& obstacle) const {
        std::vector<Eigen::Vector2d> vertices = obstacle.verticesCCW();
        Eigen::Vector2d closest_point;
        double min_dist = std::numeric_limits<double>::max();
        
        for (size_t i = 0; i < vertices.size(); ++i) {
            size_t j = (i + 1) % vertices.size();
            
            Eigen::Vector2d edge = vertices[j] - vertices[i];
            Eigen::Vector2d to_point = q - vertices[i];
            
            double edge_length_sq = edge.squaredNorm();
            if (edge_length_sq < 1e-10) {
                if (to_point.norm() < min_dist) {
                    min_dist = to_point.norm();
                    closest_point = vertices[i];
                }
            } else {
                double t = to_point.dot(edge) / edge_length_sq;
                t = std::max(0.0, std::min(1.0, t));
                
                Eigen::Vector2d point_on_edge = vertices[i] + t * edge;
                double dist = (q - point_on_edge).norm();
                if (dist < min_dist) {
                    min_dist = dist;
                    closest_point = point_on_edge;
                }
            }
        }
        
        return closest_point;
    }
    
    // Find the furthest and closest points on obstacle relative to goal
    std::pair<Eigen::Vector2d, Eigen::Vector2d> findExtremePointsOnObstacle(const amp::Obstacle2D& obstacle, const Eigen::Vector2d& reference_point) const {
        std::vector<Eigen::Vector2d> vertices = obstacle.verticesCCW();
        
        Eigen::Vector2d furthest_point = vertices[0];
        Eigen::Vector2d closest_point = vertices[0];
        double max_dist = (vertices[0] - reference_point).norm();
        double min_dist = max_dist;
        
        // Check all vertices
        for (const auto& vertex : vertices) {
            double dist = (vertex - reference_point).norm();
            if (dist > max_dist) {
                max_dist = dist;
                furthest_point = vertex;
            }
            if (dist < min_dist) {
                min_dist = dist;
                closest_point = vertex;
            }
        }
        
        // Also check points along edges (sample at regular intervals)
        for (size_t i = 0; i < vertices.size(); ++i) {
            size_t j = (i + 1) % vertices.size();
            Eigen::Vector2d edge = vertices[j] - vertices[i];
            
            // Sample along the edge at multiple points
            for (int k = 1; k < 10; ++k) {  // Sample 9 points along each edge
                double t = k / 10.0;
                Eigen::Vector2d point_on_edge = vertices[i] + t * edge;
                double dist = (point_on_edge - reference_point).norm();
                
                if (dist > max_dist) {
                    max_dist = dist;
                    furthest_point = point_on_edge;
                }
                if (dist < min_dist) {
                    min_dist = dist;
                    closest_point = point_on_edge;
                }
            }
        }
        
        return std::make_pair(furthest_point, closest_point);
    }
};

class MyEnhancedGDAlgorithm : public amp::GDAlgorithm {
public:
    // Constructor to easily tune parameters
    MyEnhancedGDAlgorithm(double d_star, double zetta, double Q_star, double eta) :
        d_star(d_star),
        zetta(zetta),
        Q_star(Q_star),
        eta(eta) {}

    // Override this method to solve a given problem.
    virtual amp::Path2D plan(const amp::Problem2D& problem) override {
        amp::Path2D path;
        
        // Start from initial position
        Eigen::Vector2d current = problem.q_init;
        path.waypoints.push_back(current);
        
        // Create enhanced potential function for vector field with tangential flow
        MyEnhancedPotentialFunction potential(problem.q_goal, problem.obstacles, d_star, zetta, Q_star, eta);
        
        std::cout << "Running enhanced algorithm with tangential flow..." << std::endl;
        
        // Follow the vector field step by step
        for (int i = 0; i < 1000; ++i) {
            // Get the vector field direction at current position
            Eigen::Vector2d gradient = potential.getGradient(current);
            double grad_magnitude = gradient.norm();
            
            // Check for convergence
            if (grad_magnitude < 1e-5 && (current - problem.q_goal).norm() < 0.2) {
                path.waypoints.push_back(problem.q_goal);
                std::cout << "Converged to goal!" << std::endl;
                break;
            }
            
            // Normalize the gradient to get direction
            Eigen::Vector2d direction = gradient.normalized();
            
            // Take a step in the direction of the vector field
            Eigen::Vector2d step = eta * direction;
            Eigen::Vector2d new_position = current + step;
            
            // Keep within workspace boundaries
            new_position[0] = std::max(problem.x_min, std::min(problem.x_max, new_position[0]));
            new_position[1] = std::max(problem.y_min, std::min(problem.y_max, new_position[1]));
            
            current = new_position;
            path.waypoints.push_back(current);
            
            // Check if we reached the goal
            double dist_to_goal = (current - problem.q_goal).norm();
            if (dist_to_goal < 0.1) {
                path.waypoints.push_back(problem.q_goal);
                std::cout << "Reached goal!" << std::endl;
                break;
            }
            
            // Debug output every 100 steps
            if (i % 100 == 0) {
                std::cout << "Step " << i << ": (" << current[0] << ", " << current[1] 
                          << "), dist to goal: " << dist_to_goal << std::endl;
            }
        }
        
        return path;
    }
    
private:
    double d_star, zetta, Q_star, eta;
};