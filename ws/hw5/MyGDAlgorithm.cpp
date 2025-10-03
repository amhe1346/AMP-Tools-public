#include "MyGDAlgorithm.h"
#include <iostream>
#include <algorithm>

// Implement gradient descent following 2D vector field
amp::Path2D MyGDAlgorithm::plan(const amp::Problem2D& problem) {
    amp::Path2D path;
    
    // Start from initial position
    Eigen::Vector2d current = problem.q_init;
    path.waypoints.push_back(current);
    
    // Create potential function for vector field
    MyPotentialFunction potential(problem.q_goal, problem.obstacles, d_star, zetta, Q_star, eta);
    
 
    
    // Oscillation detection
    Eigen::Vector2d last_position = current;
    Eigen::Vector2d two_steps_ago = current;
    int stuck_counter = 0;
    int oscillation_count = 0;
    double adaptive_eta = eta;
    
    // Follow the vector field step by step
    for (int i = 0; i < 500; ++i) {
        // Get the vector field direction at current position
        Eigen::Vector2d gradient = potential.getGradient(current);
        double grad_magnitude = gradient.norm();
        
        // Check for convergence (very small gradient)
        if (grad_magnitude < 1e-5) {
            std::cout << "Converged at step " << i << " (gradient magnitude: " << grad_magnitude << ")" << std::endl;
            break;
        }
        
        // Normalize the gradient to get direction
        Eigen::Vector2d direction = gradient.normalized();
        
        // Detect oscillation (returning to a previous position)
        if (i >= 2 && (current - two_steps_ago).norm() < 0.01) {
            oscillation_count++;
            if (oscillation_count >= 3) {
                std::cout << "  OSCILLATION DETECTED! Reducing step size from " << adaptive_eta;
                adaptive_eta *= 0.5;
                std::cout << " to " << adaptive_eta << std::endl;
                oscillation_count = 0;
                
                if (adaptive_eta < 1e-6) {
                    std::cout << "Step size too small, convergence assumed" << std::endl;
                    break;
                }
            }
        } else {
            oscillation_count = 0;
        }
        
        // Take a step of adaptive size in the direction of the vector field
        Eigen::Vector2d step = adaptive_eta * direction;
        Eigen::Vector2d new_position = current + step;
        
        // Debug: Show position changes
        if (i % 50 == 0) {
            std::cout << "  Before step: (" << current[0] << ", " << current[1] << ")" << std::endl;
            std::cout << "  After step: (" << new_position[0] << ", " << new_position[1] << ")" << std::endl;
            std::cout << "  Workspace bounds: x[" << problem.x_min << ", " << problem.x_max 
                      << "] y[" << problem.y_min << ", " << problem.y_max << "]" << std::endl;
        }
        
        current = new_position;
        
        // Keep within workspace boundaries
        Eigen::Vector2d clamped_current = current;
        clamped_current[0] = std::max(problem.x_min, std::min(problem.x_max, current[0]));
        clamped_current[1] = std::max(problem.y_min, std::min(problem.y_max, current[1]));
        
        if (i % 50 == 0) {
            std::cout << "  After clamping: (" << clamped_current[0] << ", " << clamped_current[1] << ")" << std::endl;
        }
        
        current = clamped_current;
        
        // Update position history for oscillation detection
        two_steps_ago = last_position;
        last_position = current;
        
        // Check if we're stuck in local minima
        if ((current - last_position).norm() < 0.001) {  // Very small movement
            stuck_counter++;
        } else {
            stuck_counter = 0;
        }
        
        // If stuck for too long, add random perturbation
        if (stuck_counter > 20) {
            std::cout << "Local minimum detected at step " << i << "! Adding random perturbation" << std::endl;
            Eigen::Vector2d random_dir = Eigen::Vector2d::Random().normalized();
            current += 0.1 * random_dir;  // Small random jump
            
            // Clamp after perturbation
            current[0] = std::max(problem.x_min, std::min(problem.x_max, current[0]));
            current[1] = std::max(problem.y_min, std::min(problem.y_max, current[1]));
            
            stuck_counter = 0;
            last_position = current;
        }
        
        // Add the new point to the path
        path.waypoints.push_back(current);
        
        // Check if we reached the goal
        double dist_to_goal = (current - problem.q_goal).norm();
        if (dist_to_goal < 0.1) {
            path.waypoints.push_back(problem.q_goal);  // Ensure we end exactly at the goal
            std::cout << "Reached goal at step " << i << " (distance: " << dist_to_goal << ")" << std::endl;
            break;
        }
        
        // Print detailed progress every 50 steps
        if (i % 50 == 0) {
            std::cout << "Step " << i << ": pos(" << current[0] << ", " << current[1] 
                      << ") gradient_mag=" << grad_magnitude << " dist_to_goal=" << dist_to_goal << std::endl;
            std::cout << "  Gradient: (" << gradient[0] << ", " << gradient[1] << ")" << std::endl;
            std::cout << "  Direction: (" << direction[0] << ", " << direction[1] << ")" << std::endl;
            std::cout << "  Adaptive eta: " << adaptive_eta << std::endl;
            std::cout << "  Step: (" << step[0] << ", " << step[1] << ")" << std::endl;
        }
    }
    
    std::cout << "Path completed with " << path.waypoints.size() << " points" << std::endl;
    return path;
}