#include "ManipulatorSkeleton.h"

MyManipulator2D::MyManipulator2D()
    : LinkManipulator2D({1.0, 1.0, 1.0}) // Default to a 3-link with all links of 1.0 length
{}

MyManipulator2D::MyManipulator2D(const std::vector<double>& link_lengths) 
    : LinkManipulator2D(link_lengths) {}

// Override this method for implementing forward kinematics
Eigen::Vector2d MyManipulator2D::getJointLocation(const amp::ManipulatorState& state, uint32_t joint_index) const {
    // If requesting joint 0 (base), it's at origin
    if (joint_index == 0) {
        return Eigen::Vector2d::Zero();
    }

    // Calculate cumulative angle and position for joint_index
    double cumulative_angle = 0.0;
    Eigen::Vector2d position = Eigen::Vector2d::Zero();
    
    for (uint32_t i = 0; i < joint_index; ++i) {
        cumulative_angle += state[i];
        
        // Add the length of link i in the direction of cumulative angle
        position.x() += getLinkLengths()[i] * cos(cumulative_angle);
        position.y() += getLinkLengths()[i] * sin(cumulative_angle);
    }
    
    return position;
}

// Override this method for implementing inverse kinematics
amp::ManipulatorState MyManipulator2D::getConfigurationFromIK(const Eigen::Vector2d& end_effector_location) const {
    amp::ManipulatorState state;
    double x = end_effector_location.x();
    double y = end_effector_location.y();
    
    if (nLinks() == 2) {
        // 2-link IK - analytical solution
        double L1 = getLinkLengths()[0];
        double L2 = getLinkLengths()[1];
        
        double distance = sqrt(x*x + y*y);
        
        if (distance > L1 + L2) {
            // Target unreachable - point towards target
            state.resize(2);
            state[0] = atan2(y, x);
            state[1] = 0.0;
            return state;
        }
        
        if (distance < fabs(L1 - L2)) {
            // Target too close - fold manipulator
            state.resize(2);
            state[0] = atan2(y, x);
            state[1] = (L1 > L2) ? M_PI : -M_PI;
            return state;
        }
        
        // Use law of cosines for elbow angle
        double cos_theta2 = (distance*distance - L1*L1 - L2*L2) / (2*L1*L2);
        cos_theta2 = std::max(-1.0, std::min(1.0, cos_theta2)); // Clamp
        double theta2 = acos(cos_theta2);
        
        // Calculate shoulder angle
        double alpha = atan2(y, x);
        double beta = acos((L1*L1 + distance*distance - L2*L2) / (2*L1*distance));
        beta = std::max(0.0, std::min(M_PI, beta)); // Clamp
        double theta1 = alpha - beta;
        
        state.resize(2);
        state[0] = theta1;
        state[1] = theta2;
        
    } else if (nLinks() == 3) {
        // 3-link IK - more precise analytical approach
        double L1 = getLinkLengths()[0];
        double L2 = getLinkLengths()[1];
        double L3 = getLinkLengths()[2];
        
        state.resize(3);
        
        double distance = sqrt(x*x + y*y);
        double max_reach = L1 + L2 + L3;
        double min_reach = fabs(fabs(L1 - L2) - L3);
        
        // Check reachability
        if (distance > max_reach - 1e-6) {
            // Unreachable - fully extend towards target
            double target_angle = atan2(y, x);
            state[0] = target_angle;
            state[1] = 0.0;
            state[2] = 0.0;
            return state;
        }
        
        if (distance < min_reach + 1e-6) {
            // Target too close - fold manipulator
            double target_angle = atan2(y, x);
            state[0] = target_angle;
            state[1] = M_PI;
            state[2] = -M_PI;
            return state;
        }
        
        // For 3-link manipulator, we need a more sophisticated approach
        // Method: Parameterize the problem and solve numerically
        
        double target_angle = atan2(y, x);
        
        // Try multiple configurations and pick the best one
        double best_error = 1e6;
        amp::ManipulatorState best_state(3);
        
        // Strategy 1: Use first two links to get close, third to fine-tune
        for (int attempt = 0; attempt < 8; attempt++) {
            amp::ManipulatorState candidate(3);
            
            // Try different ways to distribute the reach
            double reach_ratio = 0.5 + 0.1 * (attempt - 4); // Range from 0.1 to 0.9
            reach_ratio = std::max(0.1, std::min(0.9, reach_ratio));
            
            // Target point for first two links
            double intermediate_distance = distance * reach_ratio;
            double intermediate_x = x * reach_ratio;
            double intermediate_y = y * reach_ratio;
            
            // Solve 2-link problem for first two joints
            if (intermediate_distance > L1 + L2 - 1e-6) {
                // Extend first two links
                candidate[0] = atan2(intermediate_y, intermediate_x);
                candidate[1] = 0.0;
            } else if (intermediate_distance < fabs(L1 - L2) + 1e-6) {
                // Fold first two links
                candidate[0] = atan2(intermediate_y, intermediate_x);
                candidate[1] = (L1 > L2) ? M_PI : -M_PI;
            } else {
                // Normal 2-link solution
                double cos_theta2 = (intermediate_distance*intermediate_distance - L1*L1 - L2*L2) / (2*L1*L2);
                cos_theta2 = std::max(-1.0, std::min(1.0, cos_theta2));
                double theta2 = acos(cos_theta2);
                
                double alpha = atan2(intermediate_y, intermediate_x);
                double beta = atan2(L2*sin(theta2), L1 + L2*cos(theta2));
                candidate[0] = alpha - beta;
                candidate[1] = theta2;
            }
            
            // Now compute where the second joint is
            double second_joint_x = L1 * cos(candidate[0]);
            double second_joint_y = L1 * sin(candidate[0]);
            
            double third_joint_x = second_joint_x + L2 * cos(candidate[0] + candidate[1]);
            double third_joint_y = second_joint_y + L2 * sin(candidate[0] + candidate[1]);
            
            // Vector from third joint to target
            double dx = x - third_joint_x;
            double dy = y - third_joint_y;
            double remaining_distance = sqrt(dx*dx + dy*dy);
            
            if (remaining_distance > L3 + 1e-6) {
                // Cannot reach target from third joint
                candidate[2] = atan2(dy, dx) - (candidate[0] + candidate[1]);
            } else if (remaining_distance < 1e-6) {
                // Already at target
                candidate[2] = 0.0;
            } else {
                // Third link can reach target
                candidate[2] = atan2(dy, dx) - (candidate[0] + candidate[1]);
            }
            
            // Normalize angles
            for (int i = 0; i < 3; i++) {
                while (candidate[i] > M_PI) candidate[i] -= 2*M_PI;
                while (candidate[i] < -M_PI) candidate[i] += 2*M_PI;
            }
            
            // Compute error
            Eigen::Vector2d achieved_pos = getEndEffectorLocation(candidate);
            double error = (Eigen::Vector2d(x, y) - achieved_pos).norm();
            
            if (error < best_error) {
                best_error = error;
                best_state = candidate;
            }
        }
        
        state = best_state;
        
    } else {
        // Fallback for n-link manipulators (n > 3)
        state.setZero(nLinks());
        
        // Use a simple heuristic: distribute angles to point towards target
        double target_angle = atan2(y, x);
        double total_length = 0.0;
        for (const auto& length : getLinkLengths()) {
            total_length += length;
        }
        
        double distance = sqrt(x*x + y*y);
        
        if (distance > total_length) {
            // Target unreachable - extend all joints towards target
            for (int i = 0; i < nLinks(); ++i) {
                state[i] = target_angle / nLinks();
            }
        } else if (distance < 1e-6) {
            // Target at origin - fold the manipulator
            for (int i = 1; i < nLinks(); ++i) {
                state[i] = M_PI / (nLinks() - 1);
            }
        } else {
            // Use a simple strategy: first joint points toward target,
            // others adjust to reach the correct distance
            state[0] = target_angle * 0.7; // Primary direction
            
            // Distribute remaining angle adjustment among other joints
            double remaining_angle = target_angle - state[0];
            for (int i = 1; i < nLinks(); ++i) {
                state[i] = remaining_angle / (nLinks() - 1) * 0.5;
            }
            
            // Fine-tune based on distance
            double scale = std::min(1.0, distance / total_length);
            for (int i = 1; i < nLinks(); ++i) {
                state[i] *= scale;
            }
        }
    }
    
    return state;
}

Eigen::Vector2d MyManipulator2D::getEndEffectorLocation(const amp::ManipulatorState& state) const {
    return getJointLocation(state, nLinks());
}

void MyManipulator2D::forwardKinematicsMode() const {
    std::cout << "\n=== FORWARD KINEMATICS MODE ===" << std::endl;
    std::cout << "This is a " << nLinks() << "-link planar manipulator" << std::endl;
    std::cout << "Link lengths: ";
    for (size_t i = 0; i < getLinkLengths().size(); ++i) {
        std::cout << getLinkLengths()[i];
        if (i < getLinkLengths().size() - 1) std::cout << ", ";
    }
    std::cout << std::endl;
}

void MyManipulator2D::inverseKinematicsMode() const {
    std::cout << "\n=== INVERSE KINEMATICS MODE ===" << std::endl;
    std::cout << "This is a " << nLinks() << "-link planar manipulator" << std::endl;
    std::cout << "Link lengths: ";
    for (size_t i = 0; i < getLinkLengths().size(); ++i) {
        std::cout << getLinkLengths()[i];
        if (i < getLinkLengths().size() - 1) std::cout << ", ";
    }
    std::cout << std::endl;
}

void MyManipulator2D::printAssumptions() const {
    std::cout << "\n=== MANIPULATOR IMPLEMENTATION ASSUMPTIONS ===" << std::endl;
    std::cout << "• Planar manipulator operating in 2D workspace" << std::endl;
    std::cout << "• All joints are revolute" << std::endl;
    std::cout << "• Base is fixed at origin (0, 0)" << std::endl;
    std::cout << "• Forward kinematics uses cumulative angle approach" << std::endl;
    std::cout << "• Inverse kinematics:" << std::endl;
    std::cout << "  - 2-link: Analytical solution using law of cosines" << std::endl;
    std::cout << "  - 3-link: Geometric decomposition approach" << std::endl;
    std::cout << "• Link lengths: [";
    for (size_t i = 0; i < getLinkLengths().size(); ++i) {
        std::cout << getLinkLengths()[i];
        if (i < getLinkLengths().size() - 1) std::cout << ", ";
    }
    std::cout << "]" << std::endl;
}

MyManipulator2D MyManipulator2D::createCustomManipulator(const std::vector<double>& link_lengths) {
    return MyManipulator2D(link_lengths);
}

MyManipulator2D MyManipulator2D::createManipulatorFromUserInput() {
    std::vector<double> link_lengths;
    int num_links;
    
    std::cout << "Enter number of links: ";
    std::cin >> num_links;
    
    link_lengths.resize(num_links);
    for (int i = 0; i < num_links; ++i) {
        std::cout << "Enter length for link " << (i + 1) << ": ";
        std::cin >> link_lengths[i];
    }
    
    return MyManipulator2D(link_lengths);
}