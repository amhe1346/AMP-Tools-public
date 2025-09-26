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
        // 3-link IK - treat first two links as 2-link, third as orientation
        double L1 = getLinkLengths()[0];
        double L2 = getLinkLengths()[1];
        double L3 = getLinkLengths()[2];
        
        // For 3-link, use geometric decomposition
        // Compute wrist position by backing off from end effector
        double wrist_x = x - L3 * cos(atan2(y, x));
        double wrist_y = y - L3 * sin(atan2(y, x));
        
        // Solve 2-link problem to wrist
        double wrist_distance = sqrt(wrist_x*wrist_x + wrist_y*wrist_y);
        
        state.resize(3);
        
        if (wrist_distance > L1 + L2) {
            // Unreachable - point towards target
            state[0] = atan2(y, x);
            state[1] = 0.0;
            state[2] = 0.0;
            return state;
        }
        
        if (wrist_distance < fabs(L1 - L2)) {
            // Too close - fold first two links
            state[0] = atan2(wrist_y, wrist_x);
            state[1] = (L1 > L2) ? M_PI : -M_PI;
            state[2] = 0.0;
            return state;
        }
        
        // Solve for first two joints using 2-link solution
        double cos_q2 = (wrist_distance*wrist_distance - L1*L1 - L2*L2) / (2*L1*L2);
        cos_q2 = std::max(-1.0, std::min(1.0, cos_q2));
        double q2 = acos(cos_q2);
        
        double alpha = atan2(wrist_y, wrist_x);
        double beta = acos((L1*L1 + wrist_distance*wrist_distance - L2*L2) / (2*L1*wrist_distance));
        beta = std::max(0.0, std::min(M_PI, beta));
        double q1 = alpha - beta;
        
        state[0] = q1;
        state[1] = q2;
        
        // Third joint aligns end effector towards original target
        double cumulative_angle = q1 + q2;
        double desired_angle = atan2(y, x);
        state[2] = desired_angle - cumulative_angle;
        
        // Normalize angles to [-π, π]
        while (state[2] > M_PI) state[2] -= 2*M_PI;
        while (state[2] < -M_PI) state[2] += 2*M_PI;
        
    } else {
        // Fallback for other number of links
        state.setZero(nLinks());
        double target_angle = atan2(y, x);
        if (nLinks() > 0) state[0] = target_angle;
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