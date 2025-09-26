#pragma once

// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW4.h"

// Derive the amp::LinkManipulator2D class
class MyManipulator2D : public amp::LinkManipulator2D {
    public:
        // Default constructor
        MyManipulator2D();
        
        // Constructor with custom link lengths
        MyManipulator2D(const std::vector<double>& link_lengths);

        // Override this method for implementing forward kinematics
        virtual Eigen::Vector2d getJointLocation(const amp::ManipulatorState& state, uint32_t joint_index) const override;

        // Override this method for implementing inverse kinematics
        virtual amp::ManipulatorState getConfigurationFromIK(const Eigen::Vector2d& end_effector_location) const override;
        
        // Additional utility methods
        Eigen::Vector2d getEndEffectorLocation(const amp::ManipulatorState& state) const;
        
        // Interactive user interface methods
        void forwardKinematicsMode() const;
        void inverseKinematicsMode() const;
        void printAssumptions() const;
        
        // Static factory methods for creating custom manipulators
        static MyManipulator2D createCustomManipulator(const std::vector<double>& link_lengths);
        static MyManipulator2D createManipulatorFromUserInput();
};