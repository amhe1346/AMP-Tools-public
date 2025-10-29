#pragma once

// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework headers
#include "hw/HW9.h"

class MyKinoRRT : public amp::KinodynamicRRT {
    public:
        // Default constructor with standard parameters
        MyKinoRRT() = default;
        
        // Constructor with configurable parameters
        MyKinoRRT(int max_iterations, double dt, double goal_bias, int control_samples)
            : m_max_iterations(max_iterations), m_dt(dt), m_goal_bias(goal_bias), 
              m_control_samples(control_samples) {}
        
        // Main planning function (overrides base class)
        virtual amp::KinoPath plan(const amp::KinodynamicProblem2D& problem, amp::DynamicAgent& agent) override;
        
        // Parameter configuration methods
        void setMaxIterations(int max_iterations) { m_max_iterations = max_iterations; }
        void setTimeStep(double dt) { m_dt = dt; }
        void setGoalBias(double goal_bias) { m_goal_bias = goal_bias; }
        void setControlSamples(int control_samples) { m_control_samples = control_samples; }
        
        // Parameter getters
        int getMaxIterations() const { return m_max_iterations; }
        double getTimeStep() const { return m_dt; }
        double getGoalBias() const { return m_goal_bias; }
        int getControlSamples() const { return m_control_samples; }
        
    private:
        // Configurable algorithm parameters
        int m_max_iterations = 5000;     // Maximum RRT iterations
        double m_dt = 0.1;               // Integration time step
        double m_goal_bias = 0.1;        // Goal biasing probability
        int m_control_samples = 5;       // Random controls per iteration
};

// Helper functions for Runge-Kutta 4th order integration
Eigen::VectorXd computeStateDerivative(const Eigen::VectorXd& state, const Eigen::VectorXd& control, 
                                      amp::DynamicAgent& agent);
Eigen::VectorXd integrateRK4(const Eigen::VectorXd& x0, const Eigen::VectorXd& u, 
                            double dt, amp::DynamicAgent& agent);  

class MySingleIntegrator : public amp::DynamicAgent {
    public:
        virtual void propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt) override;
};

class MyFirstOrderUnicycle : public amp::DynamicAgent {
    public:
        virtual void propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt) override;
};

class MySecondOrderUnicycle : public amp::DynamicAgent {
    public:
        virtual void propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt) override;
};

class MySimpleCar : public amp::DynamicAgent {
    public:
        virtual void propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt) override;
};