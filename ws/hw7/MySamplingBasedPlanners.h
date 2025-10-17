#pragma once

// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework headers
#include "hw/HW7.h"

class MyPRM : public amp::PRM2D {
    public:
        virtual amp::Path2D plan(const amp::Problem2D& problem) override;
        std::shared_ptr<amp::Graph<double>> getGraph() const { return graphPtr_; }
        const std::map<amp::Node, Eigen::Vector2d>& getNodes() const { return nodes_; }
        void setParams(int n, double r) { n_ = n; r_ = r; }
    private:
        int n_ = 200;
        double r_ = 1.0;
        std::shared_ptr<amp::Graph<double>> graphPtr_;
        std::map<amp::Node, Eigen::Vector2d> nodes_;
};

class MyRRT : public amp::GoalBiasRRT2D {
    public:
        virtual amp::Path2D plan(const amp::Problem2D& problem) override;
        
    private:
        bool isValidPath(const Eigen::Vector2d& start, const Eigen::Vector2d& end, const amp::Problem2D& problem);
};

class MYcollisionChecker {
    public:
        MYcollisionChecker() = default;
        ~MYcollisionChecker() = default;
        
        bool isValidPath(const Eigen::Vector2d& start, const Eigen::Vector2d& end, const amp::Problem2D& problem);
};  