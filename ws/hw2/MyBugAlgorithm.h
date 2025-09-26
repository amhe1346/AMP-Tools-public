#pragma once

#include "AMPCore.h"
#include "hw/HW2.h"
#include "MyBugAlgorithm.h"


/// @brief Declare your bug algorithm class here. Note this class derives the bug algorithm class declared in HW2.h
class MyBugAlgorithm : public amp::BugAlgorithm {
    // Plots waypoints in reverse order from the closest pair to a given point, then calls bug_move
    static void reverse_bug(const amp::Problem2D& problem, amp::Path2D& path, const Eigen::Vector2d& to_point);
    // Finds the pair of waypoints with the smallest m-line (Euclidean distance)
    static std::tuple<size_t, size_t, double> find_smallest_m_line(const amp::Path2D& path);
    public:
        // Override and implement the bug algorithm in the plan method. The methods are declared here in the `.h` file
        virtual amp::Path2D plan(const amp::Problem2D& problem) override;

        // Add any other methods here...
    
    private:
        // Add any member variables here...
};