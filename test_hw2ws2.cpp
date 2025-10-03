#include <iostream>

// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework headers
#include "hw/HW5.h"
#include "hw/HW2.h"  // For accessing HW2 workspaces

// Include the enhanced algorithm from hw2ws2.cpp
#include "hw2ws2.cpp"

using namespace amp;

int main(int argc, char** argv) {
    /* Include this line to have different randomized environments every time you run your code */
    amp::RNG::seed(amp::RNG::randiUnbounded());

    // Algorithm parameters - optimized for HW2 WS2
    struct {
        double d_star;  // Repulsive influence distance
        double zeta;    // Attractive potential gain  
        double Q_star;  // Repulsive potential gain
        double eta;     // Step size
    } variables = {0.5, 10.0, 5.0, 0.05};  // Parameters optimized for HW2 WS2

    // Get HW2 Workspace 2 problem
    Problem2D prob = HW2::getWorkspace2();
    std::cout << "=== Testing Enhanced MyGDAlgorithm on HW2 Workspace 2 ===" << std::endl;
    std::cout << "Start: (" << prob.q_init[0] << ", " << prob.q_init[1] << ")" << std::endl;
    std::cout << "Goal: (" << prob.q_goal[0] << ", " << prob.q_goal[1] << ")" << std::endl;
    std::cout << "Obstacles: " << prob.obstacles.size() << std::endl;
    std::cout << "Parameters: d_star=" << variables.d_star 
              << ", zeta=" << variables.zeta 
              << ", Q_star=" << variables.Q_star 
              << ", eta=" << variables.eta << std::endl;

    // Test your enhanced gradient descent algorithm with tangential flow
    MyGDAlgorithm algo(variables.d_star, variables.zeta, variables.Q_star, variables.eta);
    
    std::cout << "\nRunning enhanced algorithm with tangential flow..." << std::endl;
    Path2D path = algo.plan(prob);
    
    // Check if the algorithm reached the goal
    if (path.waypoints.size() > 0) {
        Eigen::Vector2d last_waypoint = path.waypoints.back();
        double dist_to_goal = (last_waypoint - prob.q_goal).norm();
        std::cout << "\n=== RESULTS ===" << std::endl;
        std::cout << "Path waypoints: " << path.waypoints.size() << std::endl;
        std::cout << "Last waypoint: (" << last_waypoint[0] << ", " << last_waypoint[1] << ")" << std::endl;
        std::cout << "Goal: (" << prob.q_goal[0] << ", " << prob.q_goal[1] << ")" << std::endl;
        std::cout << "Distance to goal: " << dist_to_goal << std::endl;
        std::cout << "Reaches goal: " << (dist_to_goal < 0.1 ? "YES" : "NO") << std::endl;
        
        // Calculate path length
        double path_length = 0.0;
        for (size_t i = 1; i < path.waypoints.size(); ++i) {
            path_length += (path.waypoints[i] - path.waypoints[i-1]).norm();
        }
        std::cout << "Total path length: " << path_length << std::endl;
    } else {
        std::cout << "ERROR: No path generated!" << std::endl;
        return 1;
    }
    
    // Generate visualizations
    std::cout << "\n=== GENERATING VISUALIZATIONS ===" << std::endl;
    
    // FIGURE 1: Path Planning Result with Enhanced Algorithm
    std::cout << "Generating Figure 1: Enhanced Path Planning Result" << std::endl;
    Visualizer::makeFigure(prob, path);

    // FIGURE 2: Vector Field Visualization with Tangential Flow
    std::cout << "Generating Figure 2: Enhanced Vector Field with Tangential Flow" << std::endl;
    Visualizer::makeFigure(MyPotentialFunction{prob.q_goal, prob.obstacles, 
                                              variables.d_star, variables.zeta, 
                                              variables.Q_star, variables.eta}, prob, 30);

    // Save figures
    Visualizer::saveFigures(true, "hw2ws2_enhanced_test");
    
    std::cout << "\nTest completed! Check the generated figures to see the enhanced algorithm's performance." << std::endl;
    std::cout << "The algorithm uses tangential flow around obstacles for better navigation." << std::endl;
    
    return 0;
}