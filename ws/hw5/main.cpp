// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework headers
#include "hw/HW5.h"
#include "hw/HW2.h"  // For accessing HW2 workspaces

// Include any custom headers you created in your workspace
#include "MyGDAlgorithm.h"

using namespace amp;

int main(int argc, char** argv) {
    /* Include this line to have different randomized environments every time you run your code (NOTE: this has no affect on grade()) */
    amp::RNG::seed(amp::RNG::randiUnbounded());

    // Algorithm parameters - easy to change in one place
    struct {
        double d_star;  // Repulsive influence distance
        double zeta;    // Attractive potential gain  
        double Q_star;  // Repulsive potential gain
        double eta;     // Step size
        //hw 2 {.5, 10.0, 5.0, 0.05};
    } variables = {.5, 10.0, 15.0, 0.05};  // Balanced forces: smaller d_star, higher zeta, moderate Q_star

    // Use HW2 Workspace 1 for testing
    Problem2D prob = HW2::getWorkspace1();
    std::cout << "Using HW2 Workspace 1:" << std::endl;
    std::cout << "Start: (" << prob.q_init[0] << ", " << prob.q_init[1] << ")" << std::endl;
    std::cout << "Goal: (" << prob.q_goal[0] << ", " << prob.q_goal[1] << ")" << std::endl;
    std::cout << "Obstacles: " << prob.obstacles.size() << std::endl;
    std::cout << "Parameters: d_star=" << variables.d_star 
              << ", zeta=" << variables.zeta 
              << ", Q_star=" << variables.Q_star 
              << ", eta=" << variables.eta << std::endl;

    // Test your gradient descent algorithm using the variables
    MyGDAlgorithm algo(variables.d_star, variables.zeta, variables.Q_star, variables.eta);
    Path2D path = algo.plan(prob);
    
    // Visualize the problem and solution path
    Visualizer::makeFigure(prob, path);

    // Visualize your potential function with attractive + repulsive forces using algorithm parameters
    Visualizer::makeFigure(MyPotentialFunction{prob.q_goal, prob.obstacles, 
                                              variables.d_star, variables.zeta, 
                                              variables.Q_star, variables.eta}, prob, 30);

    // Test HW5 Workspace 1
    Problem2D hw5_prob = HW5::getWorkspace1();
    std::cout << "\n=== HW5 Workspace 1 ===" << std::endl;
    std::cout << "Start: (" << hw5_prob.q_init[0] << ", " << hw5_prob.q_init[1] << ")" << std::endl;
    std::cout << "Goal: (" << hw5_prob.q_goal[0] << ", " << hw5_prob.q_goal[1] << ")" << std::endl;
    std::cout << "Obstacles: " << hw5_prob.obstacles.size() << std::endl;

    MyGDAlgorithm algo_hw5(variables.d_star, variables.zeta, variables.Q_star, variables.eta);
    Path2D path_hw5 = algo_hw5.plan(hw5_prob);
    
    // Visualize HW5 workspace problem and solution path
    Visualizer::makeFigure(hw5_prob, path_hw5);

    // Visualize HW5 workspace 2D vector field
    Visualizer::makeFigure(MyPotentialFunction{hw5_prob.q_goal, hw5_prob.obstacles, 
                                              variables.d_star, variables.zeta, 
                                              variables.Q_star, variables.eta}, hw5_prob, 30);

    // Test HW2 Workspace 2
    Problem2D hw2_ws2_prob = HW2::getWorkspace2();
    std::cout << "\n=== HW2 Workspace 2 ===" << std::endl;
    std::cout << "Start: (" << hw2_ws2_prob.q_init[0] << ", " << hw2_ws2_prob.q_init[1] << ")" << std::endl;
    std::cout << "Goal: (" << hw2_ws2_prob.q_goal[0] << ", " << hw2_ws2_prob.q_goal[1] << ")" << std::endl;
    std::cout << "Obstacles: " << hw2_ws2_prob.obstacles.size() << std::endl;

    MyGDAlgorithm algo_hw2_ws2(variables.d_star, variables.zeta, variables.Q_star, variables.eta);
    Path2D path_hw2_ws2 = algo_hw2_ws2.plan(hw2_ws2_prob);
    
    // Visualize HW2 workspace 2 problem and solution path
    Visualizer::makeFigure(hw2_ws2_prob, path_hw2_ws2);

    // Visualize HW2 workspace 2 2D vector field
    Visualizer::makeFigure(MyPotentialFunction{hw2_ws2_prob.q_goal, hw2_ws2_prob.obstacles, 
                                              variables.d_star, variables.zeta, 
                                              variables.Q_star, variables.eta}, hw2_ws2_prob, 30);
    
    Visualizer::saveFigures(true, "hw5_figs");
    
    // Arguments following argv correspond to the constructor arguments of MyGDAlgorithm:
    HW5::grade<MyGDAlgorithm>("nonhuman.biologic@myspace.edu", argc, argv, 
                              variables.d_star, variables.zeta, variables.Q_star, variables.eta);
    return 0;
}