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
    } variables = {.5, 10.0, 5.0, 0.05};  // Balanced forces: smaller d_star, higher zeta, moderate Q_star

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
    
    Visualizer::saveFigures(true, "hw5_figs");
    
    // Arguments following argv correspond to the constructor arguments of MyGDAlgorithm:
    HW5::grade<MyGDAlgorithm>("nonhuman.biologic@myspace.edu", argc, argv, 
                              variables.d_star, variables.zeta, variables.Q_star, variables.eta);
    return 0;
}