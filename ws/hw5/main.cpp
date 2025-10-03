// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework headers
#include "hw/HW5.h"
#include "hw/HW2.h"  // For accessing HW2 workspaces

// Include any custom headers you created in your workspace
#include "MyGDAlgorithm.h"
#include "MyEnhancedGDAlgorithm.h"

using namespace amp;

int main(int argc, char** argv) {
    /* Include this line to have different randomized environments every time you run your code (NOTE: this has no affect on grade()) */
    amp::RNG::seed(amp::RNG::randiUnbounded());

    // Algorithm parameters for enhanced algorithm
    struct {
        double d_star;  // Repulsive influence distance
        double zeta;    // Attractive potential gain  
        double Q_star;  // Repulsive potential gain
        double eta;     // Step size
    } variables = {0.5, 10.0, 9.0, 0.05};  // Enhanced algorithm parameters
    
    std::cout << "\n" << std::string(70, '=') << std::endl;
    std::cout << "TESTING ENHANCED ALGORITHM WITH TANGENTIAL FLOW" << std::endl;
    std::cout << std::string(70, '=') << std::endl;
    
    // ==============================================
    // ENHANCED ALGORITHM TEST 1: HW5 WORKSPACE 1
    // ==============================================
    
    Problem2D hw5_ws1_enhanced = HW5::getWorkspace1();
    std::cout << "\n=== Enhanced Algorithm on HW5 Workspace 1 ===" << std::endl;
    std::cout << "Start: (" << hw5_ws1_enhanced.q_init[0] << ", " << hw5_ws1_enhanced.q_init[1] << ")" << std::endl;
    std::cout << "Goal: (" << hw5_ws1_enhanced.q_goal[0] << ", " << hw5_ws1_enhanced.q_goal[1] << ")" << std::endl;
    std::cout << "Obstacles: " << hw5_ws1_enhanced.obstacles.size() << std::endl;
    std::cout << "Enhanced Parameters: d_star=" << variables.d_star 
              << ", zeta=" << variables.zeta 
              << ", Q_star=" << variables.Q_star 
              << ", eta=" << variables.eta << std::endl;

    MyEnhancedGDAlgorithm enhanced_algo_hw5_ws1(variables.d_star, variables.zeta, variables.Q_star, variables.eta);
    Path2D enhanced_path_hw5_ws1 = enhanced_algo_hw5_ws1.plan(hw5_ws1_enhanced);
    
    // Visualize HW5 WS1 path planning result
    std::cout << "\n=== GENERATING FIGURE 1: Enhanced Algorithm HW5 WS1 Path Planning ===" << std::endl;
    Visualizer::makeFigure(hw5_ws1_enhanced, enhanced_path_hw5_ws1);

    // Visualize HW5 WS1 vector field
    std::cout << "\n=== GENERATING FIGURE 2: Enhanced Algorithm HW5 WS1 Vector Field ===" << std::endl;
    Visualizer::makeFigure(MyEnhancedPotentialFunction{hw5_ws1_enhanced.q_goal, hw5_ws1_enhanced.obstacles, 
                                                      variables.d_star, variables.zeta, 
                                                      variables.Q_star, variables.eta}, hw5_ws1_enhanced, 30);
    
    // ==============================================
    // ENHANCED ALGORITHM TEST 2: HW2 WORKSPACE 1
    // ==============================================
    
    Problem2D hw2_ws1_enhanced = HW2::getWorkspace1();
    std::cout << "\n=== Enhanced Algorithm on HW2 Workspace 1 ===" << std::endl;
    std::cout << "Start: (" << hw2_ws1_enhanced.q_init[0] << ", " << hw2_ws1_enhanced.q_init[1] << ")" << std::endl;
    std::cout << "Goal: (" << hw2_ws1_enhanced.q_goal[0] << ", " << hw2_ws1_enhanced.q_goal[1] << ")" << std::endl;
    std::cout << "Obstacles: " << hw2_ws1_enhanced.obstacles.size() << std::endl;

    MyEnhancedGDAlgorithm enhanced_algo_hw2_ws1(variables.d_star, variables.zeta, variables.Q_star, variables.eta);
    Path2D enhanced_path_hw2_ws1 = enhanced_algo_hw2_ws1.plan(hw2_ws1_enhanced);
    
    // Visualize HW2 WS1 path planning result
    std::cout << "\n=== GENERATING FIGURE 3: Enhanced Algorithm HW2 WS1 Path Planning ===" << std::endl;
    Visualizer::makeFigure(hw2_ws1_enhanced, enhanced_path_hw2_ws1);

    // Visualize HW2 WS1 vector field
    std::cout << "\n=== GENERATING FIGURE 4: Enhanced Algorithm HW2 WS1 Vector Field ===" << std::endl;
    Visualizer::makeFigure(MyEnhancedPotentialFunction{hw2_ws1_enhanced.q_goal, hw2_ws1_enhanced.obstacles, 
                                                      variables.d_star, variables.zeta, 
                                                      variables.Q_star, variables.eta}, hw2_ws1_enhanced, 30);
    
    // ==============================================
    // ENHANCED ALGORITHM TEST 3: HW2 WORKSPACE 2
    // ==============================================
    
    Problem2D hw2_ws2_enhanced = HW2::getWorkspace2();
    std::cout << "\n=== Enhanced Algorithm on HW2 Workspace 2 ===" << std::endl;
    std::cout << "Start: (" << hw2_ws2_enhanced.q_init[0] << ", " << hw2_ws2_enhanced.q_init[1] << ")" << std::endl;
    std::cout << "Goal: (" << hw2_ws2_enhanced.q_goal[0] << ", " << hw2_ws2_enhanced.q_goal[1] << ")" << std::endl;
    std::cout << "Obstacles: " << hw2_ws2_enhanced.obstacles.size() << std::endl;

    MyEnhancedGDAlgorithm enhanced_algo_hw2_ws2(variables.d_star, variables.zeta, variables.Q_star, variables.eta);
    Path2D enhanced_path_hw2_ws2 = enhanced_algo_hw2_ws2.plan(hw2_ws2_enhanced);
    
    // Visualize HW2 WS2 path planning result
    std::cout << "\n=== GENERATING FIGURE 5: Enhanced Algorithm HW2 WS2 Path Planning ===" << std::endl;
    Visualizer::makeFigure(hw2_ws2_enhanced, enhanced_path_hw2_ws2);

    // Visualize HW2 WS2 vector field
    std::cout << "\n=== GENERATING FIGURE 6: Enhanced Algorithm HW2 WS2 Vector Field ===" << std::endl;
    Visualizer::makeFigure(MyEnhancedPotentialFunction{hw2_ws2_enhanced.q_goal, hw2_ws2_enhanced.obstacles, 
                                                      variables.d_star, variables.zeta, 
                                                      variables.Q_star, variables.eta}, hw2_ws2_enhanced, 30);
    
    std::cout << "\n" << std::string(70, '=') << std::endl;
    std::cout << "ALL ENHANCED ALGORITHM TESTS COMPLETED!" << std::endl;
    std::cout << "The enhanced algorithm uses tangential flow from furthest to closest obstacle points." << std::endl;
    std::cout << std::string(70, '=') << std::endl;

    Visualizer::saveFigures(true, "hw5_figs");
    
    // Arguments following argv correspond to the constructor arguments of MyGDAlgorithm:
    HW5::grade<MyGDAlgorithm>("amy.heerten@colorado.edu", argc, argv, 
                              variables.d_star, variables.zeta, variables.Q_star, variables.eta);
    return 0;
}