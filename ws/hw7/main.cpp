// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"
#include "hw/HW2.h"
#include "hw/HW5.h"
#include "MySamplingBasedPlanners.h"
#include <chrono>




using namespace amp;

int main(int argc, char** argv) {
    HW7::hint(); // Consider implementing an N-dimensional planner 

    // Remove unused points/edges demo
    // Test PRM on Workspace1 of HW2

    // Test PRM on HW2 Workspace 1
    Problem2D problem1 = HW2::getWorkspace1();
    MyPRM prm1;
    Path2D prm_path1 = prm1.plan(problem1);
    Visualizer::makeFigure(problem1, prm_path1, *prm1.getGraph(), prm1.getNodes());

    // Test PRM on HW2 Workspace 2
    Problem2D problem2 = HW2::getWorkspace2();
    MyPRM prm2;
    Path2D prm_path2 = prm2.plan(problem2);
    Visualizer::makeFigure(problem2, prm_path2, *prm2.getGraph(), prm2.getNodes());

    // Solve HW5 Exercise 2(a) with custom boundaries

    amp::Problem2D problem = HW5::getWorkspace1();
    problem.x_min = -1.0;
    problem.x_max = 11.0;
    problem.y_min = -3.0;
    problem.y_max = 3.0;

    MyPRM prm;
    prm.setParams(200, 1.0);
    amp::Path2D prm_path = prm.plan(problem);

    std::string title = "PRM HW5 Ex2(a): Path length = " + std::to_string(prm_path.length());
    Visualizer::makeFigure(problem, prm_path, *prm.getGraph(), prm.getNodes());
    std::cout << title << std::endl;

    Visualizer::saveFigures();
    

    // Generate a random problem and test RRT
    MyRRT rrt;
    Path2D rrt_path = rrt.plan(problem1);
    // If you want to test RRT on HW2 Workspace 2, use problem2
    HW7::generateAndCheck(rrt, rrt_path, problem1);
    Visualizer::makeFigure(problem1, rrt_path); // Only visualize the path, no graph/nodes
    Visualizer::saveFigures();


    // Generate sample visualizations for benchmark results
    // Run a few examples with different parameters to visualize
    std::cout << "Generating sample visualizations for benchmark..." << std::endl;

    std::vector<std::pair<int, double>> sample_params = {
        {200, 0.5}, {200, 1}, {200, 1.5}, {200, 2},
        {500, 0.5}, {500, 1}, {500, 1.5}, {500, 2}
    };
    for (const auto& [n, r] : sample_params) {
        MyPRM sample_prm;
        sample_prm.setParams(n, r);
        auto start = std::chrono::high_resolution_clock::now();
        Path2D sample_path = sample_prm.plan(problem);
        auto end = std::chrono::high_resolution_clock::now();
        double time_ms = std::chrono::duration<double, std::milli>(end - start).count();
        std::string title = "PRM n=" + std::to_string(n) + " r=" + std::to_string(r) + 
                           " length=" + std::to_string(sample_path.length()) + 
                           " time=" + std::to_string(time_ms) + "ms";
        std::cout << title << std::endl;
        Visualizer::makeFigure(problem, sample_path, *sample_prm.getGraph(), sample_prm.getNodes());
    }
    Visualizer::saveFigures();

    // Grade method
    HW7::grade<MyPRM, MyRRT>("amy.heerten@colorado.edu", argc, argv, std::make_tuple(), std::make_tuple());
    return 0;
}