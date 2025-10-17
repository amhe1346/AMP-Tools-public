// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"
#include "hw/HW2.h"
#include "hw/HW5.h"
#include "MySamplingBasedPlanners.h"




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

    

    // Generate a random problem and test RRT
    MyRRT rrt;
    Path2D path = rrt.plan(problem1);
    // If you want to test RRT on HW2 Workspace 2, use problem2
    HW7::generateAndCheck(rrt, path, problem1);
    Visualizer::makeFigure(problem1, path); // Only visualize the path, no graph/nodes
    Visualizer::saveFigures();


    // Grade method
    HW7::grade<MyPRM, MyRRT>("amy.heerten@colorado.edu", argc, argv, std::make_tuple(), std::make_tuple());
    return 0;
}