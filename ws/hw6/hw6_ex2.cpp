// HW6 Exercise 2: Wave-front planner for manipulator motion planning
#include "AMPCore.h"
#include "hw/HW4.h"
#include "hw/HW6.h"
#include "MyCSConstructors.h"
#include "MyAStar.h"
#include "ManipulatorSkeleton.h"

using namespace amp;

int main(int argc, char** argv) {
    std::cout << "=== HW6 Exercise 2: Manipulator Motion Planning with Wave-Front ===" << std::endl;
    
    // Create a 2-link manipulator
    MyManipulator2D manipulator;
    
    // Get the workspace from HW4 Exercise 3
    Environment2D env = HW4::getEx3Workspace1();
    
    // Create a problem with the workspace and desired end-effector positions
    Problem2D manip_problem;
    manip_problem.obstacles = env.obstacles;
    manip_problem.x_min = -M_PI;
    manip_problem.x_max = M_PI;
    manip_problem.y_min = -M_PI;
    manip_problem.y_max = M_PI;
    
    // Define start and goal in workspace (end-effector positions)
    Eigen::Vector2d x_start(-2.0, 0.0);
    Eigen::Vector2d x_goal(2.0, 0.0);
    
    std::cout << "\nStart position (end-effector): " << x_start.transpose() << std::endl;
    std::cout << "Goal position (end-effector): " << x_goal.transpose() << std::endl;
    
    std::cout << "\nManipulator info:" << std::endl;
    std::cout << "Number of links: " << manipulator.nLinks() << std::endl;
    std::cout << "Link lengths: ";
    for (const auto& len : manipulator.getLinkLengths()) {
        std::cout << len << " ";
    }
    std::cout << std::endl;
    
    // Set the problem's start and goal as WORKSPACE POSITIONS (not joint configs)
    // The ManipulatorWaveFrontAlgorithm will call IK internally
    manip_problem.q_init = x_start;
    manip_problem.q_goal = x_goal;
    
    // Construct the C-space using the manipulator C-space constructor and wavefront algorithm
    std::size_t cells_per_dim = 500; // High resolution for better visualization
    std::shared_ptr<MyManipulatorCSConstructor> manipulator_ctor = std::make_shared<MyManipulatorCSConstructor>(cells_per_dim);
    std::shared_ptr<WaveFrontAlgorithm> wf_algo = std::make_shared<MyWaveFrontAlgorithm>();
    
    std::cout << "\nConstructing C-space with " << cells_per_dim << " cells per dimension..." << std::endl;
    ManipulatorWaveFrontAlgorithm manip_algo(wf_algo, manipulator_ctor);
    
    std::cout << "C-space constructed!" << std::endl;
    
    // Plan using wave-front algorithm
    std::cout << "\nPlanning path using Wave-Front algorithm..." << std::endl;
    ManipulatorTrajectory2Link trajectory = manip_algo.plan(manipulator, manip_problem);
    
    std::cout << "Path found with " << trajectory.waypoints.size() << " waypoints" << std::endl;
    
    // Calculate path length
    double path_length = 0.0;
    for (size_t i = 1; i < trajectory.waypoints.size(); ++i) {
        path_length += (trajectory.waypoints[i] - trajectory.waypoints[i-1]).norm();
    }
    std::cout << "Path length in C-space: " << path_length << std::endl;
    
    // Visualize the C-space with the path
    std::cout << "\nVisualizing C-space with path..." << std::endl;
    Path2D cspace_path;
    cspace_path.waypoints = trajectory.waypoints;
    Visualizer::makeFigure(*manip_algo.getCSpace(), cspace_path);
    
    // Visualize the manipulator motion in workspace
    std::cout << "Visualizing manipulator motion in workspace..." << std::endl;
    Visualizer::makeFigure(manip_problem, manipulator, trajectory);
    
    std::cout << "\nSaving all figures..." << std::endl;
    Visualizer::saveFigures();
    
    std::cout << "\n=== HW6 Exercise 2 Complete ===" << std::endl;
    std::cout << "Figures saved:" << std::endl;
    std::cout << "1. C-space plot with planned path (joint space visualization)" << std::endl;
    std::cout << "2. Workspace with manipulator animation showing motion from start to goal" << std::endl;
    
    return 0;
}
