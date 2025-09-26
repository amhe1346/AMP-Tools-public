// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW4.h"

// Include the headers for HW4 code
#include "CSpaceSkeleton.h"
#include "ManipulatorSkeleton.h"

using namespace amp;

int main(int argc, char** argv) {
    /* Include this line to have different randomized environments every time you run your code (NOTE: this has no affect on grade()) */
    amp::RNG::seed(amp::RNG::randiUnbounded());

    MyManipulator2D manipulator;
        manipulator.forwardKinematicsMode();
        manipulator.inverseKinematicsMode();
        manipulator.printAssumptions();
    
        // Test custom kinematics scenarios

    std::cout << "=== CUSTOM MANIPULATOR KINEMATICS ===" << std::endl;
    
    // ===== CUSTOM FORWARD KINEMATICS =====
    std::cout << "\n--- Custom Forward Kinematics ---" << std::endl;
    
    // Specify custom link lengths and joint angles
    std::vector<double> custom_lengths = {.5, 1, .5};  // Modify these lengths as needed
    amp::ManipulatorState custom_angles(3);
    custom_angles[0] = M_PI/4;    // 45 degrees
    custom_angles[1] = M_PI/6;    // 30 degrees  
    custom_angles[2] = 7*M_PI/4;   // 315 
    
    // Create manipulator with custom lengths
    MyManipulator2D custom_manipulator = MyManipulator2D::createCustomManipulator(custom_lengths);
    
    std::cout << "Link lengths: [" << custom_lengths[0] << ", " << custom_lengths[1] << ", " << custom_lengths[2] << "]" << std::endl;
    std::cout << "Joint angles: [" << custom_angles[0] << ", " << custom_angles[1] << ", " << custom_angles[2] << "] radians" << std::endl;
    std::cout << "Joint angles: [" << custom_angles[0]*180/M_PI << "°, " << custom_angles[1]*180/M_PI << "°, " << custom_angles[2]*180/M_PI << "°]" << std::endl;
    
    // Compute forward kinematics
    Eigen::Vector2d end_effector_pos = custom_manipulator.getEndEffectorLocation(custom_angles);
    std::cout << "End effector position: (" << end_effector_pos.x() << ", " << end_effector_pos.y() << ")" << std::endl;
    
    // Visualize the result
    Visualizer::makeFigure(custom_manipulator, custom_angles);
    std::cout << "✓ Forward kinematics visualization created" << std::endl;
    
    // ===== CUSTOM INVERSE KINEMATICS =====
    std::cout << "\n--- Custom Inverse Kinematics ---" << std::endl;
    
    // Specify different custom link lengths and target endpoint
    std::vector<double> ik_lengths = {1, .5, 1};  // Modify these lengths as needed
    Eigen::Vector2d target_point(2.0, 0.0);            // Modify this target as needed
    
    // Create manipulator with custom lengths for IK
    MyManipulator2D ik_manipulator = MyManipulator2D::createCustomManipulator(ik_lengths);
    
    std::cout << "Link lengths: [" << ik_lengths[0] << ", " << ik_lengths[1] << ", " << ik_lengths[2] << "]" << std::endl;
    std::cout << "Target endpoint: (" << target_point.x() << ", " << target_point.y() << ")" << std::endl;
    
    // Solve inverse kinematics
    amp::ManipulatorState ik_solution = ik_manipulator.getConfigurationFromIK(target_point);
    std::cout << "IK solution: [" << ik_solution[0] << ", " << ik_solution[1] << ", " << ik_solution[2] << "] radians" << std::endl;
    std::cout << "IK solution: [" << ik_solution[0]*180/M_PI << "°, " << ik_solution[1]*180/M_PI << "°, " << ik_solution[2]*180/M_PI << "°]" << std::endl;
    
    // Verify the solution
    Eigen::Vector2d achieved_pos = ik_manipulator.getEndEffectorLocation(ik_solution);
    double error = (target_point - achieved_pos).norm();
    std::cout << "Achieved position: (" << achieved_pos.x() << ", " << achieved_pos.y() << ")" << std::endl;
    std::cout << "Error: " << error << std::endl;
    
    // Visualize the result
    Visualizer::makeFigure(ik_manipulator, ik_solution);
    std::cout << "✓ Inverse kinematics visualization created" << std::endl;


    //----------------

    // Save all visualizations
    std::cout << "\n=== SAVING VISUALIZATIONS ===" << std::endl;
    Visualizer::saveFigures();
    std::cout << "✓ All figures saved" << std::endl;
    
    // Create the collision space constructor


  std::cout << "=== EXERCISE 3: C-SPACE CONSTRUCTION FOR 2-LINK MANIPULATOR ===" << std::endl;
    
    // Create 2-link manipulator with link lengths of 1.0 each 
    MyManipulator2D ex3_manipulator = MyManipulator2D::createCustomManipulator({1.0, 1.0});
    std::cout << "Created 2-link manipulator with link lengths: [1.0, 1.0]" << std::endl;
    
    // Use fine grid resolution for detailed C-space visualization
    std::size_t n_cells = 100;  // Fine grid as specified
    MyManipulatorCSConstructor cspace_constructor(n_cells);
    std::cout << "Using fine grid resolution: " << n_cells << "x" << n_cells << std::endl;
    
    // ===== EXERCISE 3(a): Triangular Obstacle =====
    std::cout << "\n--- Exercise 3(a): Triangular Obstacle ---" << std::endl;
    
    std::unique_ptr<amp::GridCSpace2D> cspace_3a = cspace_constructor.construct(ex3_manipulator, HW4::getEx3Workspace1());
    std::cout << "✓ C-space constructed for triangular obstacle workspace" << std::endl;
    
    // Visualize the C-space
    Visualizer::makeFigure(*cspace_3a);
    std::cout << "✓ C-space visualization created for Exercise 3(a)" << std::endl;
    
    // ===== EXERCISE 3(b): Two Rectangular Obstacles =====
    std::cout << "\n--- Exercise 3(b): Two Rectangular Obstacles ---" << std::endl;
    
    std::unique_ptr<amp::GridCSpace2D> cspace_3b = cspace_constructor.construct(ex3_manipulator, HW4::getEx3Workspace2());
    std::cout << "✓ C-space constructed for two rectangular obstacles workspace" << std::endl;
    
    // Visualize the C-space
    Visualizer::makeFigure(*cspace_3b);
    std::cout << "✓ C-space visualization created for Exercise 3(b)" << std::endl;
    
    // ===== EXERCISE 3(c): Modified Rectangular Obstacles =====
    std::cout << "\n--- Exercise 3(c): Modified Rectangular Obstacles ---" << std::endl;
    
    std::unique_ptr<amp::GridCSpace2D> cspace_3c = cspace_constructor.construct(ex3_manipulator, HW4::getEx3Workspace3());
    std::cout << "✓ C-space constructed for modified rectangular obstacles workspace" << std::endl;
    
    // Visualize the C-space
    Visualizer::makeFigure(*cspace_3c);
    std::cout << "✓ C-space visualization created for Exercise 3(c)" << std::endl;
    
    std::cout << "\n=== Exercise 3 Complete - All C-spaces created ===" << std::endl;
    
    // Save all Exercise 3 visualizations
    std::cout << "\n=== SAVING ALL VISUALIZATIONS ===" << std::endl;
    Visualizer::saveFigures();
    std::cout << "✓ All Exercise 3 C-space figures saved" << std::endl;

    // Grade method (required for homework submission)
    amp::HW4::grade<MyManipulator2D>(cspace_constructor, "amy.heerten@colorado.edu", argc, argv);
    return 0;
}