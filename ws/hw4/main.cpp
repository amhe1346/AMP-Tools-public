// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW4.h"

// Include the headers for HW4 code
#include "CSpaceSkeleton.h"
#include "ManipulatorSkeleton.h"

using namespace amp;

// Helper function to create custom obstacles easily
amp::Polygon createCustomTriangle(double x1, double y1, double x2, double y2, double x3, double y3) {
    std::vector<Eigen::Vector2d> vertices = {
        Eigen::Vector2d(x1, y1),
        Eigen::Vector2d(x2, y2),
        Eigen::Vector2d(x3, y3)
    };
    return amp::Polygon(vertices);
}

amp::Polygon createCustomRectangle(double x_min, double y_min, double x_max, double y_max) {
    std::vector<Eigen::Vector2d> vertices = {
        Eigen::Vector2d(x_min, y_min),
        Eigen::Vector2d(x_max, y_min),
        Eigen::Vector2d(x_max, y_max),
        Eigen::Vector2d(x_min, y_max)
    };
    return amp::Polygon(vertices);
}

int main(int argc, char** argv) {
// problem one 

    std::cout << "=== STARTING HW4 EXECUTION ===" << std::endl;
    std::cout.flush();  // Force output
    
    /*
    ===== USER CUSTOMIZATION GUIDE FOR EXERCISE 3A =====
    
    To customize Exercise 3(a), modify these parameters in the Exercise 3(a) section:
    
    1. LINK LENGTHS: 
       std::vector<double> custom_link_lengths = {1.2, 0.8};  // [link1_length, link2_length]
       
    2. NUMBER OF OBSTACLES:
       int num_obstacles = 2;  // Change this number (1 or more)
       
    3. OBSTACLE VERTICES:
       - For Triangle: Modify triangle_vertices array with 3 points
       - For Rectangle: Use createCustomRectangle(x_min, y_min, x_max, y_max)
       - For Custom Triangle: Use createCustomTriangle(x1, y1, x2, y2, x3, y3)
       
    Example vertex format: Eigen::Vector2d(x_coordinate, y_coordinate)
    
    The program will create visualizations showing:
    - Your custom workspace with obstacles
    - Manipulator in the workspace  
    - Corresponding C-space representation
    - Original Exercise 3(a) for comparison
    */

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
    Visualizer::saveFigures(true, "hw4_figs", "png");  // Show figures and save to hw4_figs directory
    std::cout << "✓ All figures saved to figures/hw4_figs/ directory" << std::endl;
    
    // Create the collision space constructor


  std::cout << "=== EXERCISE 3: C-SPACE CONSTRUCTION FOR 2-LINK MANIPULATOR ===" << std::endl;
    std::cout.flush();
    
    // Create 2-link manipulator with link lengths of 1.0 each 
    MyManipulator2D ex3_manipulator = MyManipulator2D::createCustomManipulator({1.0, 1.0});
    std::cout << "Created 2-link manipulator with link lengths: [1.0, 1.0]" << std::endl;
    std::cout.flush();
    
    // Use fine grid resolution for detailed C-space visualization
    std::size_t n_cells = 300;  // Fine grid as specified
    MyManipulatorCSConstructor cspace_constructor(n_cells);
    std::cout << "Using fine grid resolution: " << n_cells << "x" << n_cells << std::endl;
    
    // ===== EXERCISE 3(a): Customizable Triangular Obstacle =====
    std::cout << "\n--- Exercise 3(a): Customizable Triangular Obstacle ---" << std::endl;
    
    // USER CUSTOMIZABLE PARAMETERS FOR EXERCISE 3A
    // Customize link lengths
    std::vector<double> custom_link_lengths = {1.2, 0.8};  // MODIFY THESE VALUES
    std::cout << "Using custom link lengths: [" << custom_link_lengths[0] << ", " << custom_link_lengths[1] << "]" << std::endl;
    
    // Create custom 2-link manipulator
    MyManipulator2D custom_ex3_manipulator = MyManipulator2D::createCustomManipulator(custom_link_lengths);
    
    // Create custom environment with user-specified obstacles
    amp::Environment2D custom_workspace_3a;
    custom_workspace_3a.x_min = -3.0;
    custom_workspace_3a.x_max = 3.0;
    custom_workspace_3a.y_min = -3.0;
    custom_workspace_3a.y_max = 3.0;
    
    // Number of obstacles (USER CUSTOMIZABLE)
    int num_obstacles = 2;  // MODIFY THIS VALUE (was 1 in original)
    std::cout << "Creating " << num_obstacles << " custom obstacles" << std::endl;
    
    // Custom Obstacle 1: Triangle (USER CUSTOMIZABLE VERTICES)
    // Method 1: Direct vertex specification
    std::vector<Eigen::Vector2d> triangle_vertices = {
        Eigen::Vector2d(0.5, 1.0),   // MODIFY THESE VERTICES
        Eigen::Vector2d(1.5, 1.0),
        Eigen::Vector2d(1.0, 2.0)
    };
    amp::Polygon triangle_obstacle(triangle_vertices);
    
    // Method 2: Using helper function (alternative)
    // amp::Polygon triangle_obstacle = createCustomTriangle(0.5, 1.0, 1.5, 1.0, 1.0, 2.0);
    
    custom_workspace_3a.obstacles.push_back(triangle_obstacle);
    std::cout << "Added triangle obstacle with vertices: ";
    for(const auto& v : triangle_vertices) {
        std::cout << "(" << v.x() << "," << v.y() << ") ";
    }
    std::cout << std::endl;
    
    // Custom Obstacle 2 (if num_obstacles > 1): Another shape
    if(num_obstacles > 1) {
        // Option A: Another triangle
        amp::Polygon second_obstacle = createCustomTriangle(-1.5, 0.0, -0.5, 0.0, -1.0, 1.0);
        
        // Option B: Rectangle (uncomment to use instead)
        // amp::Polygon second_obstacle = createCustomRectangle(-1.5, -0.5, -0.5, 0.5);
        
        custom_workspace_3a.obstacles.push_back(second_obstacle);
        std::cout << "Added second obstacle" << std::endl;
    }
    
    // Add more obstacles if specified
    for(int i = 2; i < num_obstacles; i++) {
        // Example: Create rectangular obstacles for additional ones
        double offset = i * 0.8;  // Spread them out
        amp::Polygon rect_obstacle = createCustomRectangle(-2.0 + offset, -1.0, -1.2 + offset, -0.2);
        custom_workspace_3a.obstacles.push_back(rect_obstacle);
        std::cout << "Added rectangular obstacle " << (i+1) << " at offset " << offset << std::endl;
    }
    
    std::cout << "Custom workspace 3(a) has " << custom_workspace_3a.obstacles.size() << " obstacles" << std::endl;
    
    // Visualize the custom workspace environment
    Visualizer::makeFigure(custom_workspace_3a);
    std::cout << "✓ Custom workspace 3(a) environment visualization created" << std::endl;
    
    // Visualize manipulator in the custom workspace
    amp::ManipulatorState sample_config_3a(2);
    sample_config_3a[0] = M_PI/6;  // 30 degrees
    sample_config_3a[1] = M_PI/4;  // 45 degrees
    Visualizer::makeFigure(custom_workspace_3a, custom_ex3_manipulator, sample_config_3a);
    std::cout << "✓ Custom manipulator in workspace 3(a) visualization created" << std::endl;
    
    // Construct C-space with custom parameters
    std::cout << "Constructing C-space with custom parameters..." << std::endl;
    std::unique_ptr<amp::GridCSpace2D> cspace_3a = cspace_constructor.construct(custom_ex3_manipulator, custom_workspace_3a);
    std::cout << "✓ C-space constructed for custom triangular obstacle workspace" << std::endl;
    
    // Visualize the custom C-space
    Visualizer::makeFigure(*cspace_3a);
    std::cout << "✓ Custom C-space visualization created for Exercise 3(a)" << std::endl;
    
    // Also run original Exercise 3(a) for comparison
    std::cout << "\n--- Original Exercise 3(a) for Comparison ---" << std::endl;
    amp::Environment2D original_workspace_3a = HW4::getEx3Workspace1();
    std::unique_ptr<amp::GridCSpace2D> original_cspace_3a = cspace_constructor.construct(ex3_manipulator, original_workspace_3a);
    Visualizer::makeFigure(original_workspace_3a);
    Visualizer::makeFigure(*original_cspace_3a);
    std::cout << "✓ Original Exercise 3(a) completed for comparison" << std::endl;
    
    // ===== EXERCISE 3(b): Two Rectangular Obstacles =====
    std::cout << "\n--- Exercise 3(b): Two Rectangular Obstacles ---" << std::endl;
    
    // Get the workspace environment
    amp::Environment2D workspace_3b = HW4::getEx3Workspace2();
    std::cout << "Workspace 3(b) has " << workspace_3b.obstacles.size() << " obstacles" << std::endl;
    
    // Visualize the workspace environment
    Visualizer::makeFigure(workspace_3b);
    std::cout << "✓ Workspace 3(b) environment visualization created" << std::endl;
    
    // Visualize manipulator in the workspace
    amp::ManipulatorState sample_config_3b(2);
    sample_config_3b[0] = M_PI/3;  // 60 degrees
    sample_config_3b[1] = M_PI/6;  // 30 degrees
    Visualizer::makeFigure(workspace_3b, ex3_manipulator, sample_config_3b);
    std::cout << "✓ Manipulator in workspace 3(b) visualization created" << std::endl;
    
    // Construct C-space
    std::unique_ptr<amp::GridCSpace2D> cspace_3b = cspace_constructor.construct(ex3_manipulator, workspace_3b);
    std::cout << "✓ C-space constructed for two rectangular obstacles workspace" << std::endl;
    
    // Visualize the C-space
    Visualizer::makeFigure(*cspace_3b);
    std::cout << "✓ C-space visualization created for Exercise 3(b)" << std::endl;
    
    // ===== EXERCISE 3(c): Modified Rectangular Obstacles =====
    std::cout << "\n--- Exercise 3(c): Modified Rectangular Obstacles ---" << std::endl;
    
    // Get the workspace environment
    amp::Environment2D workspace_3c = HW4::getEx3Workspace3();
    std::cout << "Workspace 3(c) has " << workspace_3c.obstacles.size() << " obstacles" << std::endl;
    
    // Visualize the workspace environment
    Visualizer::makeFigure(workspace_3c);
    std::cout << "✓ Workspace 3(c) environment visualization created" << std::endl;
    
    // Visualize manipulator in the workspace
    amp::ManipulatorState sample_config_3c(2);
    sample_config_3c[0] = M_PI/4;  // 45 degrees
    sample_config_3c[1] = M_PI/3;  // 60 degrees
    Visualizer::makeFigure(workspace_3c, ex3_manipulator, sample_config_3c);
    std::cout << "✓ Manipulator in workspace 3(c) visualization created" << std::endl;
    
    // Construct C-space
    std::unique_ptr<amp::GridCSpace2D> cspace_3c = cspace_constructor.construct(ex3_manipulator, workspace_3c);
    std::cout << "✓ C-space constructed for modified rectangular obstacles workspace" << std::endl;
    
    // Visualize the C-space
    Visualizer::makeFigure(*cspace_3c);
    std::cout << "✓ C-space visualization created for Exercise 3(c)" << std::endl;
    Visualizer::makeFigure(*cspace_3c);
    std::cout << "✓ C-space visualization created for Exercise 3(c)" << std::endl;
    
    std::cout << "\n=== Exercise 3 Complete - All C-spaces created ===" << std::endl;
    
    // Save all Exercise 3 visualizations
    std::cout << "\n=== SAVING ALL VISUALIZATIONS ===" << std::endl;
    Visualizer::saveFigures(true, "exercise3_figs", "png");  // Show figures and save to exercise3_figs directory
    std::cout << "✓ All Exercise 3 figures saved to figures/exercise3_figs/ directory" << std::endl;

    // Grade method (required for homework submission)
    amp::HW4::grade<MyManipulator2D>(cspace_constructor, "amy.heerten@colorado.edu", argc, argv);
    return 0;
}