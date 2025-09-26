#include "custom_triangle.h"
#include "AMPCore.h"
#include "hw/HW4.h"
#include "tools/Environment.h"
#include <iostream>
#include <cmath>
#include <tuple>
#include <fstream>

namespace amp {
    
    // Function to create  custom triangle obstacles
    Obstacle2D getCustomTriangle1() {
        std::vector<Eigen::Vector2d> vertices;
        vertices.push_back(Eigen::Vector2d(0.0, 0.0));  // Vertex 1
        vertices.push_back(Eigen::Vector2d(1.0, 2.0));  // Vertex 2  
        vertices.push_back(Eigen::Vector2d(0.0, 2.0));  // Vertex 3
        return Obstacle2D(vertices);
    }
    
    

    
    // Function to create environment with custom triangle
    Environment2D getCustomTriangleEnvironment(int triangle_type) {
        Environment2D env;
        env.x_min = 0.0;
        env.x_max = 5.0;
        env.y_min = 0.0;
        env.y_max = 5.0;
        
        switch(triangle_type) {
            case 1:
                env.obstacles.push_back(getCustomTriangle1());
                break;
           
            default:
                env.obstacles.push_back(getCustomTriangle1());
        }
        
        return env;
    }
    
    // Function to print triangle vertices (for debugging)
    void printTriangleVertices() {
        // Get the custom triangle obstacle  
        Obstacle2D triangle = getCustomTriangle1();
        std::vector<Eigen::Vector2d> obstacle_vertices = triangle.verticesCW();

        // Print the obstacle vertices
        std::cout << "Custom Triangle Obstacle Vertices:" << std::endl;
        for(size_t i = 0; i < obstacle_vertices.size(); ++i) {
            std::cout << "Vertex " << i << ": (" << obstacle_vertices[i].x() 
                      << ", " << obstacle_vertices[i].y() << ")" << std::endl;
        }
    }

    // Function to compute C-space obstacle for translating robot
    std::vector<Eigen::Vector2d> computeCSpaceObstacle(const std::vector<Eigen::Vector2d>& obstacle_vertices) {
        
        // Find the lower-left vertex (minimum x, then minimum y)
        Eigen::Vector2d lower_left = obstacle_vertices[0];
        for(const auto& vertex : obstacle_vertices) {
            if(vertex.x() < lower_left.x() || 
               (std::abs(vertex.x() - lower_left.x()) < 1e-6 && vertex.y() < lower_left.y())) {
                lower_left = vertex;
            }
        }
        
        std::cout << "Reference point (lower-left): (" << lower_left.x() 
                  << ", " << lower_left.y() << ")" << std::endl;
        
        // Print robot vertices relative to reference point
        std::cout << "\nRobot vertices relative to reference point:" << std::endl;
        for(size_t i = 0; i < obstacle_vertices.size(); ++i) {
            Eigen::Vector2d robot_relative = obstacle_vertices[i] - lower_left;
            std::cout << "Robot vertex " << i << " relative: (" << robot_relative.x() 
                      << ", " << robot_relative.y() << ")" << std::endl;
        }
        
        // Compute Minkowski sum: for each obstacle vertex, add each robot vertex
        std::vector<Eigen::Vector2d> cspace_vertices;
        
        for(const auto& obs_vertex : obstacle_vertices) {
            for(const auto& robot_vertex : obstacle_vertices) {
                // Translate robot vertex relative to reference point
                Eigen::Vector2d robot_relative = robot_vertex - lower_left;
                // Add to obstacle vertex
                Eigen::Vector2d cspace_vertex = obs_vertex + robot_relative;
                cspace_vertices.push_back(cspace_vertex);
            }
        }
        // Print C-space vertices
    std::cout << "\nC-Space Obstacle Vertices (" << cspace_vertices.size() << " total):" << std::endl;
    for(size_t i = 0; i < cspace_vertices.size(); ++i) {
        std::cout << "C-Space Vertex " << i << ": (" << cspace_vertices[i].x() 
                  << ", " << cspace_vertices[i].y() << ")" << std::endl;
    }
        return cspace_vertices;
    }
    
    // Function to create workspace visualization showing Minkowski sum process
    Environment2D getMinkowskiSumVisualization() {
        Environment2D env;
        env.x_min = -3.0;
        env.x_max = 5.0;
        env.y_min = -3.0;
        env.y_max = 7.0;
        
        // Get the original obstacle triangle
        Obstacle2D original_obstacle = getCustomTriangle1();
        std::vector<Eigen::Vector2d> obstacle_vertices = original_obstacle.verticesCW();
        
        // Add the original obstacle
        env.obstacles.push_back(original_obstacle);
        
        // Create robot triangles at several key positions to show the sweep
        std::vector<Eigen::Vector2d> demo_positions = {
            Eigen::Vector2d(0.0, 0.0),   // At obstacle vertex
            Eigen::Vector2d(1.0, 2.0),   // At another obstacle vertex  
            Eigen::Vector2d(0.0, 2.0),   // At third obstacle vertex
            Eigen::Vector2d(0.5, 1.0),   // Intermediate position
            Eigen::Vector2d(-0.5, 0.5)   // Outside position
        };
        
        // For each demo position, create a robot triangle
        for(const auto& pos : demo_positions) {
            std::vector<Eigen::Vector2d> robot_vertices;
            for(const auto& vertex : obstacle_vertices) {
                robot_vertices.push_back(vertex + pos);
            }
            env.obstacles.push_back(Obstacle2D(robot_vertices));
        }
        
        return env;
    }
    
    // Function to rotate a 2D point by angle theta (in radians)
    Eigen::Vector2d rotatePoint(const Eigen::Vector2d& point, double theta) {
        double cos_theta = std::cos(theta);
        double sin_theta = std::sin(theta);
        return Eigen::Vector2d(
            cos_theta * point.x() - sin_theta * point.y(),
            sin_theta * point.x() + cos_theta * point.y()
        );
    }
    
    // Function to compute 3D C-space obstacle for translating + rotating robot
    std::vector<std::tuple<double, double, double>> compute3DCSpaceObstacle(const std::vector<Eigen::Vector2d>& obstacle_vertices) {
        std::vector<std::tuple<double, double, double>> cspace_3d_vertices;
        
        // 12 equally-spaced rotation angles from 0 to 2π
        const int num_rotations = 12;
        std::vector<double> rotation_angles;
        for(int i = 0; i < num_rotations; ++i) {
            double theta = (2.0 * M_PI * i) / num_rotations;
            rotation_angles.push_back(theta);
        }
        
        std::cout << "\nComputing 3D C-space obstacle with " << num_rotations << " rotation angles..." << std::endl;
        
        // Find reference point (lower-left of original robot)
        Eigen::Vector2d lower_left = obstacle_vertices[0];
        for(const auto& vertex : obstacle_vertices) {
            if(vertex.x() < lower_left.x() || 
               (std::abs(vertex.x() - lower_left.x()) < 1e-6 && vertex.y() < lower_left.y())) {
                lower_left = vertex;
            }
        }
        
        // For each rotation angle
        for(size_t rot_idx = 0; rot_idx < rotation_angles.size(); ++rot_idx) {
            double theta = rotation_angles[rot_idx];
            std::cout << "Processing rotation " << rot_idx << ": " << theta << " radians (" 
                      << (theta * 180.0 / M_PI) << " degrees)" << std::endl;
            
            // Rotate robot vertices around reference point
            std::vector<Eigen::Vector2d> rotated_robot_vertices;
            for(const auto& robot_vertex : obstacle_vertices) {
                Eigen::Vector2d relative_vertex = robot_vertex - lower_left;
                Eigen::Vector2d rotated_vertex = rotatePoint(relative_vertex, theta);
                rotated_robot_vertices.push_back(rotated_vertex);
            }
            
            // Compute 2D Minkowski sum for this rotation
            for(const auto& obs_vertex : obstacle_vertices) {
                for(const auto& rotated_robot_vertex : rotated_robot_vertices) {
                    Eigen::Vector2d cspace_2d_vertex = obs_vertex + rotated_robot_vertex;
                    // Store as (x, y, theta) tuple
                    cspace_3d_vertices.push_back(std::make_tuple(
                        cspace_2d_vertex.x(), 
                        cspace_2d_vertex.y(), 
                        theta
                    ));
                }
            }
        }
        
        std::cout << "Generated " << cspace_3d_vertices.size() << " 3D C-space vertices" << std::endl;
        return cspace_3d_vertices;
    }
    
    // Function to create multiple 2D slices of the 3D C-space for visualization
    std::vector<Environment2D> get3DCSpaceSlices() {
        std::vector<Environment2D> slices;
        
        // Get original obstacle
        Obstacle2D original_obstacle = getCustomTriangle1();
        std::vector<Eigen::Vector2d> obstacle_vertices = original_obstacle.verticesCW();
        
        // Compute 3D C-space
        auto cspace_3d_vertices = compute3DCSpaceObstacle(obstacle_vertices);
        
        // Create 12 slices (one for each rotation angle)
        const int num_slices = 12;
        for(int slice_idx = 0; slice_idx < num_slices; ++slice_idx) {
            double target_theta = (2.0 * M_PI * slice_idx) / num_slices;
            
            Environment2D slice_env;
            slice_env.x_min = -4.0; slice_env.x_max = 6.0;
            slice_env.y_min = -4.0; slice_env.y_max = 8.0;
            
            // Collect all vertices for this rotation angle
            std::vector<Eigen::Vector2d> slice_vertices;
            for(const auto& vertex_3d : cspace_3d_vertices) {
                double x = std::get<0>(vertex_3d);
                double y = std::get<1>(vertex_3d);
                double theta = std::get<2>(vertex_3d);
                
                // Check if this vertex belongs to current slice
                if(std::abs(theta - target_theta) < 1e-6) {
                    slice_vertices.push_back(Eigen::Vector2d(x, y));
                }
            }
            
            // Create obstacle from slice vertices (if we have enough vertices)
            if(slice_vertices.size() >= 3) {
                slice_env.obstacles.push_back(Obstacle2D(slice_vertices));
            }
            
            slices.push_back(slice_env);
        }
        
        return slices;
    }
    
    // Function to create a 3D stacked visualization of all rotation slices
    Environment2D get3DStackedVisualization() {
        Environment2D stacked_env;
        stacked_env.x_min = -4.0; 
        stacked_env.x_max = 6.0;
        stacked_env.y_min = -4.0; 
        stacked_env.y_max = 20.0;  // Extended to accommodate stacking
        
        // Get original obstacle
        Obstacle2D original_obstacle = getCustomTriangle1();
        std::vector<Eigen::Vector2d> obstacle_vertices = original_obstacle.verticesCW();
        
        // Compute 3D C-space
        auto cspace_3d_vertices = compute3DCSpaceObstacle(obstacle_vertices);
        
        // Create 12 slices stacked vertically
        const int num_slices = 12;
        const double stack_spacing = 1.5;  // Vertical spacing between slices
        
        std::cout << "\nCreating 3D stacked visualization with " << num_slices << " layers..." << std::endl;
        
        for(int slice_idx = 0; slice_idx < num_slices; ++slice_idx) {
            double target_theta = (2.0 * M_PI * slice_idx) / num_slices;
            double y_offset = slice_idx * stack_spacing;  // Stack vertically
            
            std::cout << "Processing slice " << slice_idx << " at θ=" 
                      << (target_theta * 180.0 / M_PI) << "° (y_offset=" << y_offset << ")" << std::endl;
            
            // Collect all vertices for this rotation angle
            std::vector<Eigen::Vector2d> slice_vertices;
            for(const auto& vertex_3d : cspace_3d_vertices) {
                double x = std::get<0>(vertex_3d);
                double y = std::get<1>(vertex_3d);
                double theta = std::get<2>(vertex_3d);
                
                // Check if this vertex belongs to current slice
                if(std::abs(theta - target_theta) < 1e-6) {
                    // Add to slice with vertical offset to simulate 3D stacking
                    slice_vertices.push_back(Eigen::Vector2d(x, y + y_offset));
                }
            }
            
            // Create obstacle from slice vertices (if we have enough vertices)
            if(slice_vertices.size() >= 3) {
                stacked_env.obstacles.push_back(Obstacle2D(slice_vertices));
            }
            
            // Also add the original obstacle at each level for reference
            std::vector<Eigen::Vector2d> original_vertices_offset;
            for(const auto& vertex : obstacle_vertices) {
                original_vertices_offset.push_back(Eigen::Vector2d(vertex.x(), vertex.y() + y_offset));
            }
            if(original_vertices_offset.size() >= 3) {
                stacked_env.obstacles.push_back(Obstacle2D(original_vertices_offset));
            }
        }
        
        std::cout << "Created stacked visualization with " << stacked_env.obstacles.size() 
                  << " obstacles across " << num_slices << " layers" << std::endl;
        
        return stacked_env;
    }
    
    // Function to export 3D C-space data for external 3D plotting
    void export3DCSpaceData(const std::string& filename) {
        // Get original obstacle
        Obstacle2D original_obstacle = getCustomTriangle1();
        std::vector<Eigen::Vector2d> obstacle_vertices = original_obstacle.verticesCW();
        
        // Compute 3D C-space
        auto cspace_3d_vertices = compute3DCSpaceObstacle(obstacle_vertices);
        
        // Export to file for 3D plotting
        std::ofstream file(filename);
        if(!file.is_open()) {
            std::cout << "Error: Could not open file " << filename << " for writing" << std::endl;
            return;
        }
        
        std::cout << "Exporting 3D C-space data to " << filename << "..." << std::endl;
        
        // Write header
        file << "# 3D C-Space Obstacle Data\n";
        file << "# Format: x y theta(radians) theta(degrees)\n";
        file << "# Total vertices: " << cspace_3d_vertices.size() << "\n";
        
        // Write data
        for(const auto& vertex_3d : cspace_3d_vertices) {
            double x = std::get<0>(vertex_3d);
            double y = std::get<1>(vertex_3d);
            double theta_rad = std::get<2>(vertex_3d);
            double theta_deg = theta_rad * 180.0 / M_PI;
            
            file << x << " " << y << " " << theta_rad << " " << theta_deg << "\n";
        }
        
        file.close();
        std::cout << "Successfully exported " << cspace_3d_vertices.size() 
                  << " 3D vertices to " << filename << std::endl;
        
        // Also create a Python script for 3D visualization
        std::string python_script = filename.substr(0, filename.find_last_of('.')) + "_plot.py";
        std::ofstream py_file(python_script);
        if(py_file.is_open()) {
            py_file << "#!/usr/bin/env python3\n";
            py_file << "import numpy as np\n";
            py_file << "import matplotlib.pyplot as plt\n";
            py_file << "from mpl_toolkits.mplot3d import Axes3D\n";
            py_file << "from mpl_toolkits.mplot3d.art3d import Poly3DCollection\n";
            py_file << "from scipy.spatial import ConvexHull\n\n";
            
            py_file << "# Load 3D C-space data\n";
            py_file << "data = np.loadtxt('" << filename << "', comments='#')\n";
            py_file << "x = data[:, 0]\n";
            py_file << "y = data[:, 1]\n";
            py_file << "theta = data[:, 2]  # radians\n";
            py_file << "theta_deg = data[:, 3]  # degrees\n\n";
            
            py_file << "# Create 3D plot with stacked shapes\n";
            py_file << "fig = plt.figure(figsize=(15, 10))\n";
            py_file << "ax = fig.add_subplot(111, projection='3d')\n\n";
            
            py_file << "# Get unique rotation angles\n";
            py_file << "unique_angles = np.unique(theta_deg)\n";
            py_file << "colors = plt.cm.viridis(np.linspace(0, 1, len(unique_angles)))\n\n";
            
            py_file << "# For each rotation angle, create a 3D polygon\n";
            py_file << "for i, angle in enumerate(unique_angles):\n";
            py_file << "    # Get points for this rotation angle\n";
            py_file << "    mask = np.abs(theta_deg - angle) < 0.1\n";
            py_file << "    x_slice = x[mask]\n";
            py_file << "    y_slice = y[mask]\n";
            py_file << "    \n";
            py_file << "    if len(x_slice) > 2:  # Need at least 3 points for a shape\n";
            py_file << "        try:\n";
            py_file << "            # Create convex hull to get proper polygon shape\n";
            py_file << "            points_2d = np.column_stack([x_slice, y_slice])\n";
            py_file << "            hull = ConvexHull(points_2d)\n";
            py_file << "            hull_points = points_2d[hull.vertices]\n";
            py_file << "            \n";
            py_file << "            # Create 3D polygon at this rotation layer\n";
            py_file << "            z_level = angle  # Use angle in degrees as z-coordinate\n";
            py_file << "            vertices_3d = [(pt[0], pt[1], z_level) for pt in hull_points]\n";
            py_file << "            \n";
            py_file << "            # Add polygon to 3D plot\n";
            py_file << "            poly = [vertices_3d]\n";
            py_file << "            ax.add_collection3d(Poly3DCollection(poly, alpha=0.6, \n";
            py_file << "                                               facecolors=colors[i], \n";
            py_file << "                                               edgecolors='black', linewidth=0.5))\n";
            py_file << "            \n";
            py_file << "            # Also plot the outline\n";
            py_file << "            hull_x = np.append(hull_points[:, 0], hull_points[0, 0])\n";
            py_file << "            hull_y = np.append(hull_points[:, 1], hull_points[0, 1])\n";
            py_file << "            hull_z = np.full_like(hull_x, z_level)\n";
            py_file << "            ax.plot(hull_x, hull_y, hull_z, color='black', linewidth=2)\n";
            py_file << "            \n";
            py_file << "        except Exception as e:\n";
            py_file << "            print(f'Could not create polygon for angle {angle}: {e}')\n";
            py_file << "            # Fallback to scatter plot for this layer\n";
            py_file << "            ax.scatter(x_slice, y_slice, angle, c=[colors[i]], s=20, alpha=0.8)\n\n";
            
            py_file << "# Set labels and title\n";
            py_file << "ax.set_xlabel('X Position', fontsize=12)\n";
            py_file << "ax.set_ylabel('Y Position', fontsize=12)\n";
            py_file << "ax.set_zlabel('Rotation Angle (degrees)', fontsize=12)\n";
            py_file << "ax.set_title('3D C-Space Obstacle\\n(Stacked Polygon Layers)', fontsize=14)\n\n";
            
            py_file << "# Set better viewing angle\n";
            py_file << "ax.view_init(elev=20, azim=45)\n\n";
            
            py_file << "# Add a color bar\n";
            py_file << "sm = plt.cm.ScalarMappable(cmap='viridis', norm=plt.Normalize(vmin=0, vmax=330))\n";
            py_file << "sm.set_array([])\n";
            py_file << "plt.colorbar(sm, ax=ax, label='Rotation Angle (degrees)', shrink=0.5)\n\n";
            
            py_file << "# Plot by rotation slices\n";
            py_file << "fig2, ax2 = plt.subplots(3, 4, figsize=(16, 12))\n";
            py_file << "fig2.suptitle('C-Space Slices at Different Rotation Angles')\n";
            py_file << "angles = np.unique(theta_deg)\n";
            py_file << "for i, angle in enumerate(angles):\n";
            py_file << "    if i < 12:  # Only plot first 12\n";
            py_file << "        row, col = i // 4, i % 4\n";
            py_file << "        mask = np.abs(theta_deg - angle) < 0.1\n";
            py_file << "        ax2[row, col].scatter(x[mask], y[mask], alpha=0.7, s=10)\n";
            py_file << "        ax2[row, col].set_title(f'θ = {angle:.1f}°')\n";
            py_file << "        ax2[row, col].set_xlabel('X')\n";
            py_file << "        ax2[row, col].set_ylabel('Y')\n";
            py_file << "        ax2[row, col].grid(True)\n\n";
            
            py_file << "plt.tight_layout()\n";
            py_file << "plt.show()\n";
            
            py_file.close();
            std::cout << "Created Python 3D plotting script: " << python_script << std::endl;
            std::cout << "Run with: python3 " << python_script << std::endl;
        }
    }
}