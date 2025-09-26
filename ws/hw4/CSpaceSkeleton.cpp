#include "CSpaceSkeleton.h"
#include <cmath>

// Override this method for returning whether or not a point is in collision
std::pair<std::size_t, std::size_t> MyGridCSpace2D::getCellFromPoint(double x0, double x1) const {
    // Convert continuous coordinates to discrete grid cells
    // x0 and x1 are joint angles (theta1 and theta2) in the range [0, 2π]
    
    // Get configuration space bounds
    auto x0_bounds = x0Bounds();
    auto x1_bounds = x1Bounds();
    double x_min = x0_bounds.first;
    double x_max = x0_bounds.second;
    double y_min = x1_bounds.first;
    double y_max = x1_bounds.second;
    
    // Get grid dimensions
    auto grid_size = size();
    std::size_t nx = grid_size.first;
    std::size_t ny = grid_size.second;
    
    // Calculate cell dimensions
    double cell_width_x = (x_max - x_min) / nx;
    double cell_width_y = (y_max - y_min) / ny;
    
    std::size_t cell_x = std::min((std::size_t)((x0 - x_min) / cell_width_x), nx - 1);
    std::size_t cell_y = std::min((std::size_t)((x1 - y_min) / cell_width_y), ny - 1);
    
    return {cell_x, cell_y};
}

// Helper function to check if a line segment intersects with a polygon
bool lineSegmentIntersectsPolygon(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2, 
                                 const std::vector<Eigen::Vector2d>& polygon) {
    // Check intersection with each edge of the polygon
    for (size_t i = 0; i < polygon.size(); ++i) {
        size_t next = (i + 1) % polygon.size();
        const Eigen::Vector2d& poly_p1 = polygon[i];
        const Eigen::Vector2d& poly_p2 = polygon[next];
        
        // Line-line intersection test
        double d1, d2, d3, d4;
        d1 = ((poly_p2.x() - poly_p1.x()) * (p1.y() - poly_p1.y()) - (poly_p2.y() - poly_p1.y()) * (p1.x() - poly_p1.x()));
        d2 = ((poly_p2.x() - poly_p1.x()) * (p2.y() - poly_p1.y()) - (poly_p2.y() - poly_p1.y()) * (p2.x() - poly_p1.x()));
        d3 = ((p2.x() - p1.x()) * (poly_p1.y() - p1.y()) - (p2.y() - p1.y()) * (poly_p1.x() - p1.x()));
        d4 = ((p2.x() - p1.x()) * (poly_p2.y() - p1.y()) - (p2.y() - p1.y()) * (poly_p2.x() - p1.x()));
        
        if (((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) && ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0))) {
            return true;
        }
        
        // Check for endpoint intersections
        if (std::abs(d1) < 1e-10 || std::abs(d2) < 1e-10 || std::abs(d3) < 1e-10 || std::abs(d4) < 1e-10) {
            return true;
        }
    }
    return false;
}

// Helper function to check if a point is inside a polygon
bool pointInPolygon(const Eigen::Vector2d& point, const std::vector<Eigen::Vector2d>& polygon) {
    int crossings = 0;
    for (size_t i = 0; i < polygon.size(); ++i) {
        size_t j = (i + 1) % polygon.size();
        
        if (((polygon[i].y() > point.y()) != (polygon[j].y() > point.y())) &&
            (point.x() < (polygon[j].x() - polygon[i].x()) * (point.y() - polygon[i].y()) / 
            (polygon[j].y() - polygon[i].y()) + polygon[i].x())) {
            crossings++;
        }
    }
    return (crossings % 2) == 1;
}

// Override this method for computing all of the boolean collision values for each cell in the cspace
std::unique_ptr<amp::GridCSpace2D> MyManipulatorCSConstructor::construct(const amp::LinkManipulator2D& manipulator, const amp::Environment2D& env) {
    // Create C-space grid from 0 to 2π for both joint angles
    std::unique_ptr<MyGridCSpace2D> cspace_ptr = std::make_unique<MyGridCSpace2D>(
        m_cells_per_dim, m_cells_per_dim, 0.0, 2*M_PI, 0.0, 2*M_PI);
    MyGridCSpace2D& cspace = *cspace_ptr;

    // Get link lengths (assuming 2-link manipulator)
    const std::vector<double>& lengths = manipulator.getLinkLengths();
    double L1 = lengths.size() > 0 ? lengths[0] : 1.0;
    double L2 = lengths.size() > 1 ? lengths[1] : 1.0;
    
    std::cout << "Computing C-space for 2-link manipulator with lengths [" << L1 << ", " << L2 << "]" << std::endl;
    std::cout << "Environment has " << env.obstacles.size() << " obstacles" << std::endl;
    
    // Get grid dimensions
    auto grid_size = cspace.size();
    std::size_t nx = grid_size.first;
    std::size_t ny = grid_size.second;
    
    // For each cell in the C-space grid
    for (std::size_t i = 0; i < nx; ++i) {
        for (std::size_t j = 0; j < ny; ++j) {
            // Convert grid indices to joint angles
            double theta1 = (double)i / nx * 2 * M_PI;
            double theta2 = (double)j / ny * 2 * M_PI;
            
            // Compute forward kinematics for this configuration
            // Joint 0 (base) at origin
            Eigen::Vector2d joint0(0.0, 0.0);
            
            // Joint 1 (end of first link)
            Eigen::Vector2d joint1(L1 * cos(theta1), L1 * sin(theta1));
            
            // Joint 2 (end of second link) 
            double cumulative_angle = theta1 + theta2;
            Eigen::Vector2d joint2 = joint1 + Eigen::Vector2d(L2 * cos(cumulative_angle), L2 * sin(cumulative_angle));
            
            // Check collision for both links with all obstacles
            bool in_collision = false;
            
            for (const auto& obstacle : env.obstacles) {
                std::vector<Eigen::Vector2d> obstacle_vertices = obstacle.verticesCW();
                
                // Check if first link (joint0 to joint1) intersects with obstacle
                if (lineSegmentIntersectsPolygon(joint0, joint1, obstacle_vertices)) {
                    in_collision = true;
                    break;
                }
                
                // Check if second link (joint1 to joint2) intersects with obstacle  
                if (lineSegmentIntersectsPolygon(joint1, joint2, obstacle_vertices)) {
                    in_collision = true;
                    break;
                }
                
                // Check if any joint is inside the obstacle
                if (pointInPolygon(joint0, obstacle_vertices) || 
                    pointInPolygon(joint1, obstacle_vertices) || 
                    pointInPolygon(joint2, obstacle_vertices)) {
                    in_collision = true;
                    break;
                }
            }
            
            // Store collision status
            cspace(i, j) = in_collision;
        }
    }
    
    std::cout << "C-space construction completed" << std::endl;
    return cspace_ptr;
}
