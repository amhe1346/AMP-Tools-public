#include <Eigen/Dense>
#include <vector>


#include "MyCSConstructors.h"
#include <Eigen/Dense>
#include <queue>
// Local point-in-polygon test for Eigen::Vector2d and std::vector<Eigen::Vector2d>
bool pointInPolygon(const Eigen::Vector2d& pt, const std::vector<Eigen::Vector2d>& poly) {
    int n = poly.size();
    bool inside = false;
    for (int i = 0, j = n - 1; i < n; j = i++) {
        double xi = poly[i].x(), yi = poly[i].y();
        double xj = poly[j].x(), yj = poly[j].y();
        if (((yi > pt.y()) != (yj > pt.y())) &&
            (pt.x() < (xj - xi) * (pt.y() - yi) / (yj - yi + 1e-12) + xi)) {
            inside = !inside;
        }
    }
    return inside;
}

////////////////////// THIS IS FROM HW4 //////////////////////

/* You can just move these classes to shared folder and include them instead of copying them to hw6 project*/

std::pair<std::size_t, std::size_t> MyGridCSpace2D::getCellFromPoint(double x0, double x1) const {
    // Map (x0, x1) to grid cell indices, clamped to grid bounds
    std::size_t cell_x = 0, cell_y = 0;
    if (m_x_max > m_x_min && m_y_max > m_y_min && m_x_cells > 0 && m_y_cells > 0) {
        double x_step = (m_x_max - m_x_min) / m_x_cells;
        double y_step = (m_y_max - m_y_min) / m_y_cells;
        cell_x = std::min(m_x_cells - 1, std::max(std::size_t(0), std::size_t((x0 - m_x_min) / x_step)));
        cell_y = std::min(m_y_cells - 1, std::max(std::size_t(0), std::size_t((x1 - m_y_min) / y_step)));
    }
    return {cell_x, cell_y};
}

// Override this method for computing all of the boolean collision values for each cell in the cspace
std::unique_ptr<amp::GridCSpace2D> MyManipulatorCSConstructor::construct(const amp::LinkManipulator2D& manipulator, const amp::Environment2D& env) {
    // Create an object of my custom cspace type (e.g. MyGridCSpace2D) and store it in a unique pointer. 
    // Pass the constructor parameters to std::make_unique()
    // Use joint limits [0, 2*pi] for both dimensions
    double joint_min = 0.0;
    double joint_max = 2 * M_PI;
    std::unique_ptr<MyGridCSpace2D> cspace_ptr = std::make_unique<MyGridCSpace2D>(m_cells_per_dim, m_cells_per_dim, joint_min, joint_max, joint_min, joint_max);
    // In order to use the pointer as a regular GridCSpace2D object, we can just create a reference
    MyGridCSpace2D& cspace = *cspace_ptr;
    std::cout << "Constructing C-space for manipulator" << std::endl;
    // Mark cells as obstacles if any manipulator link collides with any workspace obstacle
    for (std::size_t ix = 0; ix < cspace.m_x_cells; ++ix) {
        for (std::size_t iy = 0; iy < cspace.m_y_cells; ++iy) {
            double theta1 = cspace.m_x_min + (ix + 0.5) * (cspace.m_x_max - cspace.m_x_min) / cspace.m_x_cells;
            double theta2 = cspace.m_y_min + (iy + 0.5) * (cspace.m_y_max - cspace.m_y_min) / cspace.m_y_cells;
            amp::ManipulatorState state(2);
            state[0] = theta1;
            state[1] = theta2;
            bool collision = false;
            // For each link, check if it collides with any obstacle
            for (std::size_t link = 0; link < manipulator.nLinks(); ++link) {
                Eigen::Vector2d joint_start = manipulator.getJointLocation(state, link);
                Eigen::Vector2d joint_end = manipulator.getJointLocation(state, link + 1);
                // Check for collision with each obstacle
                for (const auto& obs : env.obstacles) {
                    const auto& vertices = obs.verticesCCW();
                    // Check if either joint is inside the obstacle
                    if (pointInPolygon(joint_start, vertices) || pointInPolygon(joint_end, vertices)) {
                        collision = true;
                        break;
                    }
                    // Check if the link segment intersects any edge of the obstacle
                    for (size_t v = 0; v < vertices.size(); ++v) {
                        Eigen::Vector2d v1 = vertices[v];
                        Eigen::Vector2d v2 = vertices[(v + 1) % vertices.size()];
                        // Simple segment intersection test
                        auto cross = [](const Eigen::Vector2d& a, const Eigen::Vector2d& b) {
                            return a.x() * b.y() - a.y() * b.x();
                        };
                        Eigen::Vector2d r = joint_end - joint_start;
                        Eigen::Vector2d s = v2 - v1;
                        Eigen::Vector2d diff = v1 - joint_start;
                        double denom = cross(r, s);
                        double t = cross(diff, s) / (denom + 1e-12);
                        double u = cross(diff, r) / (denom + 1e-12);
                        if (fabs(denom) > 1e-8 && t >= 0 && t <= 1 && u >= 0 && u <= 1) {
                            collision = true;
                            break;
                        }
                    }
                    if (collision) break;
                }
                if (collision) break;
            }
            if (collision) {
                cspace(ix, iy) = true;
            }
        }
    }

    // Returning the object of type std::unique_ptr<MyGridCSpace2D> can automatically cast it to a polymorphic base-class pointer of type std::unique_ptr<amp::GridCSpace2D>.
    // The reason why this works is not super important for our purposes, but if you are curious, look up polymorphism!
    return cspace_ptr;
}

//////////////////////////////////////////////////////////////

// Override this method for computing all of the boolean collision values for each cell in the cspace
std::unique_ptr<amp::GridCSpace2D> MyPointAgentCSConstructor::construct(const amp::Environment2D& env) {
    // Create an object of my custom cspace type (e.g. MyGridCSpace2D) and store it in a unique pointer. 
    // Pass the constructor parameters to std::make_unique()
    std::unique_ptr<MyGridCSpace2D> cspace_ptr = std::make_unique<MyGridCSpace2D>(m_cells_per_dim, m_cells_per_dim, env.x_min, env.x_max, env.y_min, env.y_max);
    // In order to use the pointer as a regular GridCSpace2D object, we can just create a reference
    MyGridCSpace2D& cspace = *cspace_ptr;
    std::cout << "Constructing C-space for point agent" << std::endl;
    // Mark cells as obstacles if their center is inside any workspace obstacle
    for (std::size_t ix = 0; ix < cspace.m_x_cells; ++ix) {
        for (std::size_t iy = 0; iy < cspace.m_y_cells; ++iy) {
            double x = cspace.m_x_min + (ix + 0.5) * (cspace.m_x_max - cspace.m_x_min) / cspace.m_x_cells;
            double y = cspace.m_y_min + (iy + 0.5) * (cspace.m_y_max - cspace.m_y_min) / cspace.m_y_cells;
            Eigen::Vector2d cell_center(x, y);
            for (const auto& obs : env.obstacles) {
                if (pointInPolygon(cell_center, obs.verticesCCW())) {
                    cspace(ix, iy) = true;
                    break;
                }
            }
        }
    }

    // Returning the object of type std::unique_ptr<MyGridCSpace2D> can automatically cast it to a polymorphic base-class pointer of type std::unique_ptr<amp::GridCSpace2D>.
    // The reason why this works is not super important for our purposes, but if you are curious, look up polymorphism!
    return cspace_ptr;
}

amp::Path2D MyWaveFrontAlgorithm::planInCSpace(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace, bool isManipulator) {
    // Full WaveFront grid-based BFS implementation
    using Cell = std::pair<std::size_t, std::size_t>;
    // Cast to MyGridCSpace2D to access member variables
    const MyGridCSpace2D& my_grid = static_cast<const MyGridCSpace2D&>(grid_cspace);
    std::size_t nx = my_grid.m_x_cells;
    std::size_t ny = my_grid.m_y_cells;
    std::vector<std::vector<int>> wave(nx, std::vector<int>(ny, -1));
    std::vector<std::vector<Cell>> parent(nx, std::vector<Cell>(ny, Cell{nx, ny}));

    // Get start and goal cell indices
    Cell start_cell = my_grid.getCellFromPoint(q_init.x(), q_init.y());
    Cell goal_cell = my_grid.getCellFromPoint(q_goal.x(), q_goal.y());

    // Mark obstacles in wave grid
    for (std::size_t ix = 0; ix < nx; ++ix) {
        for (std::size_t iy = 0; iy < ny; ++iy) {
            if (my_grid(ix, iy)) {
                wave[ix][iy] = -2; // obstacle
            }
        }
    }

    // BFS from goal
    std::queue<Cell> q;
    wave[goal_cell.first][goal_cell.second] = 1;
    q.push(goal_cell);
    std::vector<Cell> moves = {{1,0},{-1,0},{0,1},{0,-1}};
    while (!q.empty()) {
        Cell curr = q.front(); q.pop();
        int curr_wave = wave[curr.first][curr.second];
        for (const auto& m : moves) {
            std::size_t nx_cell = curr.first + m.first;
            std::size_t ny_cell = curr.second + m.second;
            if (nx_cell < nx && ny_cell < ny && wave[nx_cell][ny_cell] == -1) {
                wave[nx_cell][ny_cell] = curr_wave + 1;
                parent[nx_cell][ny_cell] = curr;
                q.push({nx_cell, ny_cell});
            }
        }
    }

    // Trace path from start to goal
    amp::Path2D path;
    Cell curr = start_cell;
    if (wave[curr.first][curr.second] < 0) {
        // No path found
        return path;
    }
    std::vector<Cell> cell_path;
    while (curr != goal_cell) {
        cell_path.push_back(curr);
        curr = parent[curr.first][curr.second];
        if (curr.first == nx && curr.second == ny) break; // No parent
    }
    cell_path.push_back(goal_cell);

    // Calculate cell center manually
    double x_min = my_grid.m_x_min;
    double x_max = my_grid.m_x_max;
    double y_min = my_grid.m_y_min;
    double y_max = my_grid.m_y_max;
    double x_step = (x_max - x_min) / nx;
    double y_step = (y_max - y_min) / ny;
    for (auto it = cell_path.begin(); it != cell_path.end(); ++it) {
        double x = x_min + (it->first + 0.5) * x_step;
        double y = y_min + (it->second + 0.5) * y_step;
        path.waypoints.push_back(Eigen::Vector2d(x, y));
    }
    // Ensure path starts with q_init and ends with q_goal
    if (path.waypoints.size() > 0) {
        path.waypoints.front() = q_init;
        path.waypoints.back() = q_goal;
    }
    
    if (path.waypoints.size() > 0) {
        std::cout << "[DEBUG] First waypoint: [" << path.waypoints.front().transpose() << "]\n";
        std::cout << "[DEBUG] Last waypoint:  [" << path.waypoints.back().transpose() << "]\n";
        std::cout << "[DEBUG] q_init:         [" << q_init.transpose() << "]\n";
        std::cout << "[DEBUG] q_goal:         [" << q_goal.transpose() << "]\n";
    }
    if (isManipulator) {
        // Wrap negative joint values to [0, 2*pi]
        double two_pi = 2.0 * M_PI;
        for (auto& wp : path.waypoints) {
            for (int i = 0; i < wp.size(); ++i) {
                if (wp[i] < 0) wp[i] += two_pi;
            }
        }
        Eigen::Vector2d bounds0 = Eigen::Vector2d(0.0, 0.0);
        Eigen::Vector2d bounds1 = Eigen::Vector2d(2*M_PI, 2*M_PI);
        amp::unwrapWaypoints(path.waypoints, bounds0, bounds1);
    }
    return path;
}
