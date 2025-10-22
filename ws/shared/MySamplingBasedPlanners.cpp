#include "MySamplingBasedPlanners.h"
#include <random>
#include <cmath>
#include "MyCollisionChecker.h"

// Implement your PRM algorithm here
amp::Path2D MyPRM::plan(const amp::Problem2D& problem) {
    // Helper function for collision checking between two points
    auto isValidPath = [&](const Eigen::Vector2d& start, const Eigen::Vector2d& end) {
        return amp::MyCollisionChecker::isValidPath(
            start, end,
            problem.obstacles,
            problem.x_min, problem.x_max,
            problem.y_min, problem.y_max);
    };

    int n = n_; // number of samples
    double r = r_; // Connection radius
    std::vector<Eigen::Vector2d> vertices;
    vertices.push_back(problem.q_init); // Always include start
    vertices.push_back(problem.q_goal); // Always include goal

    // Sample free space and add nodes
    for (int i = 0; i < n; ++i) {
        Eigen::Vector2d q_rand;
        q_rand.x() = problem.x_min + static_cast<double>(rand()) / RAND_MAX * (problem.x_max - problem.x_min);
        q_rand.y() = problem.y_min + static_cast<double>(rand()) / RAND_MAX * (problem.y_max - problem.y_min);

        // Check if in obstacle
        if (amp::MyCollisionChecker::pointInObstacles(q_rand, problem.obstacles)) {
            continue; // Skip this sample
        }
        vertices.push_back(q_rand);
    }

    // Find neighbors within radius r for each vertex
    std::vector<std::vector<int>> neighbors(vertices.size());
    for (size_t i = 0; i < vertices.size(); ++i) {
        for (size_t j = 0; j < vertices.size(); ++j) {
            if (i == j) continue;
            double dist = (vertices[i] - vertices[j]).norm();
            if (dist <= r) {
                neighbors[i].push_back(j);
            }
        }
    }

    // Check for valid edges
    std::vector<std::pair<int, int>> edges;
    for (size_t i = 0; i < vertices.size(); ++i) {
        for (int neighbor : neighbors[i]) {
            if (isValidPath(vertices[i], vertices[neighbor])) {
                edges.emplace_back(i, neighbor);
            }
        }
    }

    // Build the PRM graph
    graphPtr_ = std::make_shared<amp::Graph<double>>();
    // Add valid edges
    for (const auto& edge : edges) {
        int i = edge.first;
        int j = edge.second;
        double weight = (vertices[i] - vertices[j]).norm();
        graphPtr_->connect(i, j, weight);
        graphPtr_->connect(j, i, weight); // undirected
    }

    // Use ShortestPathProblem and simple Dijkstra
    amp::ShortestPathProblem spp;
    spp.graph = graphPtr_;
    spp.init_node = 0; // start is the first vertex
    spp.goal_node = 1; // goal is the second vertex

    // Simple Dijkstra implementation
    std::vector<int> prev(vertices.size(), -1);
    std::vector<double> dist(vertices.size(), std::numeric_limits<double>::infinity());
    dist[spp.init_node] = 0.0;
    std::vector<bool> visited(vertices.size(), false);
    
    for (size_t count = 0; count < vertices.size(); ++count) {
        // Find unvisited node with smallest dist
        double min_dist = std::numeric_limits<double>::infinity();
        int u = -1;
        for (size_t i = 0; i < vertices.size(); ++i) {
            if (!visited[i] && dist[i] < min_dist) {
                min_dist = dist[i];
                u = i;
            }
        }
        if (u == -1) break;
        visited[u] = true;
        
        // For each neighbor
        for (const auto& v : graphPtr_->children(u)) {
            double weight = graphPtr_->outgoingEdges(u)[&v - &graphPtr_->children(u)[0]];
            if (dist[u] + weight < dist[v]) {
                dist[v] = dist[u] + weight;
                prev[v] = u;
            }
        }
    }

    // Store node positions for visualization
    nodes_.clear();
    for (size_t i = 0; i < vertices.size(); ++i) {
        nodes_[(amp::Node)i] = vertices[i];
    }

    // Reconstruct path
    amp::Path2D path;
    int at = spp.goal_node;
    while (at != -1) {
        path.waypoints.push_back(vertices[at]);
        at = prev[at];
    }
    std::reverse(path.waypoints.begin(), path.waypoints.end());
    path.valid = (path.waypoints.size() > 1);

    return path;
}

// RRT Implementation
amp::Path2D MyRRT::plan(const amp::Problem2D& problem) {
    // RRT parameters - aggressively tuned for constrained C-spaces
    const int max_iterations = 25000; // Much more iterations for complex spaces
    const double step_size = 0.1;     // Very small steps for precision
    const double goal_bias = 0.5;     // Higher goal bias to reach target
    const double goal_threshold = 0.2; // Tighter threshold for accuracy
    
    // Random number generation
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> x_dist(problem.x_min, problem.x_max);
    std::uniform_real_distribution<double> y_dist(problem.y_min, problem.y_max);
    std::uniform_real_distribution<double> uniform(0.0, 1.0);
    
    // Tree structure: each node stores its parent index and position
    struct TreeNode {
        Eigen::Vector2d position;
        int parent_index;
        
        TreeNode(const Eigen::Vector2d& pos, int parent) : position(pos), parent_index(parent) {}
    };
    
    std::vector<TreeNode> tree;
    tree.emplace_back(problem.q_init, -1); // Root has no parent
    
    // RRT main loop
    for (int iter = 0; iter < max_iterations; ++iter) {
        // Step 1: Sample random configuration (with goal bias)
        Eigen::Vector2d q_rand;
        if (uniform(gen) < goal_bias) {
            q_rand = problem.q_goal; // Sample goal
        } else {
            q_rand = Eigen::Vector2d(x_dist(gen), y_dist(gen)); // Random sample
        }
        
        // Step 2: Find nearest node in tree
        int nearest_index = 0;
        double min_dist = (tree[0].position - q_rand).norm();
        for (size_t i = 1; i < tree.size(); ++i) {
            double dist = (tree[i].position - q_rand).norm();
            if (dist < min_dist) {
                min_dist = dist;
                nearest_index = i;
            }
        }
        
        // Step 3: Extend tree toward sample
        Eigen::Vector2d q_near = tree[nearest_index].position;
        Eigen::Vector2d direction = (q_rand - q_near).normalized();
        Eigen::Vector2d q_new = q_near + step_size * direction;
        
        // Step 4: Check if path from q_near to q_new is collision-free
        if (isValidPath(q_near, q_new, problem)) {
            // Add new node to tree
            tree.emplace_back(q_new, nearest_index);
            
            // Step 5: Check if goal is reached
            if ((q_new - problem.q_goal).norm() < goal_threshold) {
                // Reconstruct path from goal back to start
                amp::Path2D path;
                std::vector<Eigen::Vector2d> waypoints;
                
                int current_index = tree.size() - 1; // Start from last added node
                while (current_index != -1) {
                    waypoints.push_back(tree[current_index].position);
                    current_index = tree[current_index].parent_index;
                }
                
                // Reverse path (we built it backwards)
                std::reverse(waypoints.begin(), waypoints.end());
                
                // Add goal if not exactly reached
                if ((waypoints.back() - problem.q_goal).norm() > 1e-6) {
                    waypoints.push_back(problem.q_goal);
                }
                
                path.waypoints = waypoints;
                return path;
            }
        }
    }
    
    // If no path found, return path with just start point (safer than straight line)
    amp::Path2D path;
    path.waypoints.push_back(problem.q_init);
    
    // Try a simple path - if start and goal are directly reachable, use it
    if (isValidPath(problem.q_init, problem.q_goal, problem)) {
        path.waypoints.push_back(problem.q_goal);
    } else {
        // Stay at start position rather than risk collision
        path.waypoints.push_back(problem.q_init);
    }
    
    return path;
}

// Helper function to check if a straight-line path is collision-free
bool MyRRT::isValidPath(const Eigen::Vector2d& start, const Eigen::Vector2d& end, const amp::Problem2D& problem) {
    return amp::MyCollisionChecker::isValidPath(
        start, end,
        problem.obstacles,
        problem.x_min, problem.x_max,
        problem.y_min, problem.y_max);
}

// Implementation of RRT with detailed visualization
amp::Path2D MyRRTWithVisualization::planWithVisualization(const amp::Problem2D& problem) {
    std::cout << "\n=== RRT PLANNING WITH VISUALIZATION ===" << std::endl;
    std::cout << "Start: (" << problem.q_init.x() << ", " << problem.q_init.y() << ")" << std::endl;
    std::cout << "Goal: (" << problem.q_goal.x() << ", " << problem.q_goal.y() << ")" << std::endl;
    std::cout << "Space bounds: [" << problem.x_min << "," << problem.x_max << "] x [" 
              << problem.y_min << "," << problem.y_max << "]" << std::endl;
    std::cout << "Number of obstacles: " << problem.obstacles.size() << std::endl;
    
    // Create workspace visualization (only for first few instances)
    static int workspace_figure_count = 0;
    if (workspace_figure_count < 3) {
        amp::Visualizer::makeFigure(problem);
        workspace_figure_count++;
    }
    
    // Create environment for collision checking
    amp::Environment2D env;
    env.x_min = problem.x_min;
    env.x_max = problem.x_max;
    env.y_min = problem.y_min;
    env.y_max = problem.y_max;
    env.obstacles = problem.obstacles;
    
    // Check if start/goal are valid using collision checker
    amp::MyCollisionChecker checker;
    bool start_valid = !checker.isInCollision(env, problem.q_init);
    bool goal_valid = !checker.isInCollision(env, problem.q_goal);
    
    std::cout << "Start point valid: " << (start_valid ? "YES" : "NO") << std::endl;
    std::cout << "Goal point valid: " << (goal_valid ? "YES" : "NO") << std::endl;
    
    if (!start_valid || !goal_valid) {
        std::cout << "ERROR: Start or goal in collision!" << std::endl;
        amp::Path2D empty_path;
        empty_path.waypoints.push_back(problem.q_init);
        return empty_path;
    }
    
    // Check direct path using the inherited isValidPath method
    bool direct_path = amp::MyCollisionChecker::isValidPath(
        problem.q_init, problem.q_goal, problem.obstacles,
        problem.x_min, problem.x_max, problem.y_min, problem.y_max);
    std::cout << "Direct path possible: " << (direct_path ? "YES" : "NO") << std::endl;
    
    if (direct_path) {
        std::cout << "Using direct path!" << std::endl;
        amp::Path2D path;
        path.waypoints.push_back(problem.q_init);
        path.waypoints.push_back(problem.q_goal);
        return path;
    }
    
    // RRT parameters - aggressively tuned for constrained C-spaces
    const int max_iterations = 25000;
    const double step_size = 0.1;
    const double goal_bias = 0.5;
    const double goal_threshold = 0.2;
    
    std::cout << "RRT Parameters:" << std::endl;
    std::cout << "  Max iterations: " << max_iterations << std::endl;
    std::cout << "  Step size: " << step_size << std::endl;
    std::cout << "  Goal bias: " << goal_bias << std::endl;
    std::cout << "  Goal threshold: " << goal_threshold << std::endl;
    
    // Random number generation
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> x_dist(problem.x_min, problem.x_max);
    std::uniform_real_distribution<double> y_dist(problem.y_min, problem.y_max);
    std::uniform_real_distribution<double> uniform(0.0, 1.0);
    
    // Tree structure
    struct TreeNode {
        Eigen::Vector2d position;
        int parent_index;
        TreeNode(const Eigen::Vector2d& pos, int parent) : position(pos), parent_index(parent) {}
    };
    
    std::vector<TreeNode> tree;
    tree.emplace_back(problem.q_init, -1);
    
    int successful_extensions = 0;
    int collision_rejections = 0;
    
    // RRT main loop with progress tracking
    for (int iter = 0; iter < max_iterations; ++iter) {
        // Progress reporting
        if (iter % 5000 == 0 && iter > 0) {
            std::cout << "\nIteration " << iter << "/" << max_iterations << std::endl;
            std::cout << "Tree size: " << tree.size() << std::endl;
            std::cout << "Successful extensions: " << successful_extensions << std::endl;
            std::cout << "Collision rejections: " << collision_rejections << std::endl;
            double success_rate = 100.0 * successful_extensions / iter;
            std::cout << "Extension success rate: " << success_rate << "%" << std::endl;
        }
        
        // Sample random configuration (with goal bias)
        Eigen::Vector2d q_rand;
        if (uniform(gen) < goal_bias) {
            q_rand = problem.q_goal;
        } else {
            q_rand = Eigen::Vector2d(x_dist(gen), y_dist(gen));
        }
        
        // Find nearest node in tree
        int nearest_index = 0;
        double min_distance = (tree[0].position - q_rand).norm();
        for (size_t i = 1; i < tree.size(); ++i) {
            double distance = (tree[i].position - q_rand).norm();
            if (distance < min_distance) {
                min_distance = distance;
                nearest_index = i;
            }
        }
        
        // Extend towards random point
        Eigen::Vector2d q_near = tree[nearest_index].position;
        Eigen::Vector2d direction = q_rand - q_near;
        double distance = direction.norm();
        
        Eigen::Vector2d q_new;
        if (distance <= step_size) {
            q_new = q_rand;
        } else {
            q_new = q_near + (direction / distance) * step_size;
        }
        
        // Check if extension is valid
        if (amp::MyCollisionChecker::isValidPath(q_near, q_new, problem.obstacles,
                                                 problem.x_min, problem.x_max, problem.y_min, problem.y_max)) {
            tree.emplace_back(q_new, nearest_index);
            successful_extensions++;
            
            // Check if we reached the goal
            double goal_distance = (q_new - problem.q_goal).norm();
            if (goal_distance < goal_threshold) {
                std::cout << "\nGOAL REACHED at iteration " << iter << "!" << std::endl;
                std::cout << "Final tree size: " << tree.size() << std::endl;
                std::cout << "Goal distance: " << goal_distance << std::endl;
                
                // Reconstruct path
                std::vector<Eigen::Vector2d> waypoints;
                int current_index = tree.size() - 1;
                
                while (current_index != -1) {
                    waypoints.push_back(tree[current_index].position);
                    current_index = tree[current_index].parent_index;
                }
                
                std::reverse(waypoints.begin(), waypoints.end());
                
                std::cout << "Path found with " << waypoints.size() << " waypoints" << std::endl;
                
                amp::Path2D path;
                path.waypoints = waypoints;
                
                // Create workspace visualization with path (only for first few instances)
                static int path_figure_count = 0;
                if (path_figure_count < 3) {
                    amp::Visualizer::makeFigure(problem, path);
                    path_figure_count++;
                }
                
                return path;
            }
        } else {
            collision_rejections++;
        }
    }
    
    // No path found
    std::cout << "\nRRT FAILED after " << max_iterations << " iterations" << std::endl;
    std::cout << "Final tree size: " << tree.size() << std::endl;
    std::cout << "Successful extensions: " << successful_extensions << std::endl;
    std::cout << "Collision rejections: " << collision_rejections << std::endl;
    double success_rate = 100.0 * successful_extensions / max_iterations;
    std::cout << "Extension success rate: " << success_rate << "%" << std::endl;
    
    // Return path with just start position to avoid collisions
    amp::Path2D path;
    path.waypoints.push_back(problem.q_init);
    return path;
}