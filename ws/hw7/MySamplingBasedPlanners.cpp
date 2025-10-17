#include "MySamplingBasedPlanners.h"
#include <random>
#include <cmath>
#include "../shared/MyCollisionChecker.h"

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
    // RRT parameters
    const int max_iterations = 5000;
    const double step_size = 0.5;
    const double goal_bias = 0.5; // 50% chance to sample goal
    const double goal_threshold = 0.25; // Distance to consider goal reached
    
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
    
    // If no path found, return empty path
    amp::Path2D path;
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