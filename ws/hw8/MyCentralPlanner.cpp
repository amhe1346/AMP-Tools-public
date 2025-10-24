// All includes at the top
#include "MyMultiAgentPlanners.h"
#include "../shared/MyCollisionChecker.h"
#include "../shared/ObstacleExpansion.h"
#include "../shared/MyTranslationalCSpace.h"
#include <unordered_set>
#include <random>
#include <algorithm>

// Default override: call parameterized plan with default values
amp::MultiAgentPath2D MyCentralPlanner::plan(const amp::MultiAgentProblem2D& problem) {
    // Default values: n=5000, r=1.0, pgoal=0.1, epsilon=0.5
    return plan(problem, 5000, 1.0, 0.1, 0.5);
}

MyCentralPlanner::~MyCentralPlanner() {}
#include <algorithm>


// New: plan with precomputed C-spaces
amp::MultiAgentPath2D MyCentralPlanner::plan(
    const amp::MultiAgentProblem2D& problem,
    const std::vector<amp::MyTranslationalCSpace>& cspaces,
    int n, double r, double pgoal, double epsilon) {

    amp::MultiAgentPath2D multi_agent_path;
    // Joint GoalBiasRRT in the coupled multi-agent space
    using JointConfig = std::vector<Eigen::Vector2d>;
    struct JointNode {
        JointConfig config;
        int parent_idx;
        JointNode(const JointConfig& c, int p) : config(c), parent_idx(p) {}
    };

    const size_t num_agents = problem.agent_properties.size();
    const double step_size = 0.1;
    const double goal_bias = 0.2;
    const double goal_threshold = 0.2;
    const int max_iterations = n;

    // Initial and goal joint configs
    JointConfig q_init, q_goal;
    for (size_t i = 0; i < num_agents; ++i) {
        q_init.push_back(problem.agent_properties[i].q_init);
        q_goal.push_back(problem.agent_properties[i].q_goal);
    }

    // Random number generation for each agent
    std::random_device rd;
    std::mt19937 gen(rd());
    std::vector<std::uniform_real_distribution<double>> x_dists, y_dists;
    for (size_t i = 0; i < num_agents; ++i) {
        x_dists.emplace_back(cspaces[i].x0Bounds().first, cspaces[i].x0Bounds().second);
        y_dists.emplace_back(cspaces[i].x1Bounds().first, cspaces[i].x1Bounds().second);
    }
    std::uniform_real_distribution<double> uniform(0.0, 1.0);

    // Tree of joint configs
    std::vector<JointNode> tree;
    tree.emplace_back(q_init, -1);

    auto is_valid_joint = [&](const JointConfig& config) {
        // Check each agent for obstacle collision and treat other agents as moving obstacles
        for (size_t i = 0; i < num_agents; ++i) {
            if (cspaces[i].inCollision(config[i].x(), config[i].y())) {
                return false;
            }
            // Treat other agents as moving obstacles
            for (size_t j = 0; j < num_agents; ++j) {
                if (i == j) continue;
                double buffer = 0.2;
                double min_dist = problem.agent_properties[i].radius + problem.agent_properties[j].radius + buffer; // buffer is now 0.2
                if ((config[i] - config[j]).norm() < min_dist) {
                    return false;
                }
            }
        }
        return true;
    };

    auto distance = [&](const JointConfig& a, const JointConfig& b) {
        double d = 0.0;
        for (size_t i = 0; i < num_agents; ++i) d += (a[i] - b[i]).squaredNorm();
        return std::sqrt(d);
    };

    int goal_idx = -1;
    for (int iter = 0; iter < max_iterations; ++iter) {
        bool valid_sample_found = false;
    for (int resample_attempt = 0; resample_attempt < 200; ++resample_attempt) {
            // Sample joint config
            JointConfig q_rand(num_agents);
            if (uniform(gen) < goal_bias) {
                q_rand = q_goal;
            } else {
                // Biased sampling: ensure agents are not too close
                bool valid_sample = false;
                int sample_attempts = 0;
                while (!valid_sample && sample_attempts < 20) {
                    for (size_t i = 0; i < num_agents; ++i) {
                        q_rand[i].x() = x_dists[i](gen);
                        q_rand[i].y() = y_dists[i](gen);
                    }
                    valid_sample = true;
                    for (size_t i = 0; i < num_agents; ++i) {
                        for (size_t j = i+1; j < num_agents; ++j) {
                            double min_dist = problem.agent_properties[i].radius + problem.agent_properties[j].radius + 0.2; // buffer is now 0.2
                            if ((q_rand[i] - q_rand[j]).norm() < min_dist) {
                                // Bias: move one robot clockwise, the other counter-clockwise
                                Eigen::Vector2d diff = q_rand[i] - q_rand[j];
                                if (diff.norm() > 1e-6) {
                                    // Clockwise orthogonal: (-diff.y(), diff.x())
                                    Eigen::Vector2d ortho_cw(-diff.y(), diff.x());
                                    ortho_cw.normalize();
                                    // Counter-clockwise orthogonal: (diff.y(), -diff.x())
                                    Eigen::Vector2d ortho_ccw(diff.y(), -diff.x());
                                    ortho_ccw.normalize();
                                    double bias_dist = 0.2; // step to move away
                                    q_rand[i] += ortho_cw * bias_dist;
                                    q_rand[j] += ortho_ccw * bias_dist;
                                }
                                valid_sample = false;
                                break;
                            }
                        }
                        if (!valid_sample) break;
                    }
                    ++sample_attempts;
                }
                if (!valid_sample) {
                    std::cout << "[JOINT RRT] Failed to find valid sample after " << sample_attempts << " attempts. Agents may be stuck in collision." << std::endl;
                }
            }
            // Find nearest node in tree
            int nearest_idx = 0;
            double min_dist = distance(tree[0].config, q_rand);
            for (size_t i = 1; i < tree.size(); ++i) {
                double d = distance(tree[i].config, q_rand);
                if (d < min_dist) { min_dist = d; nearest_idx = i; }
            }
            // Extend: move each agent toward its sampled goal by step_size
            JointConfig q_new = tree[nearest_idx].config;
            for (size_t i = 0; i < num_agents; ++i) {
                Eigen::Vector2d dir = q_rand[i] - q_new[i];
                double len = dir.norm();
                if (len > step_size) dir = dir / len * step_size;
                q_new[i] += dir;
            }
            // Check new joint config for obstacle and inter-agent collisions
            if (!is_valid_joint(q_new)) continue;
            // Check edge (motion) validity for each agent
            bool edge_valid = true;
            // Stricter edge checking: interpolate between start and end for all robots
            int interp_steps = 10;
            for (int step = 0; step <= interp_steps; ++step) {
                double alpha = double(step) / interp_steps;
                std::vector<Eigen::Vector2d> interp_config(num_agents);
                for (size_t i = 0; i < num_agents; ++i) {
                    interp_config[i] = tree[nearest_idx].config[i] * (1.0 - alpha) + q_new[i] * alpha;
                }
                // Check for obstacle and inter-agent collisions at this interpolated step
                for (size_t i = 0; i < num_agents; ++i) {
                    if (cspaces[i].inCollision(interp_config[i].x(), interp_config[i].y())) {
                        edge_valid = false;
                        break;
                    }
                    for (size_t j = 0; j < num_agents; ++j) {
                        if (i == j) continue;
                        double min_dist = problem.agent_properties[i].radius + problem.agent_properties[j].radius + 0.2; // buffer is now 0.2
                        if ((interp_config[i] - interp_config[j]).norm() < min_dist) {
                            edge_valid = false;
                            break;
                        }
                    }
                    if (!edge_valid) break;
                }
                if (!edge_valid) break;
            }
            if (!edge_valid) {
                std::cout << "[JOINT RRT] Edge collision persists for Agent(s) at step. Unable to find valid edge." << std::endl;
                continue;
            }
            // Add to tree
            tree.emplace_back(q_new, nearest_idx);
            // Check for goal: all robots within epsilon of their goals
            bool all_within_epsilon = true;
            for (size_t i = 0; i < num_agents; ++i) {
                if ((q_new[i] - q_goal[i]).norm() > epsilon) {
                    all_within_epsilon = false;
                    break;
                }
            }
            if (all_within_epsilon) {
                goal_idx = tree.size() - 1;
                break;
            }
            valid_sample_found = true;
            break;
        }
        if (!valid_sample_found) continue;
    }

    multi_agent_path.agent_paths.resize(num_agents);
    if (goal_idx != -1) {
        // Reconstruct joint path
        std::vector<JointConfig> joint_path;
        int idx = goal_idx;
        while (idx != -1) {
            joint_path.push_back(tree[idx].config);
            idx = tree[idx].parent_idx;
        }
        std::reverse(joint_path.begin(), joint_path.end());
        // Fill agent paths
        for (size_t agent_idx = 0; agent_idx < num_agents; ++agent_idx) {
            multi_agent_path.agent_paths[agent_idx].waypoints.clear();
            for (const auto& joint : joint_path) {
                multi_agent_path.agent_paths[agent_idx].waypoints.push_back(joint[agent_idx]);
            }
            // Ensure path starts at q_init and ends at q_goal
            if (multi_agent_path.agent_paths[agent_idx].waypoints.empty() ||
                multi_agent_path.agent_paths[agent_idx].waypoints.front() != problem.agent_properties[agent_idx].q_init) {
                multi_agent_path.agent_paths[agent_idx].waypoints.insert(
                    multi_agent_path.agent_paths[agent_idx].waypoints.begin(),
                    problem.agent_properties[agent_idx].q_init);
            }
            if (multi_agent_path.agent_paths[agent_idx].waypoints.back() != problem.agent_properties[agent_idx].q_goal) {
                multi_agent_path.agent_paths[agent_idx].waypoints.push_back(problem.agent_properties[agent_idx].q_goal);
            }
        }
        // Final path validation
        bool path_valid = true;
        for (size_t t = 0; t < joint_path.size(); ++t) {
            // Obstacle and inter-agent collision at each step
            for (size_t i = 0; i < num_agents; ++i) {
                if (cspaces[i].inCollision(joint_path[t][i].x(), joint_path[t][i].y())) {
                    std::cout << "[PATH VALIDATION] Agent " << i << " in collision at step " << t << " pos " << joint_path[t][i].transpose() << std::endl;
                    path_valid = false;
                }
            }
            for (size_t i = 0; i < num_agents; ++i) {
                for (size_t j = i+1; j < num_agents; ++j) {
                        double min_dist = problem.agent_properties[i].radius + problem.agent_properties[j].radius + 0.4; // match buffer used in planning
                    if ((joint_path[t][i] - joint_path[t][j]).norm() < min_dist) {
                        std::cout << "[PATH VALIDATION] Agents " << i << " and " << j << " collide at step " << t << " dist=" << (joint_path[t][i] - joint_path[t][j]).norm() << " < " << min_dist << std::endl;
                        path_valid = false;
                    }
                }
            }
            // Edge (motion) collision for each agent
            if (t > 0) {
                for (size_t i = 0; i < num_agents; ++i) {
                    if (!cspaces[i].isValidPath(joint_path[t-1][i], joint_path[t][i])) {
                        std::cout << "[PATH VALIDATION] Agent " << i << " edge collision from step " << (t-1) << " to " << t << std::endl;
                        path_valid = false;
                    }
                }
            }
        }
        multi_agent_path.valid = path_valid;
    } else {
        // Planner failed: output placeholder path for every agent
        for (size_t agent_idx = 0; agent_idx < num_agents; ++agent_idx) {
            multi_agent_path.agent_paths[agent_idx].waypoints.clear();
            multi_agent_path.agent_paths[agent_idx].waypoints.push_back(problem.agent_properties[agent_idx].q_init);
            multi_agent_path.agent_paths[agent_idx].waypoints.push_back(problem.agent_properties[agent_idx].q_goal);
        }
        multi_agent_path.valid = false;
    }
    return multi_agent_path;
}

// Old plan: fallback to new plan with internal C-space construction
amp::MultiAgentPath2D MyCentralPlanner::plan(const amp::MultiAgentProblem2D& problem, int n, double r, double pgoal, double epsilon) {
    // Build C-spaces as before
    std::vector<amp::MyTranslationalCSpace> cspaces;
    for (size_t agent_idx = 0; agent_idx < problem.agent_properties.size(); ++agent_idx) {
        const auto& agent = problem.agent_properties[agent_idx];
        std::size_t grid_resolution = std::max(40, std::min(120, (int)(80.0 / agent.radius)));
        cspaces.emplace_back(
            grid_resolution, grid_resolution,
            problem.x_min, problem.x_max,
            problem.y_min, problem.y_max,
            problem.obstacles,
            agent.radius
        );
    }
    return plan(problem, cspaces, n, r, pgoal, epsilon);
}

// MyDecentralPlanner: add new plan method with precomputed C-spaces
amp::MultiAgentPath2D MyDecentralPlanner::plan(const amp::MultiAgentProblem2D& problem,
    const std::vector<amp::MyTranslationalCSpace>& cspaces,
    int n, double r, double pgoal, double epsilon) {
    // Joint GoalBiasRRT in the coupled multi-agent space
    using JointConfig = std::vector<Eigen::Vector2d>;
    struct JointNode {
        JointConfig config;
        int parent_idx;
        JointNode(const JointConfig& c, int p) : config(c), parent_idx(p) {}
    };

    const size_t num_agents = problem.agent_properties.size();
    const double step_size = 0.1;
    const double goal_bias = 0.2;
    const double goal_threshold = 0.2;
    const int max_iterations = n;

    // Initial and goal joint configs
    JointConfig q_init, q_goal;
    for (size_t i = 0; i < num_agents; ++i) {
        q_init.push_back(problem.agent_properties[i].q_init);
        q_goal.push_back(problem.agent_properties[i].q_goal);
    }

    // Random number generation for each agent
    std::random_device rd;
    std::mt19937 gen(rd());
    std::vector<std::uniform_real_distribution<double>> x_dists, y_dists;
    for (size_t i = 0; i < num_agents; ++i) {
        x_dists.emplace_back(cspaces[i].x0Bounds().first, cspaces[i].x0Bounds().second);
        y_dists.emplace_back(cspaces[i].x1Bounds().first, cspaces[i].x1Bounds().second);
    }
    std::uniform_real_distribution<double> uniform(0.0, 1.0);

    // Tree of joint configs
    std::vector<JointNode> tree;
    tree.emplace_back(q_init, -1);

    auto is_valid_joint = [&](const JointConfig& config) {
        // Check each agent for obstacle collision
        for (size_t i = 0; i < num_agents; ++i) {
            if (cspaces[i].inCollision(config[i].x(), config[i].y())) {
                std::cout << "[JOINT RRT] Agent " << i << " in collision with obstacle at " << config[i].transpose() << std::endl;
                return false;
            }
        }
        // Check for inter-agent collisions (distance < sum of radii)
        for (size_t i = 0; i < num_agents; ++i) {
            for (size_t j = i+1; j < num_agents; ++j) {
                double min_dist = problem.agent_properties[i].radius + problem.agent_properties[j].radius;
                if ((config[i] - config[j]).norm() < min_dist) {
                    std::cout << "[JOINT RRT] Agents " << i << " and " << j << " in collision: dist=" << (config[i] - config[j]).norm() << " < " << min_dist << std::endl;
                    return false;
                }
            }
        }
        return true;
    };

    auto distance = [&](const JointConfig& a, const JointConfig& b) {
        double d = 0.0;
        for (size_t i = 0; i < num_agents; ++i) d += (a[i] - b[i]).squaredNorm();
        return std::sqrt(d);
    };

    int goal_idx = -1;
    for (int iter = 0; iter < max_iterations; ++iter) {
        // Sample joint config
        JointConfig q_rand(num_agents);
        if (uniform(gen) < goal_bias) {
            q_rand = q_goal;
        } else {
            for (size_t i = 0; i < num_agents; ++i) {
                q_rand[i].x() = x_dists[i](gen);
                q_rand[i].y() = y_dists[i](gen);
            }
        }
        // Find nearest node in tree
        int nearest_idx = 0;
        double min_dist = distance(tree[0].config, q_rand);
        for (size_t i = 1; i < tree.size(); ++i) {
            double d = distance(tree[i].config, q_rand);
            if (d < min_dist) { min_dist = d; nearest_idx = i; }
        }
        // Extend: move each agent toward its sampled goal by step_size
        JointConfig q_new = tree[nearest_idx].config;
        for (size_t i = 0; i < num_agents; ++i) {
            Eigen::Vector2d dir = q_rand[i] - q_new[i];
            double len = dir.norm();
            if (len > step_size) dir = dir / len * step_size;
            q_new[i] += dir;
        }
        // Check new joint config for obstacle and inter-agent collisions
        if (!is_valid_joint(q_new)) continue;
        // Check edge (motion) validity for each agent
        bool edge_valid = true;
        for (size_t i = 0; i < num_agents; ++i) {
            if (!cspaces[i].isValidPath(tree[nearest_idx].config[i], q_new[i])) {
                std::cout << "[JOINT RRT] Agent " << i << " edge collision from " << tree[nearest_idx].config[i].transpose() << " to " << q_new[i].transpose() << std::endl;
                edge_valid = false; break;
            }
        }
        if (!edge_valid) continue;
        // Add to tree
        tree.emplace_back(q_new, nearest_idx);
        // Check for goal
        if (distance(q_new, q_goal) < goal_threshold) {
            goal_idx = tree.size() - 1;
            break;
        }
    }

    amp::MultiAgentPath2D multi_agent_path;
    multi_agent_path.agent_paths.resize(num_agents);
    if (goal_idx != -1) {
        // Reconstruct joint path
        std::vector<JointConfig> joint_path;
        int idx = goal_idx;
        while (idx != -1) {
            joint_path.push_back(tree[idx].config);
            idx = tree[idx].parent_idx;
        }
        std::reverse(joint_path.begin(), joint_path.end());
        // Fill agent paths
        for (size_t agent_idx = 0; agent_idx < num_agents; ++agent_idx) {
            for (const auto& joint : joint_path) {
                multi_agent_path.agent_paths[agent_idx].waypoints.push_back(joint[agent_idx]);
            }
        }
        multi_agent_path.valid = true;
    } else {
        multi_agent_path.valid = false;
    }
    return multi_agent_path;
}



amp::MultiAgentPath2D MyDecentralPlanner::plan(const amp::MultiAgentProblem2D& problem) {
    amp::MultiAgentPath2D multi_agent_path;

    // Sequential planning with dynamic obstacle avoidance:
    // Each agent plans in order, treating higher-priority agents' planned paths as dynamic obstacles (occupancy at each time step)
    std::vector<amp::Path2D> planned_paths;

    for (size_t agent_idx = 0; agent_idx < problem.agent_properties.size(); ++agent_idx) {
        const auto& agent = problem.agent_properties[agent_idx];
        std::cout << "Decentralized Agent " << agent_idx << " radius: " << agent.radius << std::endl;

        std::size_t grid_resolution = std::max(40, std::min(120, (int)(80.0 / agent.radius)));
        amp::MyTranslationalCSpace cspace(
            grid_resolution, grid_resolution,
            problem.x_min, problem.x_max,
            problem.y_min, problem.y_max,
            problem.obstacles,
            agent.radius
        );

        // Plan initial path (ignoring dynamic obstacles)
        amp::Path2D agent_path = cspace.planPath(agent.q_init, agent.q_goal);

        // Dynamic obstacle avoidance: check for collisions with higher-priority agents at each time step
        // If collision, wait at current position until path is clear
        amp::Path2D safe_path;
        size_t t = 0;
        size_t max_other_len = 0;
        for (const auto& p : planned_paths) max_other_len = std::max(max_other_len, p.waypoints.size());
        size_t max_steps = std::max(agent_path.waypoints.size(), max_other_len);
        Eigen::Vector2d last_pos = agent.q_init;
    const size_t max_time_steps = 1000;
    const size_t max_wait_steps = 50; // max consecutive waits before replanning
    size_t time_steps = 0;
    size_t wait_steps = 0;
    bool exceeded_time_limit = false;
    Eigen::Vector2d replan_start = agent.q_init;
    while (t < max_steps) {
            // Get agent's intended position at this time step
            Eigen::Vector2d intended_pos = (t < agent_path.waypoints.size()) ? agent_path.waypoints[t] : agent_path.waypoints.back();

            // Check for collision with any higher-priority agent at this time step
            bool collision = false;
            for (size_t other_idx = 0; other_idx < planned_paths.size(); ++other_idx) {
                const auto& other_path = planned_paths[other_idx];
                Eigen::Vector2d other_pos = (t < other_path.waypoints.size()) ? other_path.waypoints[t] : other_path.waypoints.back();
                double dist = (intended_pos - other_pos).norm();
                    double min_dist = agent.radius + problem.agent_properties[other_idx].radius + 0.3; // increased buffer for safety
                if (dist < min_dist) {
                    collision = true;
                    break;
                }
            }

            if (collision) {
                // Wait at last safe position for an extra time step (do not advance t)
                safe_path.waypoints.push_back(last_pos);
                ++wait_steps;
                // If waited too long, replan from current position
                if (wait_steps >= max_wait_steps) {
                    std::cout << "[INFO] Agent " << agent_idx << " replanning from stuck position after " << wait_steps << " waits.\n";
                    // Replan from last_pos to goal, using updated C-space
                    agent_path = cspace.planPath(last_pos, agent.q_goal);
                    t = 0;
                    max_other_len = 0;
                    for (const auto& p : planned_paths) max_other_len = std::max(max_other_len, p.waypoints.size());
                    max_steps = std::max(agent_path.waypoints.size(), max_other_len);
                    wait_steps = 0;
                    // Optionally, clear safe_path and start a new one, or append to existing
                    // Here, we append to existing so the full trajectory is preserved
                    continue;
                }
            } else {
                // Move to intended position
                safe_path.waypoints.push_back(intended_pos);
                last_pos = intended_pos;
                ++t; // Only advance time if agent moves
                wait_steps = 0; // reset wait counter
            }
            ++time_steps;
            if (time_steps > max_time_steps) {
                exceeded_time_limit = true;
                std::cerr << "[ERROR] Agent " << agent_idx << " exceeded max time steps (" << max_time_steps << ") in decentralized planner. Marking path as invalid.\n";
                break;
            }
            // If reached goal and no other agents are moving, can break early
            bool all_others_done = true;
            for (const auto& p : planned_paths) {
                if (t < p.waypoints.size()) { all_others_done = false; break; }
            }
            if (t >= agent_path.waypoints.size() && all_others_done) break;
        }
        if (exceeded_time_limit) {
            safe_path.valid = false;
            multi_agent_path.valid = false;
        }
        planned_paths.push_back(safe_path);
        multi_agent_path.agent_paths.push_back(safe_path);
    }
    return multi_agent_path;
}