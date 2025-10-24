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
    // Default values: n=1000, r=1.0, pgoal=0.1, epsilon=0.5
    return plan(problem, 1000, 1.0, 0.1, 0.5);
}

MyCentralPlanner::~MyCentralPlanner() {}
#include <algorithm>

amp::MultiAgentPath2D MyCentralPlanner::plan(const amp::MultiAgentProblem2D& problem, int n, double r, double pgoal, double epsilon) {
    amp::MultiAgentPath2D multi_agent_path;
    // --- JOINT CONFIGURATION SPACE BFS IMPLEMENTATION ---
    std::vector<amp::MyTranslationalCSpace> cspaces;
    std::vector<std::pair<std::size_t, std::size_t>> start_cells, goal_cells;
    for (size_t agent_idx = 0; agent_idx < problem.agent_properties.size(); ++agent_idx) {
        const auto& agent = problem.agent_properties[agent_idx];
        std::size_t grid_resolution = std::max(40, std::min(120, (int)(80.0 / agent.radius)));
        amp::MyTranslationalCSpace cspace(
            grid_resolution, grid_resolution,
            problem.x_min, problem.x_max,
            problem.y_min, problem.y_max,
            problem.obstacles,
            agent.radius
        );
        cspaces.push_back(cspace);
        start_cells.push_back(cspace.getCellFromPoint(agent.q_init.x(), agent.q_init.y()));
        goal_cells.push_back(cspace.getCellFromPoint(agent.q_goal.x(), agent.q_goal.y()));
    }

    struct JointState {
        std::vector<std::pair<std::size_t, std::size_t>> cells; // one per agent (i,j)
        int time = 0;
        std::shared_ptr<JointState> parent;
        bool operator==(const JointState& other) const { return cells == other.cells; }
    };
    struct JointStateHash {
        std::size_t operator()(const JointState& s) const {
            std::size_t h = 0;
            for (const auto& c : s.cells) h ^= std::hash<std::size_t>()(c.first) ^ std::hash<std::size_t>()(c.second);
            return h;
        }
    };

    auto is_valid = [&](const JointState& s) {
        // Check each agent for obstacle collision
        for (size_t i = 0; i < s.cells.size(); ++i) {
            if (cspaces[i](s.cells[i].first, s.cells[i].second)) return false; // true means in collision
        }
        // Check for inter-agent collisions (same cell)
        for (size_t i = 0; i < s.cells.size(); ++i) {
            for (size_t j = i+1; j < s.cells.size(); ++j) {
                if (s.cells[i] == s.cells[j]) return false;
            }
        }
        return true;
    };

    // 2. BFS in joint space
    std::queue<std::shared_ptr<JointState>> q;
    std::unordered_set<JointState, JointStateHash> visited;
    auto joint_start = std::make_shared<JointState>();
    joint_start->cells = start_cells;
    joint_start->time = 0;
    joint_start->parent = nullptr;
    q.push(joint_start);
    visited.insert(*joint_start);
    std::shared_ptr<JointState> joint_goal = nullptr;

    // 3. Define possible moves (4-connected grid + stay)
    std::vector<std::pair<int,int>> moves = {
        {1,0}, {-1,0}, {0,1}, {0,-1}, {0,0}
    };
    // Use r as step size, n as max_iterations, pgoal as goal bias, epsilon as goal threshold
    int max_iterations = n;
    double step_size = r;
    double goal_bias = pgoal;
    double goal_threshold = epsilon;

    while (!q.empty()) {
        auto curr = q.front(); q.pop();
        if (curr->cells == goal_cells) { joint_goal = curr; break; }
        // Generate all combinations of moves for all agents
        std::vector<std::vector<std::pair<int,int>>> agent_moves(curr->cells.size(), moves);
        // For each agent, try all moves (cartesian product)
        std::vector<size_t> idx(curr->cells.size(), 0);
        while (true) {
            // Build next joint state
            JointState next;
            next.cells.resize(curr->cells.size());
            for (size_t i = 0; i < curr->cells.size(); ++i) {
                next.cells[i].first = static_cast<int>(curr->cells[i].first) + agent_moves[i][idx[i]].first;
                next.cells[i].second = static_cast<int>(curr->cells[i].second) + agent_moves[i][idx[i]].second;
                // Clamp to grid bounds
                auto sz = cspaces[i].size();
                if (next.cells[i].first < 0) next.cells[i].first = 0;
                if (next.cells[i].second < 0) next.cells[i].second = 0;
                if (next.cells[i].first >= sz.first) next.cells[i].first = sz.first - 1;
                if (next.cells[i].second >= sz.second) next.cells[i].second = sz.second - 1;
            }
            next.time = curr->time + 1;
            next.parent = curr;
            if (is_valid(next) && !visited.count(next)) {
                visited.insert(next);
                q.push(std::make_shared<JointState>(next));
            }
            // Increment idx (cartesian product)
            size_t k = 0;
            while (k < idx.size()) {
                idx[k]++;
                if (idx[k] < agent_moves[k].size()) break;
                idx[k] = 0; k++;
            }
            if (k == idx.size()) break;
        }
    }

    // 4. Extract joint path if found
    if (joint_goal) {
        std::vector<std::vector<std::pair<std::size_t, std::size_t>>> joint_path;
        for (auto ptr = joint_goal; ptr; ptr = ptr->parent) {
            joint_path.push_back(ptr->cells);
        }
        std::reverse(joint_path.begin(), joint_path.end());
        // Convert grid path to world path for each agent
        multi_agent_path.agent_paths.resize(cspaces.size());
        for (size_t agent_idx = 0; agent_idx < cspaces.size(); ++agent_idx) {
            auto sz = cspaces[agent_idx].size();
            auto x_bounds = cspaces[agent_idx].x0Bounds();
            auto y_bounds = cspaces[agent_idx].x1Bounds();
            double cell_width_x = (x_bounds.second - x_bounds.first) / sz.first;
            double cell_width_y = (y_bounds.second - y_bounds.first) / sz.second;
            for (const auto& step : joint_path) {
                double x = x_bounds.first + (step[agent_idx].first + 0.5) * cell_width_x;
                double y = y_bounds.first + (step[agent_idx].second + 0.5) * cell_width_y;
                multi_agent_path.agent_paths[agent_idx].waypoints.emplace_back(x, y);
            }
        }
        multi_agent_path.valid = true;
        return multi_agent_path;
    } else {
        std::cerr << "[ERROR] No joint path found in joint BFS." << std::endl;
        multi_agent_path.valid = false;
        return multi_agent_path;
    }
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
                double min_dist = agent.radius + problem.agent_properties[other_idx].radius + 0.1; // increased buffer for safety
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