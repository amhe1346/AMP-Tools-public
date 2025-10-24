#include "MyMultiAgentPlanners.h"
#include "../shared/MyCollisionChecker.h"
#include "../shared/ObstacleExpansion.h"
#include "../shared/MyTranslationalCSpace.h"
#include <random>
#include <algorithm>

amp::MultiAgentPath2D MyCentralPlanner::plan(const amp::MultiAgentProblem2D& problem) {
    amp::MultiAgentPath2D multi_agent_path;
    
    // For centralized planning, we can plan for each agent individually using PRM
    // This is a simplified approach - more advanced centralized planners would 
    // consider joint configuration space and inter-agent collision avoidance
    
    for (size_t agent_idx = 0; agent_idx < problem.agent_properties.size(); ++agent_idx) {
        const auto& agent = problem.agent_properties[agent_idx];
        
        // Debug: Print agent radius to see if it's reasonable
        std::cout << "Agent " << agent_idx << " radius: " << agent.radius << std::endl;
        
        // Create C-space for this agent with appropriate resolution
        // Use higher resolution for smaller agents, lower for larger agents
        std::size_t grid_resolution = std::max(50, std::min(200, (int)(100.0 / agent.radius)));
        
        amp::MyTranslationalCSpace cspace(
            grid_resolution, grid_resolution,
            problem.x_min, problem.x_max,
            problem.y_min, problem.y_max,
            problem.obstacles,
            agent.radius
        );
        
        // Plan path using C-space (which internally uses MyGoalBiasRRT)
        amp::Path2D agent_path = cspace.planPath(agent.q_init, agent.q_goal);
        
        multi_agent_path.agent_paths.push_back(agent_path);
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