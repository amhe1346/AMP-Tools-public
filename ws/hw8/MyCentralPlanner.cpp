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
    
    // For decentralized planning, each agent plans independently
    // This is a basic implementation - more advanced versions would consider
    // dynamic obstacles, priority ordering, and replanning
    
    for (size_t agent_idx = 0; agent_idx < problem.agent_properties.size(); ++agent_idx) {
        const auto& agent = problem.agent_properties[agent_idx];
        
        // Debug: Print agent radius for decentralized planner
        std::cout << "Decentralized Agent " << agent_idx << " radius: " << agent.radius << std::endl;
        
        // Create C-space for this agent 
        // Use moderate resolution for decentralized planning (faster)
        std::size_t grid_resolution = std::max(40, std::min(120, (int)(80.0 / agent.radius)));
        
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