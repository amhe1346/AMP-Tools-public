#include "BenchmarkUtils.h"
#include <fstream>
#include <iostream>
#include <chrono>

namespace amp {

int get_tree_size(const amp::MultiAgentPath2D& path) {
    int total = 0;
    for (const auto& agent_path : path.agent_paths) {
        total += agent_path.waypoints.size();
    }
    return total;
}

void runBenchmarks(int min_agents, int max_agents, int num_runs, int n, double r, double pgoal, double epsilon, const std::string& output_prefix) {
    MyCentralPlanner central_planner;
    MyDecentralPlanner decentral_planner;
    for (int num_agents = min_agents; num_agents <= max_agents; ++num_agents) {
        std::cout << "[DEBUG] Starting C-space construction for " << num_agents << " agents..." << std::endl;
        // Use the same workspace for all runs of this agent count
        amp::MultiAgentProblem2D problem = HW8::getWorkspace1(num_agents);
        std::vector<amp::MyTranslationalCSpace> cached_cspaces;
        for (size_t agent_idx = 0; agent_idx < problem.agent_properties.size(); ++agent_idx) {
            const auto& agent = problem.agent_properties[agent_idx];
            std::size_t grid_resolution = std::max(40, std::min(120, (int)(80.0 / agent.radius)));
            std::cout << "[DEBUG] Constructing C-space for agent " << agent_idx << " with radius " << agent.radius << ", grid " << grid_resolution << "x" << grid_resolution << std::endl;
            cached_cspaces.emplace_back(
                grid_resolution, grid_resolution,
                problem.x_min, problem.x_max,
                problem.y_min, problem.y_max,
                problem.obstacles,
                agent.radius
            );
        }
        std::cout << "[DEBUG] Finished C-space construction." << std::endl;
        std::string filename = output_prefix + "_m" + std::to_string(num_agents) + ".csv";
        std::ofstream out(filename);
        if (!out.is_open()) {
            std::cerr << "Error: Could not open " << filename << " for writing." << std::endl;
            continue;
        }
        out << "planner,run,time,tree_size,valid,agent_count\n";
        // Single-agent benchmark loop
        for (int run = 0; run < num_runs; ++run) {
            auto start = std::chrono::high_resolution_clock::now();
            amp::MultiAgentPath2D single_agent_paths;
            for (int agent_idx = 0; agent_idx < num_agents; ++agent_idx) {
                const auto& agent = problem.agent_properties[agent_idx];
                std::size_t grid_resolution = std::max(40, std::min(120, (int)(80.0 / agent.radius)));
                std::cout << "[DEBUG] Agent " << agent_idx << ": q_init = [" << agent.q_init.x() << ", " << agent.q_init.y() << "], q_goal = [" << agent.q_goal.x() << ", " << agent.q_goal.y() << "], radius = " << agent.radius << ", grid = " << grid_resolution << std::endl;
                amp::MyTranslationalCSpace cspace(
                    grid_resolution, grid_resolution,
                    problem.x_min, problem.x_max,
                    problem.y_min, problem.y_max,
                    problem.obstacles,
                    agent.radius
                );
                amp::Problem2D single_problem;
                single_problem.q_init = agent.q_init;
                single_problem.q_goal = agent.q_goal;
                single_problem.x_min = problem.x_min;
                single_problem.x_max = problem.x_max;
                single_problem.y_min = problem.y_min;
                single_problem.y_max = problem.y_max;
                single_problem.obstacles = problem.obstacles;
                amp::Path2D agent_path = cspace.planPath(
                    single_problem.q_init, single_problem.q_goal
                );
                std::cout << "[DEBUG] Agent " << agent_idx << ": path waypoints = " << agent_path.waypoints.size() << std::endl;
                single_agent_paths.agent_paths.push_back(agent_path);
            }
            single_agent_paths.valid = true;
            auto end = std::chrono::high_resolution_clock::now();
            double single_time = std::chrono::duration<double>(end - start).count();
            int single_tree_size = get_tree_size(single_agent_paths);
            std::vector<std::vector<Eigen::Vector2d>> collision_states_single;
            bool valid_single = HW8::check(single_agent_paths, problem, collision_states_single, true);
            std::cout << "[DEBUG] Single-agent: time = " << single_time << "s, tree_size = " << single_tree_size << ", valid = " << valid_single << std::endl;
            if (single_agent_paths.agent_paths.empty()) {
                std::cerr << "Single-agent planner failed on run " << run << std::endl;
                out << "single-agent," << run << ",ERROR,0,0," << num_agents << "\n";
            } else {
                out << "single-agent," << run << "," << single_time << "," << single_tree_size << "," << (valid_single ? 1 : 0) << "," << num_agents << "\n";
            }
        }
        for (int run = 0; run < num_runs; ++run) {
            std::cout << "[DEBUG] Benchmark run " << (run+1) << "/" << num_runs << " for m=" << num_agents << "..." << std::endl;
            // Centralized planner
            auto start = std::chrono::high_resolution_clock::now();
            auto central_path = central_planner.plan(problem, cached_cspaces, n, r, pgoal, epsilon);
            auto end = std::chrono::high_resolution_clock::now();
            double central_time = std::chrono::duration<double>(end - start).count();
            int central_tree_size = get_tree_size(central_path);
            std::vector<std::vector<Eigen::Vector2d>> collision_states_central;
            bool valid_central = HW8::check(central_path, problem, collision_states_central, true);
            std::cout << "[DEBUG] Central: time = " << central_time << "s, tree_size = " << central_tree_size << ", valid = " << valid_central << std::endl;
            if (central_path.agent_paths.empty()) {
                std::cerr << "Central planner failed on run " << run << std::endl;
                out << "central," << run << ",ERROR,0,0," << num_agents << "\n";
            } else {
                out << "central," << run << "," << central_time << "," << central_tree_size << "," << (valid_central ? 1 : 0) << "," << num_agents << "\n";
            }

            // Decentralized planner
            start = std::chrono::high_resolution_clock::now();
            auto decentral_path = decentral_planner.plan(problem, cached_cspaces, n, r, pgoal, epsilon);
            end = std::chrono::high_resolution_clock::now();
            double decentral_time = std::chrono::duration<double>(end - start).count();
            int decentral_tree_size = get_tree_size(decentral_path);
            std::vector<std::vector<Eigen::Vector2d>> collision_states_decentral;
            bool valid_decentral = HW8::check(decentral_path, problem, collision_states_decentral, true);
            std::cout << "[DEBUG] Decentral: time = " << decentral_time << "s, tree_size = " << decentral_tree_size << ", valid = " << valid_decentral << std::endl;
            if (decentral_path.agent_paths.empty()) {
                std::cerr << "Decentral planner failed on run " << run << std::endl;
                out << "decentral," << run << ",ERROR,0,0," << num_agents << "\n";
            } else {
                out << "decentral," << run << "," << decentral_time << "," << decentral_tree_size << "," << (valid_decentral ? 1 : 0) << "," << num_agents << "\n";
            }
        }
        out.close();
        std::cout << "Benchmark data saved to " << filename << "\n";
    }
}

} // namespace amp
