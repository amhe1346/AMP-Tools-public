// benchmarks.cpp
#include "AMPCore.h"
#include "hw/HW8.h"
#include "../hw8/MyMultiAgentPlanners.h"
#include <fstream>
#include <vector>
#include <iostream>
#include <chrono>
#include <filesystem>

using namespace amp;

int get_tree_size(const amp::MultiAgentPath2D& path) {
    // Placeholder: use total number of waypoints across all agents as a proxy for tree size
    int total = 0;
    for (const auto& agent_path : path.agent_paths) {
        total += agent_path.waypoints.size();
    }
    return total;
}

int main(int argc, char** argv) {
    std::cout << "[DEBUG] Starting C-space construction for all agents..." << std::endl;
    int num_agents = 2;
    if (argc > 1) {
        num_agents = std::max(1, std::atoi(argv[1]));
    }
    std::cout << "Benchmarking with " << num_agents << " agents." << std::endl;
    amp::MultiAgentProblem2D problem = HW8::getWorkspace1(num_agents);
    MyCentralPlanner central_planner;
    MyDecentralPlanner decentral_planner;

    // Fast parameters from main.cpp
    int n = 7500;
    double r = 0.5;
    double pgoal = 0.05;
    double epsilon = 0.25;

    // --- C-space caching: construct once for all runs ---
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

    std::ofstream out("bin/benchmark_results.csv");
    if (!out.is_open()) {
        std::cerr << "Error: Could not open bin/benchmark_results.csv for writing." << std::endl;
        return 1;
    }
    out << "planner,run,time,tree_size\n";

    int num_runs = 10; // Reduce to 10 for faster testing
    for (int run = 0; run < num_runs; ++run) {
        std::cout << "[DEBUG] Benchmark run " << (run+1) << "/" << num_runs << "..." << std::endl;
        auto start = std::chrono::high_resolution_clock::now();
        auto central_path = central_planner.plan(problem, cached_cspaces, n, r, pgoal, epsilon);
        auto end = std::chrono::high_resolution_clock::now();
        double central_time = std::chrono::duration<double>(end - start).count();
        int central_tree_size = get_tree_size(central_path);
        std::cout << "[DEBUG] Run " << run << ": time = " << central_time << "s, tree_size = " << central_tree_size << std::endl;
        if (central_path.agent_paths.empty()) {
            std::cerr << "Central planner failed on run " << run << std::endl;
            out << "central," << run << ",ERROR,0\n";
        } else {
            out << "central," << run << "," << central_time << "," << central_tree_size << "\n";
        }
    }
    out.close();
    std::cout << "Benchmark data saved to benchmark_results.csv\n";
    return 0;
}
