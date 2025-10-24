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
    int num_agents = 3;
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

        std::ofstream out("bin/benchmark_results.csv");
        if (!out.is_open()) {
            std::cerr << "Error: Could not open bin/benchmark_results.csv for writing." << std::endl;
            return 1;
        }
        out << "planner,run,time,tree_size\n";

    int num_runs = 10; // Reduce to 10 for faster testing
    for (int run = 0; run < num_runs; ++run) {
        std::cout << "Benchmark run " << (run+1) << "/" << num_runs << "..." << std::endl;
        // Centralized
        auto start = std::chrono::high_resolution_clock::now();
            auto central_path = central_planner.plan(problem, n, r, pgoal, epsilon);
        auto end = std::chrono::high_resolution_clock::now();
        double central_time = std::chrono::duration<double>(end - start).count();
        int central_tree_size = get_tree_size(central_path);
        if (central_path.agent_paths.empty()) {
            std::cerr << "Central planner failed on run " << run << std::endl;
            out << "central," << run << ",ERROR,0\n";
        } else {
            out << "central," << run << "," << central_time << "," << central_tree_size << "\n";
        }

        // Decentralized
        start = std::chrono::high_resolution_clock::now();
        auto decentral_path = decentral_planner.plan(problem);
        end = std::chrono::high_resolution_clock::now();
        double decentral_time = std::chrono::duration<double>(end - start).count();
        int decentral_tree_size = get_tree_size(decentral_path);
        if (decentral_path.agent_paths.empty()) {
            std::cerr << "Decentral planner failed on run " << run << std::endl;
            out << "decentral," << run << ",ERROR,0\n";
        } else {
            out << "decentral," << run << "," << decentral_time << "," << decentral_tree_size << "\n";
        }
    }
    out.close();
    std::cout << "Benchmark data saved to benchmark_results.csv\n";
    return 0;
}
