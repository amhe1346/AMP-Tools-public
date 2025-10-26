// This includes all of the necessary header files in the toolbox

#include "AMPCore.h"
#include "hw/HW8.h"
#include "MyMultiAgentPlanners.h"
#include "../shared/MyTranslationalCSpace.h"
#include "../shared/BenchmarkUtils.h"

using namespace amp;

void timer_example() {
    double startTime;
    amp::Timer timer("timer");
    for (int i=0; i < 5; ++i) {
        startTime = timer.now(TimeUnit::ms);  
        std::cout << "Press any key to continue...\n";
        std::cin.get();
        std::cout << "Time since last run: " << timer.now(TimeUnit::ms) - startTime << std::endl;
    }
    timer.stop();
    std::cout << "Total time since last run: " << Profiler::getTotalProfile("timer") << std::endl;
}

int main(int argc, char** argv) {
    // Example: Call benchmarking function
    int num_runs = 100;
    int min_agents = 2, max_agents = 6;
    int n = 7500;
    double r = 0.5;
    double pgoal = 0.05;
    double epsilon = 0.25;
    amp::runBenchmarks(min_agents, max_agents, num_runs, n, r, pgoal, epsilon, "bin/benchmark_results");

    // Declare and initialize missing variables
    // Generate benchmark CSV files for m = 2 to 6
    for (int m = 2; m <= 6; ++m) {
        amp::MultiAgentProblem2D problem2 = HW8::getWorkspace1(m);
        std::ofstream out("bin/main_benchmark_results_m" + std::to_string(m) + ".csv");
        if (!out.is_open()) {
            std::cerr << "Error: Could not open bin/main_benchmark_results_m" << m << ".csv for writing." << std::endl;
            continue;
        }
        out << "planner,run,time,tree_size,valid\n";
        for (int run = 0; run < num_runs; ++run) {
            amp::MultiAgentPath2D run_paths;
            auto start = std::chrono::high_resolution_clock::now();
            for (int agent_idx = 0; agent_idx < m; ++agent_idx) {
                const auto& agent = problem2.agent_properties[agent_idx];
                std::size_t grid_resolution = std::max(40, std::min(120, (int)(80.0 / agent.radius)));
                amp::MyTranslationalCSpace cspace(
                    grid_resolution, grid_resolution,
                    problem2.x_min, problem2.x_max,
                    problem2.y_min, problem2.y_max,
                    problem2.obstacles,
                    agent.radius
                );
                amp::Problem2D single_problem;
                single_problem.q_init = agent.q_init;
                single_problem.q_goal = agent.q_goal;
                single_problem.x_min = problem2.x_min;
                single_problem.x_max = problem2.x_max;
                single_problem.y_min = problem2.y_min;
                single_problem.y_max = problem2.y_max;
                single_problem.obstacles = problem2.obstacles;
                amp::Path2D agent_path = cspace.planPath(
                    single_problem.q_init, single_problem.q_goal
                );
                run_paths.agent_paths.push_back(agent_path);
            }
            run_paths.valid = true;
            auto end = std::chrono::high_resolution_clock::now();
            double run_time = std::chrono::duration<double>(end - start).count();
            int tree_size = 0;
            for (const auto& agent_path : run_paths.agent_paths) {
                tree_size += agent_path.waypoints.size();
            }
            std::vector<std::vector<Eigen::Vector2d>> collision_states2;
            bool isValid2 = HW8::check(run_paths, problem2, collision_states2);
            out << "single-agent," << run << "," << run_time << "," << tree_size << "," << (isValid2 ? 1 : 0) << "\n";
        }
        out.close();
    }

    // Optionally, run DecentralPlanner for each m
    #include "MyDecentralPlanner.cpp" // Directly include implementation for demonstration
    for (int m = 2; m <= 5; ++m) {
        amp::MultiAgentProblem2D problem = HW8::getWorkspace1(m);
        std::vector<Eigen::Vector2d> q_inits, q_goals;
        std::vector<double> radii;
        for (const auto& agent : problem.agent_properties) {
            q_inits.push_back(agent.q_init);
            q_goals.push_back(agent.q_goal);
            radii.push_back(agent.radius);
        }
        DecentralPlanner planner;
        auto paths = planner.plan(q_inits, q_goals, problem.x_min, problem.x_max, problem.y_min, problem.y_max, problem.obstacles, radii);
        // Convert paths to MultiAgentPath2D for grading/visualization
        amp::MultiAgentPath2D mapaths;
        mapaths.agent_paths = paths;
        std::vector<std::vector<Eigen::Vector2d>> collision_states;
        bool isValidDecentral = HW8::check(mapaths, problem, collision_states);
        Visualizer::makeFigure(problem, mapaths, collision_states);
    }

    // Visualize and grade methods
    Visualizer::saveFigures();
    HW8::grade<MyCentralPlanner, MyDecentralPlanner>("amy.heerten@colorado.edu", argc, argv, std::make_tuple(), std::make_tuple());
    return 0;
}