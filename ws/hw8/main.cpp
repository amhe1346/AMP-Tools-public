// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"
#include "hw/HW8.h"
#include "MyMultiAgentPlanners.h"
#include "../shared/MyTranslationalCSpace.h"

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
    // Initializing workspace 1 with 3 agents
    amp::RNG::seed(amp::RNG::randiUnbounded());
    MultiAgentPath2D path;
    MultiAgentProblem2D problem = HW8::getWorkspace1(3);
    std::vector<std::vector<Eigen::Vector2d>> collision_states;

   

    // --- Decentralized: Per-agent Goal Bias RRT for m=2 agents (no coordination) ---
    int m = 2;
    MultiAgentProblem2D problem2 = problem;
    problem2.agent_properties.resize(m); // Only use first two agents
    int n = 7500;
    double r = 0.5;
    double pgoal = 0.05;
    double epsilon = 0.25;
    MultiAgentPath2D path2;
    path2.agent_paths.resize(m);
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
        // Use RRT with specified params
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
        path2.agent_paths[agent_idx] = agent_path;
    }
    path2.valid = true;
    std::vector<std::vector<Eigen::Vector2d>> collision_states2;
    bool isValid2 = HW8::check(path2, problem2, collision_states2);
    Visualizer::makeFigure(problem2, path2, collision_states2);

    // Solve using the DecentralPlanner (multi-agent, but not fully centralized)
    MyDecentralPlanner decentral_planner;
    path = decentral_planner.plan(problem);
    bool isValidDecentral = HW8::check(path, problem, collision_states);
    Visualizer::makeFigure(problem, path, collision_states);

    // Solve using a decentralized approach
    // MyDecentralPlanner decentral_planner;
    //collision_states = {{}};
    //W8::generateAndCheck(decentral_planner, path, problem, collision_states);
    //Visualizer::makeFigure(problem, path, collision_states);

    // Visualize and grade methods
    Visualizer::saveFigures();
    // HW8::grade<MyCentralPlanner, MyDecentralPlanner>("firstName.lastName@colorado.edu", argc, argv, std::make_tuple(), std::make_tuple());
    return 0;
}