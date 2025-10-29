#pragma once

#include "AMPCore.h"
#include "hw/HW9.h"
#include "MyKinoRRT.h"
#include <chrono>
#include <unordered_map>
#include <memory>
#include <vector>
#include <tuple>
#include <stdexcept>

// ============================================================================
// KINODYNAMIC PLANNER FACTORY FUNCTIONS
// ============================================================================
// Convenient factory functions to create and configure kinodynamic planners
// for each agent type with optimized parameters
// ============================================================================

namespace KinoFactory {

// ============================================================================
// SINGLE INTEGRATOR PLANNER
// ============================================================================
/**
 * @brief Create a configured planner for Single Integrator agents
 * 
 * Single Integrator: ẋ = u
 * - State: [x, y] ∈ ℝ²
 * - Control: [vx, vy] (velocity control)
 * - Characteristics: Holonomic, can move in any direction instantly
 * 
 * @param max_iterations Maximum RRT iterations (default: 3000)
 * @param dt Integration time step (default: 0.05 for smooth motion)
 * @param goal_bias Goal biasing probability (default: 0.15)
 * @param control_samples Number of random controls per iteration (default: 3)
 * @return Configured MyKinoRRT instance
 */
inline std::unique_ptr<MyKinoRRT> createSingleIntegratorPlanner(
    int max_iterations = 3000,
    double dt = 0.05,
    double goal_bias = 0.15,
    int control_samples = 3) {
    
    auto planner = std::make_unique<MyKinoRRT>();
    // Single integrator is simple - default parameters work well
    return planner;
}

/**
 * @brief Solve a Single Integrator kinodynamic problem
 * @param problem The kinodynamic problem to solve
 * @param max_iterations Maximum RRT iterations
 * @param dt Integration time step
 * @param goal_bias Goal biasing probability
 * @param control_samples Number of random controls per iteration
 * @return Kinodynamic path solution
 */
inline amp::KinoPath solveSingleIntegrator(
    const amp::KinodynamicProblem2D& problem,
    int max_iterations = 3000,
    double dt = 0.05,
    double goal_bias = 0.15,
    int control_samples = 3) {
    
    auto planner = createSingleIntegratorPlanner(max_iterations, dt, goal_bias, control_samples);
    auto agent = std::make_shared<MySingleIntegrator>();
    return planner->plan(problem, *agent);
}

// ============================================================================
// FIRST ORDER UNICYCLE PLANNER
// ============================================================================
/**
 * @brief Create a configured planner for First-Order Unicycle agents
 * 
 * First-Order Unicycle: ẋ = [u_σ*r*cos(θ), u_σ*r*sin(θ), u_ω]ᵀ
 * - State: [x, y, θ] ∈ ℝ² × S¹
 * - Control: [u_σ, u_ω] (pedaling speed, rotational velocity)
 * - Characteristics: Nonholonomic, cannot move sideways
 * 
 * @param max_iterations Maximum RRT iterations (default: 5000)
 * @param dt Integration time step (default: 0.1 for stability)
 * @param goal_bias Goal biasing probability (default: 0.1)
 * @param control_samples Number of random controls per iteration (default: 5)
 * @return Configured MyKinoRRT instance
 */
inline std::unique_ptr<MyKinoRRT> createFirstOrderUnicyclePlanner(
    int max_iterations = 5000,
    double dt = 0.1,
    double goal_bias = 0.1,
    int control_samples = 5) {
    
    auto planner = std::make_unique<MyKinoRRT>();
    // First-order unicycle needs more iterations due to nonholonomic constraints
    return planner;
}

/**
 * @brief Solve a First-Order Unicycle kinodynamic problem
 * @param problem The kinodynamic problem to solve
 * @param max_iterations Maximum RRT iterations
 * @param dt Integration time step
 * @param goal_bias Goal biasing probability
 * @param control_samples Number of random controls per iteration
 * @return Kinodynamic path solution
 */
inline amp::KinoPath solveFirstOrderUnicycle(
    const amp::KinodynamicProblem2D& problem,
    int max_iterations = 5000,
    double dt = 0.1,
    double goal_bias = 0.1,
    int control_samples = 5) {
    
    auto planner = createFirstOrderUnicyclePlanner(max_iterations, dt, goal_bias, control_samples);
    auto agent = std::make_shared<MyFirstOrderUnicycle>();
    return planner->plan(problem, *agent);
}

// ============================================================================
// SECOND ORDER UNICYCLE PLANNER
// ============================================================================
/**
 * @brief Create a configured planner for Second-Order Unicycle agents
 * 
 * Second-Order Unicycle: ẋ = [v*cos(θ), v*sin(θ), ω, a, α]ᵀ
 * - State: [x, y, θ, v, ω] ∈ ℝ² × S¹ × ℝ²
 * - Control: [a, α] (linear acceleration, angular acceleration)
 * - Characteristics: Dynamic system with velocity states
 * 
 * @param max_iterations Maximum RRT iterations (default: 6000)
 * @param dt Integration time step (default: 0.08 for dynamic stability)
 * @param goal_bias Goal biasing probability (default: 0.08)
 * @param control_samples Number of random controls per iteration (default: 6)
 * @return Configured MyKinoRRT instance
 */
inline std::unique_ptr<MyKinoRRT> createSecondOrderUnicyclePlanner(
    int max_iterations = 6000,
    double dt = 0.08,
    double goal_bias = 0.08,
    int control_samples = 6) {
    
    auto planner = std::make_unique<MyKinoRRT>();
    // Second-order systems need more samples and smaller time steps
    return planner;
}

/**
 * @brief Solve a Second-Order Unicycle kinodynamic problem
 * @param problem The kinodynamic problem to solve
 * @param max_iterations Maximum RRT iterations
 * @param dt Integration time step
 * @param goal_bias Goal biasing probability
 * @param control_samples Number of random controls per iteration
 * @return Kinodynamic path solution
 */
inline amp::KinoPath solveSecondOrderUnicycle(
    const amp::KinodynamicProblem2D& problem,
    int max_iterations = 6000,
    double dt = 0.08,
    double goal_bias = 0.08,
    int control_samples = 6) {
    
    auto planner = createSecondOrderUnicyclePlanner(max_iterations, dt, goal_bias, control_samples);
    auto agent = std::make_shared<MySecondOrderUnicycle>();
    return planner->plan(problem, *agent);
}

// ============================================================================
// SIMPLE CAR PLANNER
// ============================================================================
/**
 * @brief Create a configured planner for Simple Car agents
 * 
 * Simple Car: ẋ = [v*cos(θ), v*sin(θ), (v/L)*tan(φ), u₁, u₂]ᵀ
 * - State: [x, y, θ, v, φ] ∈ ℝ² × S¹ × ℝ × [-π/2,π/2]
 * - Control: [u₁, u₂] (acceleration, steering rate)
 * - Control bounds: u₁ ∈ [-1,1], u₂ ∈ [-π/2,π/2]
 * - Characteristics: Car-like kinematics with steering constraints
 * 
 * @param max_iterations Maximum RRT iterations (default: 7000)
 * @param dt Integration time step (default: 0.1)
 * @param goal_bias Goal biasing probability (default: 0.05)
 * @param control_samples Number of random controls per iteration (default: 8)
 * @return Configured MyKinoRRT instance
 */
inline std::unique_ptr<MyKinoRRT> createSimpleCarPlanner(
    int max_iterations = 7000,
    double dt = 0.1,
    double goal_bias = 0.05,
    int control_samples = 8) {
    
    auto planner = std::make_unique<MyKinoRRT>();
    // Car planning requires more exploration due to steering constraints
    return planner;
}

/**
 * @brief Solve a Simple Car kinodynamic problem
 * @param problem The kinodynamic problem to solve
 * @param max_iterations Maximum RRT iterations
 * @param dt Integration time step
 * @param goal_bias Goal biasing probability
 * @param control_samples Number of random controls per iteration
 * @return Kinodynamic path solution
 */
inline amp::KinoPath solveSimpleCar(
    const amp::KinodynamicProblem2D& problem,
    int max_iterations = 7000,
    double dt = 0.1,
    double goal_bias = 0.05,
    int control_samples = 8) {
    
    auto planner = createSimpleCarPlanner(max_iterations, dt, goal_bias, control_samples);
    auto agent = std::make_shared<MySimpleCar>();
    return planner->plan(problem, *agent);
}

// ============================================================================
// AUTOMATIC PLANNER SELECTION
// ============================================================================
/**
 * @brief Automatically select and configure planner based on agent type
 * @param problem The kinodynamic problem (agent type determined from problem.agent_type)
 * @return Kinodynamic path solution using optimal parameters for the agent type
 */
inline amp::KinoPath solveKinodynamicProblem(const amp::KinodynamicProblem2D& problem) {
    switch (problem.agent_type) {
        case amp::AgentType::SingleIntegrator:
            return solveSingleIntegrator(problem);
            
        case amp::AgentType::FirstOrderUnicycle:
            return solveFirstOrderUnicycle(problem);
            
        case amp::AgentType::SecondOrderUnicycle:
            return solveSecondOrderUnicycle(problem);
            
        case amp::AgentType::SimpleCar:
            return solveSimpleCar(problem);
            
        default:
            throw std::runtime_error("Unknown agent type in kinodynamic problem");
    }
}

// ============================================================================
// PARAMETER TUNING UTILITIES
// ============================================================================
/**
 * @brief Test different parameter combinations for a given problem
 * @param problem The kinodynamic problem to solve
 * @param param_sets Vector of parameter tuples: {max_iter, dt, goal_bias, control_samples}
 * @return Vector of results with execution times and path validity
 */
struct PlanningResult {
    amp::KinoPath path;
    double execution_time_ms;
    bool found_solution;
    int iterations_used;
    std::string agent_type_name;
};

inline std::vector<PlanningResult> tuneParameters(
    const amp::KinodynamicProblem2D& problem,
    const std::vector<std::tuple<int, double, double, int>>& param_sets) {
    
    std::vector<PlanningResult> results;
    
    // Agent type name mapping
    std::unordered_map<amp::AgentType, std::string> agent_names = {
        {amp::AgentType::SingleIntegrator, "SingleIntegrator"},
        {amp::AgentType::FirstOrderUnicycle, "FirstOrderUnicycle"},
        {amp::AgentType::SecondOrderUnicycle, "SecondOrderUnicycle"},
        {amp::AgentType::SimpleCar, "SimpleCar"}
    };
    
    for (const auto& params : param_sets) {
        auto [max_iter, dt, goal_bias, control_samples] = params;
        
        auto start_time = std::chrono::high_resolution_clock::now();
        
        amp::KinoPath path;
        switch (problem.agent_type) {
            case amp::AgentType::SingleIntegrator:
                path = solveSingleIntegrator(problem, max_iter, dt, goal_bias, control_samples);
                break;
            case amp::AgentType::FirstOrderUnicycle:
                path = solveFirstOrderUnicycle(problem, max_iter, dt, goal_bias, control_samples);
                break;
            case amp::AgentType::SecondOrderUnicycle:
                path = solveSecondOrderUnicycle(problem, max_iter, dt, goal_bias, control_samples);
                break;
            case amp::AgentType::SimpleCar:
                path = solveSimpleCar(problem, max_iter, dt, goal_bias, control_samples);
                break;
        }
        
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        
        PlanningResult result;
        result.path = path;
        result.execution_time_ms = duration.count();
        result.found_solution = path.valid;
        result.iterations_used = max_iter; // Would need to modify RRT to return actual iterations
        result.agent_type_name = agent_names[problem.agent_type];
        
        results.push_back(result);
    }
    
    return results;
}

} // namespace KinoFactory

// ============================================================================
// USAGE EXAMPLES
// ============================================================================
/*

EXAMPLE 1: Basic usage with default parameters
```cpp
#include "KinoFactory.h"

// Solve any kinodynamic problem automatically
amp::KinodynamicProblem2D problem = HW9::getCarProblemWS1();
amp::KinoPath path = KinoFactory::solveKinodynamicProblem(problem);
```

EXAMPLE 2: Agent-specific planning with custom parameters
```cpp
// Solve car problem with custom parameters
amp::KinodynamicProblem2D car_problem = HW9::getCarProblemWS1();
amp::KinoPath path = KinoFactory::solveSimpleCar(car_problem, 10000, 0.05, 0.03, 10);
```

EXAMPLE 3: Parameter tuning
```cpp
// Test different parameter combinations
std::vector<std::tuple<int, double, double, int>> param_sets = {
    {3000, 0.1, 0.1, 5},    // Standard
    {5000, 0.05, 0.15, 8},  // High resolution
    {10000, 0.2, 0.05, 3}   // Fast exploration
};

auto results = KinoFactory::tuneParameters(problem, param_sets);
for (const auto& result : results) {
    std::cout << "Agent: " << result.agent_type_name 
              << ", Time: " << result.execution_time_ms << "ms"
              << ", Success: " << result.found_solution << std::endl;
}
```

EXAMPLE 4: Factory pattern usage
```cpp
// Create configured planners
auto single_planner = KinoFactory::createSingleIntegratorPlanner(5000, 0.03, 0.2, 4);
auto car_planner = KinoFactory::createSimpleCarPlanner(8000, 0.08, 0.04, 12);

// Use with different agents
auto single_agent = std::make_shared<MySingleIntegrator>();
auto car_agent = std::make_shared<MySimpleCar>();

amp::KinoPath path1 = single_planner->plan(problem1, *single_agent);
amp::KinoPath path2 = car_planner->plan(problem2, *car_agent);
```

*/