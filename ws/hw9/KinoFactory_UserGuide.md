# Kinodynamic Planner Factory Functions - User Guide

## Overview

The `KinoFactory` namespace provides convenient factory functions to create and configure kinodynamic planners for different agent types with optimized parameters. This eliminates the need to manually tune parameters for each agent type and provides a clean interface for kinodynamic motion planning.

## Quick Start

### Automatic Planning (Recommended)
```cpp
#include "KinoFactory.h"

// Load any kinodynamic problem
amp::KinodynamicProblem2D problem = HW9::getCarProblemWS1();

// Solve automatically with optimal parameters
amp::KinoPath path = KinoFactory::solveKinodynamicProblem(problem);

// Check and visualize
HW9::check(path, problem);
if (path.valid) {
    Visualizer::makeFigure(problem, path, false);
}
```

## Agent-Specific Planning

### Single Integrator
**System**: ẋ = u (Holonomic, direct velocity control)
```cpp
// Default parameters (optimized for Single Integrator)
amp::KinoPath path = KinoFactory::solveSingleIntegrator(problem);

// Custom parameters
amp::KinoPath path = KinoFactory::solveSingleIntegrator(
    problem,
    3000,    // max_iterations
    0.05,    // dt (time step)
    0.15,    // goal_bias
    3        // control_samples
);
```

### First-Order Unicycle
**System**: ẋ = [u_σ·r·cos(θ), u_σ·r·sin(θ), u_ω]ᵀ (Nonholonomic)
```cpp
// Default parameters (optimized for nonholonomic constraints)
amp::KinoPath path = KinoFactory::solveFirstOrderUnicycle(problem);

// Custom parameters
amp::KinoPath path = KinoFactory::solveFirstOrderUnicycle(
    problem,
    5000,    // max_iterations (higher for nonholonomic)
    0.1,     // dt
    0.1,     // goal_bias (lower for better exploration)
    5        // control_samples
);
```

### Second-Order Unicycle
**System**: ẋ = [v·cos(θ), v·sin(θ), ω, a, α]ᵀ (Dynamic system)
```cpp
// Default parameters (optimized for dynamic systems)
amp::KinoPath path = KinoFactory::solveSecondOrderUnicycle(problem);

// Custom parameters
amp::KinoPath path = KinoFactory::solveSecondOrderUnicycle(
    problem,
    6000,    // max_iterations
    0.08,    // dt (smaller for stability)
    0.08,    // goal_bias
    6        // control_samples
);
```

### Simple Car
**System**: ẋ = [v·cos(θ), v·sin(θ), (v/L)·tan(φ), u₁, u₂]ᵀ (Car kinematics)
```cpp
// Default parameters (optimized for steering constraints)
amp::KinoPath path = KinoFactory::solveSimpleCar(problem);

// Custom parameters  
amp::KinoPath path = KinoFactory::solveSimpleCar(
    problem,
    7000,    // max_iterations (highest for complex kinematics)
    0.1,     // dt
    0.05,    // goal_bias (lowest for maximum exploration)
    8        // control_samples (highest for steering variety)
);
```

## Advanced Configuration

### Manual Planner Configuration
```cpp
// Create planner with specific parameters
MyKinoRRT planner(10000, 0.05, 0.12, 6);

// Or configure at runtime
MyKinoRRT planner;
planner.setMaxIterations(8000);
planner.setTimeStep(0.08);
planner.setGoalBias(0.15);
planner.setControlSamples(7);

// Use with any agent
auto agent = std::make_shared<MySimpleCar>();
amp::KinoPath path = planner.plan(problem, *agent);
```

### Factory Pattern
```cpp
// Create configured planners
auto single_planner = KinoFactory::createSingleIntegratorPlanner(5000, 0.03, 0.2, 4);
auto car_planner = KinoFactory::createSimpleCarPlanner(8000, 0.08, 0.04, 12);

// Reuse with different problems
amp::KinoPath path1 = single_planner->plan(problem1, *single_agent);
amp::KinoPath path2 = car_planner->plan(problem2, *car_agent);
```

## Parameter Tuning

### Batch Testing
```cpp
// Define parameter sets to test: {max_iter, dt, goal_bias, control_samples}
std::vector<std::tuple<int, double, double, int>> param_sets = {
    {3000, 0.1, 0.05, 5},     // Conservative
    {5000, 0.08, 0.08, 6},    // Balanced
    {7000, 0.1, 0.05, 8},     // High exploration  
    {10000, 0.05, 0.12, 10}   // High resolution
};

// Test all combinations
auto results = KinoFactory::tuneParameters(problem, param_sets);

// Analyze results
for (const auto& result : results) {
    std::cout << "Agent: " << result.agent_type_name 
              << ", Time: " << result.execution_time_ms << "ms"
              << ", Success: " << result.found_solution << std::endl;
}
```

## Parameter Guidelines

### Parameter Explanations

| Parameter | Description | Typical Range | Agent-Specific Notes |
|-----------|-------------|---------------|---------------------|
| `max_iterations` | Maximum RRT iterations | 1000-15000 | Higher for complex agents (Car > Unicycle > Single) |
| `dt` | Integration time step | 0.01-0.2 | Smaller for dynamic systems, larger for stability |
| `goal_bias` | Probability of sampling goal | 0.01-0.3 | Lower for complex agents needing more exploration |
| `control_samples` | Random controls per iteration | 1-15 | Higher for agents with more control constraints |

### Tuning Strategy

1. **Start with defaults**: Use agent-specific factory functions
2. **Increase iterations**: If no solution found, increase `max_iterations`
3. **Adjust time step**: Decrease `dt` for accuracy, increase for speed
4. **Tune exploration**: Lower `goal_bias` for complex environments
5. **More control variety**: Increase `control_samples` for constrained agents

### Performance vs. Quality Trade-offs

**High Performance (Fast planning)**:
```cpp
// Minimal parameters for quick solutions
KinoFactory::solveSimpleCar(problem, 2000, 0.15, 0.1, 3);
```

**High Quality (Accurate solutions)**:
```cpp
// Maximum parameters for best paths
KinoFactory::solveSimpleCar(problem, 15000, 0.03, 0.02, 15);
```

## Integration Examples

### Complete Workflow
```cpp
#include "KinoFactory.h"

int main() {
    // Load problems
    std::vector<amp::KinodynamicProblem2D> problems = {
        HW9::getStateIntProblemWS1(),   // Single Integrator
        HW9::getFOUniProblemWS1(),      // First-Order Unicycle
        HW9::getSOUniProblemWS1(),      // Second-Order Unicycle
        HW9::getCarProblemWS1()         // Simple Car
    };
    
    // Solve all problems automatically
    for (const auto& problem : problems) {
        std::cout << "Solving problem for agent type: " << (int)problem.agent_type << std::endl;
        
        auto start = std::chrono::high_resolution_clock::now();
        amp::KinoPath path = KinoFactory::solveKinodynamicProblem(problem);
        auto end = std::chrono::high_resolution_clock::now();
        
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        
        std::cout << "  Result: " << (path.valid ? "SUCCESS" : "FAILED") << std::endl;
        std::cout << "  Time: " << duration.count() << "ms" << std::endl;
        std::cout << "  Waypoints: " << path.waypoints.size() << std::endl;
        
        if (path.valid) {
            HW9::check(path, problem);
            Visualizer::makeFigure(problem, path, false);
        }
    }
    
    Visualizer::saveFigures();
    return 0;
}
```

### Problem-Specific Optimization
```cpp
// Optimize parameters for specific problem characteristics
amp::KinoPath optimizedSolve(const amp::KinodynamicProblem2D& problem) {
    // Analyze problem complexity
    int num_obstacles = problem.obstacles.size();
    double workspace_area = (problem.x_max - problem.x_min) * (problem.y_max - problem.y_min);
    
    // Adapt parameters based on complexity
    int iterations = 5000 + num_obstacles * 500;  // More iterations for more obstacles
    double dt = (workspace_area > 100) ? 0.1 : 0.05;  // Smaller dt for large workspaces
    double bias = std::max(0.02, 0.15 - num_obstacles * 0.02);  // Less bias for cluttered environments
    int samples = 5 + problem.agent_type == AgentType::SimpleCar ? 3 : 0;  // More samples for car
    
    return KinoFactory::solveKinodynamicProblem(problem);
}
```

## Troubleshooting

### Common Issues and Solutions

1. **No solution found**: Increase `max_iterations` and decrease `goal_bias`
2. **Slow planning**: Increase `dt` and reduce `control_samples`
3. **Poor path quality**: Decrease `dt` and increase `control_samples`
4. **Frequent collisions**: Check collision detection, reduce `dt`
5. **Agent-specific failures**: Use agent-specific factory functions with default parameters

### Debug Information
```cpp
// Enable detailed output
MyKinoRRT planner;
std::cout << "Using parameters:" << std::endl;
std::cout << "  Max iterations: " << planner.getMaxIterations() << std::endl;
std::cout << "  Time step: " << planner.getTimeStep() << std::endl;
std::cout << "  Goal bias: " << planner.getGoalBias() << std::endl;
std::cout << "  Control samples: " << planner.getControlSamples() << std::endl;
```

This factory system provides a robust, user-friendly interface for kinodynamic motion planning while maintaining the flexibility for advanced users to fine-tune parameters for specific applications.