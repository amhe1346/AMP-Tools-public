#include "MyKinoRRT.h"
#include "../shared/MyCollisionChecker.h"
#include <cmath>
#include <random>
#include <algorithm>
#include <vector>

// ============================================================================
// KINODYNAMIC MOTION PLANNING - HW9 
// Robot motion described by differential equation: ẋ = f(x, u)
// ============================================================================
//
// MATHEMATICAL FRAMEWORK:
// • x ∈ X is the state, X is the state space  
// • f(·,·) is a continuous integrable function
// • u ∈ U is the control input, U is the control space
// • Robot motion subject to constraints defined by the differential equation
//
// MOTION GENERATION THEORY:
// Robot motions obtained by applying input controls and integrating equations:
//
//                  t
//    x(t) = x₀ + ∫ f(x(τ), u) dτ     ← Fundamental integration formula
//                  0
//
// Given:
// • Starting state x₀
// • Input control u  
// • Motion equation: ẋ = f(x, u)
//
// Computation methods:
// • CLOSED-FORM INTEGRATION: When analytical solution exists
// • NUMERICAL INTEGRATION: Forward Euler (our propagate() functions)
//   x_{k+1} = x_k + dt · f(x_k, u_k)
//
// CLASSIFICATION:
// • FIRST-ORDER: X = C (configuration space), u includes velocity → kinematic constraints
// • SECOND-ORDER: X includes C + derivatives, u includes acceleration → dynamic constraints
//
// ============================================================================
// IMPLEMENTED AGENT MODELS:
// ============================================================================
//
// 1. Single Integrator: ẋ = u (FIRST-ORDER)
//    State x = [x, y] ∈ ℝ², Control u = [vx, vy] (velocity)
//
// 2. First Order Unicycle: ẋ = f(x, u) (FIRST-ORDER, nonholonomic)  
//    State x = [x, y, θ] ∈ ℝ² × S¹, Control u = [u_σ, u_ω]
//    f(x,u) = [u_σ*r*cos(θ), u_σ*r*sin(θ), u_ω]ᵀ
//
// 3. Second Order Unicycle: ẋ = f(x, u) (SECOND-ORDER)
//    State x = [x, y, θ, v, ω] ∈ ℝ² × S¹ × ℝ², Control u = [a, α] (accelerations)
//    f(x,u) = [v*cos(θ), v*sin(θ), ω, a, α]ᵀ
//
// 4. Second Order Car: ẋ = f(x, u) (SECOND-ORDER, nonholonomic)
//    State x = [x, y, θ, v, φ] ∈ ℝ² × S¹ × ℝ × [-π/2,π/2], Control u = [u₁, u₂]
//    Control bounds: u₁ ∈ [-1, 1] (acceleration), u₂ ∈ [-π/2, π/2] (steering rate)
//    f(x,u) = [v*cos(θ), v*sin(θ), (v/L)*tan(φ), u₁, u₂]ᵀ
//
// ============================================================================

void MySingleIntegrator::propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt) {
    // FIRST-ORDER SYSTEM: ẋ = f(x, u) = u
    // State space: X = ℝ² (configuration space C = ℝ²)
    // Control space: U = ℝ² (velocity control)
    // Kinematic constraint: robot can move with any velocity instantaneously
    
    // MOTION GENERATION: Numerical integration of ẋ = f(x, u)
    //                     t                    dt
    // x(t) = x₀ + ∫ f(x(τ), u) dτ  ≈  x₀ + ∫ f(x₀, u) dτ = x₀ + dt·f(x₀, u)
    //                     0                    0
    //
    // Forward Euler integration: x_{k+1} = x_k + dt · f(x_k, u_k)
    // Since f(x, u) = u for single integrator:
    state += dt * control;  // x_{k+1} = x_k + dt * u_k
};

void MyFirstOrderUnicycle::propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt) {
    // FIRST-ORDER NONHOLONOMIC SYSTEM: ẋ = f(x, u)
    // State space: X = ℝ² × S¹ (position + orientation)  
    // Control space: U = ℝ² (pedaling speed + rotational velocity)
    // Kinematic constraint: unicycle cannot move sideways (nonholonomic)
    //
    // f(x, u) = [u_σ * r * cos(θ)]
    //           [u_σ * r * sin(θ)]  
    //           [u_ω            ]
    //
    // State: [x, y, θ] (position and orientation)
    // Control: [u_σ, u_ω] where u_σ is pedaling angular velocity, u_ω is rotational velocity
    
    const double r = 1.0; // wheel radius (assuming unit radius)
    
    double x = state[0];
    double y = state[1]; 
    double theta = state[2];
    
    double u_sigma = control[0]; // pedaling angular velocity
    double u_omega = control[1]; // rotational velocity
    
    // Apply unicycle dynamics using forward Euler integration
    state[0] = x + dt * u_sigma * r * cos(theta);  // ẋ = u_σ * r * cos(θ)
    state[1] = y + dt * u_sigma * r * sin(theta);  // ẏ = u_σ * r * sin(θ)
    state[2] = theta + dt * u_omega;               // θ̇ = u_ω
};

void MySecondOrderUnicycle::propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt) {
    // SECOND-ORDER SYSTEM: ẋ = f(x, u)  
    // State space: X = ℝ² × S¹ × ℝ² (position + orientation + velocities)
    // Control space: U = ℝ² (accelerations)  
    // Dynamic constraint: acceleration controls, velocity must be integrated
    //
    // f(x, u) = [v * cos(θ)]
    //           [v * sin(θ)]
    //           [ω        ]
    //           [a        ]  ← u[0] 
    //           [α        ]  ← u[1]
    //
    // State: [x, y, θ, v, ω] (position, orientation, linear velocity, angular velocity)
    // Control: [a, α] where a is linear acceleration, α is angular acceleration
    
    double x = state[0];
    double y = state[1];
    double theta = state[2];
    double v = state[3];      // linear velocity
    double omega = state[4];  // angular velocity
    
    double a = control[0];     // linear acceleration
    double alpha = control[1]; // angular acceleration
    
    // Apply second-order unicycle dynamics using forward Euler integration
    state[0] = x + dt * v * cos(theta);        // ẋ = v * cos(θ)
    state[1] = y + dt * v * sin(theta);        // ẏ = v * sin(θ)  
    state[2] = theta + dt * omega;             // θ̇ = ω
    state[3] = v + dt * a;                     // v̇ = a
    state[4] = omega + dt * alpha;             // ω̇ = α
};

void MySimpleCar::propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt) {
    // SECOND-ORDER NONHOLONOMIC SYSTEM: ẋ = f(x, u)
    // State space: X = ℝ² × S¹ × ℝ × [-π/2, π/2] (position + orientation + speed + steering)
    // Control space: U = ℝ² (acceleration + steering rate)
    // Dynamic + kinematic constraints: bicycle model with acceleration control
    //
    // f(x, u) = [v * cos(θ)    ]
    //           [v * sin(θ)    ]  
    //           [(v/L) * tan(φ)]  ← nonholonomic constraint
    //           [u₁            ]  ← u[0] (acceleration)
    //           [u₂            ]  ← u[1] (steering rate)
    //
    // State: x = [x, y, θ, v, φ] (position, orientation, linear speed, steering angle)
    // Control: u = [u1, u2] where u1 is linear acceleration, u2 is steering angle rate
    
    const double L = 2.0; // wheelbase length (distance between front and rear axles)
    
    double x = state[0];        // position x
    double y = state[1];        // position y
    double theta = state[2];    // orientation θ
    double v = state[3];        // linear speed v
    double phi = state[4];      // steering angle φ
    
    double u1 = control[0];     // linear acceleration
    double u2 = control[1];     // steering angle rate
    
    // MOTION GENERATION: Forward Euler numerical integration
    // x(t+dt) = x(t) + dt · f(x(t), u) where f(x,u) is the dynamics function
    state[0] = x + dt * v * cos(theta);           // ẋ = v * cos(θ)      | Position dynamics
    state[1] = y + dt * v * sin(theta);           // ẏ = v * sin(θ)      | 
    state[2] = theta + dt * (v / L) * tan(phi);   // θ̇ = (v/L) * tan(φ)  | Kinematic constraint
    state[3] = v + dt * u1;                       // v̇ = u1              | Control input u₁
    state[4] = phi + dt * u2;                     // φ̇ = u2              | Control input u₂
    
    // Clamp steering angle to avoid tan(π/2) = ∞ singularity
    // Use slightly less than ±π/2 to prevent numerical instability
    const double max_steering = M_PI / 2 - 1e-6; // Just under ±90 degrees
    if (state[4] > max_steering) state[4] = max_steering;
    if (state[4] < -max_steering) state[4] = -max_steering;
};

// ============================================================================
// RUNGE-KUTTA 4TH ORDER INTEGRATION
// ============================================================================
// Implements the RK4 method from the lecture for higher accuracy integration:
// x(Δt) ≈ x(0) + (Δt/6)(w₁ + 2w₂ + 2w₃ + w₄)
// where:
// w₁ = f(x(0), u)
// w₂ = f(x(0) + (Δt/2)w₁, u) 
// w₃ = f(x(0) + (Δt/2)w₂, u)
// w₄ = f(x(0) + Δt*w₃, u)
// ============================================================================

Eigen::VectorXd computeStateDerivative(const Eigen::VectorXd& state, const Eigen::VectorXd& control, 
                                      amp::DynamicAgent& agent) {
    // Compute f(x,u) = ẋ directly based on agent type
    // This avoids the issue of the propagate functions modifying state
    
    MySingleIntegrator* single_integrator = dynamic_cast<MySingleIntegrator*>(&agent);
    if (single_integrator) {
        // Single integrator: f(x,u) = u
        return control;
    }
    
    MyFirstOrderUnicycle* first_order_unicycle = dynamic_cast<MyFirstOrderUnicycle*>(&agent);
    if (first_order_unicycle) {
        // First-order unicycle: f(x,u) = [u_σ*r*cos(θ), u_σ*r*sin(θ), u_ω]
        const double r = 1.0; // wheel radius
        Eigen::VectorXd f(3);
        f[0] = control[0] * r * cos(state[2]);  // ẋ = u_σ * r * cos(θ)
        f[1] = control[0] * r * sin(state[2]);  // ẏ = u_σ * r * sin(θ)
        f[2] = control[1];                      // θ̇ = u_ω
        return f;
    }
    
    MySecondOrderUnicycle* second_order_unicycle = dynamic_cast<MySecondOrderUnicycle*>(&agent);
    if (second_order_unicycle) {
        // Second-order unicycle: f(x,u) = [v*cos(θ), v*sin(θ), ω, a, α]
        Eigen::VectorXd f(5);
        f[0] = state[3] * cos(state[2]);  // ẋ = v * cos(θ)
        f[1] = state[3] * sin(state[2]);  // ẏ = v * sin(θ)
        f[2] = state[4];                  // θ̇ = ω
        f[3] = control[0];                // v̇ = a
        f[4] = control[1];                // ω̇ = α
        return f;
    }
    
    MySimpleCar* simple_car = dynamic_cast<MySimpleCar*>(&agent);
    if (simple_car) {
        // Simple car: f(x,u) = [v*cos(θ), v*sin(θ), (v/L)*tan(φ), u₁, u₂]
        const double L = 2.0; // wheelbase length
        Eigen::VectorXd f(5);
        f[0] = state[3] * cos(state[2]);           // ẋ = v * cos(θ)
        f[1] = state[3] * sin(state[2]);           // ẏ = v * sin(θ)
        f[2] = (state[3] / L) * tan(state[4]);     // θ̇ = (v/L) * tan(φ)
        f[3] = control[0];                         // v̇ = u₁
        f[4] = control[1];                         // φ̇ = u₂
        return f;
    }
    
    // Fallback: use numerical differentiation with small dt
    Eigen::VectorXd state_copy = state;
    Eigen::VectorXd control_copy = control;
    Eigen::VectorXd original_state = state;
    agent.propagate(state_copy, control_copy, 1.0);
    return state_copy - original_state;
}

Eigen::VectorXd integrateRK4(const Eigen::VectorXd& x0, const Eigen::VectorXd& u, 
                            double dt, amp::DynamicAgent& agent) {
    // Fourth-order Runge-Kutta integration as specified in lecture
    // x(Δt) ≈ x(0) + (Δt/6)(w₁ + 2w₂ + 2w₃ + w₄)
    
    // w₁ = f(x(0), u)
    Eigen::VectorXd w1 = computeStateDerivative(x0, u, agent);
    
    // w₂ = f(x(0) + (Δt/2)w₁, u)
    Eigen::VectorXd x_temp2 = x0 + (dt/2.0) * w1;
    Eigen::VectorXd w2 = computeStateDerivative(x_temp2, u, agent);
    
    // w₃ = f(x(0) + (Δt/2)w₂, u)  
    Eigen::VectorXd x_temp3 = x0 + (dt/2.0) * w2;
    Eigen::VectorXd w3 = computeStateDerivative(x_temp3, u, agent);
    
    // w₄ = f(x(0) + Δt*w₃, u)
    Eigen::VectorXd x_temp4 = x0 + dt * w3;
    Eigen::VectorXd w4 = computeStateDerivative(x_temp4, u, agent);
    
    // Final RK4 integration step
    Eigen::VectorXd x_new = x0 + (dt/6.0) * (w1 + 2.0*w2 + 2.0*w3 + w4);
    
    return x_new;
}

// Improved integration that handles arbitrary durations without dropping time steps
Eigen::VectorXd integrateRK4_robust(const Eigen::VectorXd& x0, const Eigen::VectorXd& u, 
                                   double total_duration, double max_dt, amp::DynamicAgent& agent) {
    // Ensure we don't drop any time by using appropriate step size
    // If total_duration = 0.1005 and max_dt = 0.001, we should use dt such that
    // total_duration / dt is an integer, or handle the remainder properly
    
    if (total_duration <= 0.0) return x0;
    
    // Calculate number of steps and actual dt to avoid dropping time
    int num_steps = std::max(1, (int)std::ceil(total_duration / max_dt));
    double actual_dt = total_duration / num_steps;  // This ensures no remainder
    
    Eigen::VectorXd x_current = x0;
    
    // Integrate step by step with exact time alignment
    for (int step = 0; step < num_steps; ++step) {
        x_current = integrateRK4(x_current, u, actual_dt, agent);
    }
    
    return x_current;
}

amp::KinoPath MyKinoRRT::plan(const amp::KinodynamicProblem2D& problem, amp::DynamicAgent& agent) {
    // ============================================================================
    // KINODYNAMIC RRT PLANNING
    // ============================================================================
    // Unlike geometric path planning, kinodynamic planning must respect:
    // 1. Differential constraints: ẋ = f(x, u)
    // 2. State space: X (may include velocities, accelerations)  
    // 3. Control space: U (velocity, acceleration, or other inputs)
    // 4. Trajectory feasibility: path must be achievable by the dynamics
    //
    // CURRENT IMPLEMENTATION: Random walk (placeholder)
    // TODO: Implement proper kinodynamic RRT:
    //   - Sample random states in X (not just configuration space)
    //   - Apply feasible controls u ∈ U for finite time
    //   - Use agent.propagate() to compute reachable states
    //   - Build tree in state space X, not just C
    // ============================================================================
    
    amp::KinoPath path;
    Eigen::VectorXd state = problem.q_init;
    path.waypoints.push_back(state);
    
    // ============================================================================
    // KINODYNAMIC RRT IMPLEMENTATION
    // ============================================================================
    
    // Algorithm parameters (use configurable member variables)
    const int max_iterations = m_max_iterations;
    const double dt = m_dt;                    // Control duration (time step)
    const double goal_bias = m_goal_bias;      // Probability of sampling goal
    const double goal_tolerance = 0.5;        // Distance to consider goal reached
    const int control_samples = m_control_samples;  // Number of random controls to try per iteration
    
    // Initialize random number generator
    std::random_device rd;
    std::mt19937 gen(rd());
    
    // Control space bounds (following tricycle constraints)
    // u_v ∈ [-1, 1] (velocity control)
    // u_φ ∈ [-π/2, π/2] (steering control)
    std::uniform_real_distribution<double> velocity_control_dist(-1.0, 1.0);
    std::uniform_real_distribution<double> steering_control_dist(-M_PI/2, M_PI/2);
    std::uniform_real_distribution<double> general_control_dist(-2.0, 2.0);  // For other agents
    std::uniform_real_distribution<double> uniform_dist(0.0, 1.0);
    
    // Determine state space bounds for sampling
    double x_min = problem.x_min, x_max = problem.x_max;
    double y_min = problem.y_min, y_max = problem.y_max;
    std::uniform_real_distribution<double> x_dist(x_min, x_max);
    std::uniform_real_distribution<double> y_dist(y_min, y_max);
    std::uniform_real_distribution<double> angle_dist(-M_PI, M_PI);
    std::uniform_real_distribution<double> velocity_dist(-5.0, 5.0);
    
    // Tree structure: each node contains state, parent index, control, and duration
    struct TreeNode {
        Eigen::VectorXd state;
        int parent_idx;
        Eigen::VectorXd control;
        double duration;
        
        TreeNode(const Eigen::VectorXd& s, int p, const Eigen::VectorXd& u, double d) 
            : state(s), parent_idx(p), control(u), duration(d) {}
    };
    
    std::vector<TreeNode> tree;
    tree.emplace_back(problem.q_init, -1, Eigen::VectorXd::Zero(2), 0.0);
    
    // Helper function to generate random state in state space X
    auto sampleRandomState = [&]() -> Eigen::VectorXd {
        Eigen::VectorXd random_state(problem.q_init.size());
        random_state[0] = x_dist(gen);  // x position
        random_state[1] = y_dist(gen);  // y position
        
        if (random_state.size() >= 3) {
            random_state[2] = angle_dist(gen);  // orientation θ
        }
        if (random_state.size() >= 4) {
            random_state[3] = velocity_dist(gen);  // velocity or speed
        }
        if (random_state.size() >= 5) {
            random_state[4] = velocity_dist(gen);  // angular velocity or steering
        }
        return random_state;
    };
    
    // Helper function to compute distance in state space
    auto stateDistance = [](const Eigen::VectorXd& s1, const Eigen::VectorXd& s2) -> double {
        // Weighted distance considering position more heavily than velocities
        double pos_weight = 1.0, vel_weight = 0.1;
        double dist = 0.0;
        
        // Position distance
        for (int i = 0; i < std::min(2, (int)s1.size()); ++i) {
            dist += pos_weight * (s1[i] - s2[i]) * (s1[i] - s2[i]);
        }
        
        // Velocity/orientation distance  
        for (int i = 2; i < s1.size(); ++i) {
            dist += vel_weight * (s1[i] - s2[i]) * (s1[i] - s2[i]);
        }
        
        return sqrt(dist);
    };
    
    // Helper function to check if state is in goal region
    auto isInGoalRegion = [&](const Eigen::VectorXd& state) -> bool {
        // Check if position components are within goal bounds
        for (size_t i = 0; i < std::min(problem.q_goal.size(), (size_t)state.size()); ++i) {
            if (state[i] < problem.q_goal[i].first || state[i] > problem.q_goal[i].second) {
                return false;
            }
        }
        return true;
    };
    
    // Initialize collision checker for obstacle-aware planning
    amp::MyCollisionChecker collision_checker;
    
    // Helper function to check if state is collision-free (enhanced with obstacle checking)
    auto isValidState = [&](const Eigen::VectorXd& state) -> bool {
        // Check workspace bounds for position
        if (state.size() >= 2) {
            if (state[0] < x_min || state[0] > x_max || 
                state[1] < y_min || state[1] > y_max) {
                return false;
            }
            
            // Extract position for collision checking
            Eigen::Vector2d position(state[0], state[1]);
            
            // Check for collision with obstacles using advanced collision checker
            if (collision_checker.isInCollision(problem, position)) {
                return false;
            }
        }
        return true;
    };
    
    // Helper function to check if trajectory segment is collision-free
    auto isValidTrajectorySegment = [&](const Eigen::VectorXd& start_state, 
                                        const Eigen::VectorXd& end_state) -> bool {
        // Extract positions from states
        Eigen::Vector2d start_pos(start_state[0], start_state[1]);
        Eigen::Vector2d end_pos(end_state[0], end_state[1]);
        
        // Use advanced collision checker for path validation
        return amp::MyCollisionChecker::isValidPath(start_pos, end_pos, 
                                                   problem.obstacles,
                                                   problem.x_min, problem.x_max,
                                                   problem.y_min, problem.y_max);
    };
    
    // Main RRT loop
    for (int iter = 0; iter < max_iterations; ++iter) {
        // Step 1: Sample random state (with goal biasing)
        Eigen::VectorXd x_rand;
        if (uniform_dist(gen) < goal_bias && !problem.q_goal.empty()) {
            // Sample from goal region
            x_rand = Eigen::VectorXd(problem.q_init.size());
            for (size_t i = 0; i < std::min(problem.q_goal.size(), (size_t)x_rand.size()); ++i) {
                std::uniform_real_distribution<double> goal_dist(
                    problem.q_goal[i].first, problem.q_goal[i].second);
                x_rand[i] = goal_dist(gen);
            }
            // Set remaining dimensions to reasonable values
            for (int i = problem.q_goal.size(); i < x_rand.size(); ++i) {
                x_rand[i] = (i == 2) ? angle_dist(gen) : velocity_dist(gen);
            }
        } else {
            x_rand = sampleRandomState();
        }
        
        // Step 2: Find nearest node in tree
        int nearest_idx = 0;
        double min_dist = stateDistance(tree[0].state, x_rand);
        for (size_t i = 1; i < tree.size(); ++i) {
            double dist = stateDistance(tree[i].state, x_rand);
            if (dist < min_dist) {
                min_dist = dist;
                nearest_idx = i;
            }
        }
        
        // Step 3: Try multiple random controls to extend toward x_rand
        double best_progress = -1.0;
        Eigen::VectorXd best_new_state, best_control;
        double best_duration = dt;  // Track the duration used for best extension
        bool found_valid_extension = false;
        
        for (int ctrl_attempt = 0; ctrl_attempt < control_samples; ++ctrl_attempt) {
            // Generate random control u ∈ U respecting problem bounds
            Eigen::VectorXd u_rand(problem.u_bounds.size());
            
            // For all agents: use the bounds specified in the problem
            for (size_t u_dim = 0; u_dim < problem.u_bounds.size(); ++u_dim) {
                std::uniform_real_distribution<double> u_dist(
                    problem.u_bounds[u_dim].first, 
                    problem.u_bounds[u_dim].second
                );
                u_rand[u_dim] = u_dist(gen);
            }
            
            // Sample random duration from problem bounds to avoid fixed time step issues
            std::uniform_real_distribution<double> duration_dist(
                problem.dt_bounds.first, problem.dt_bounds.second
            );
            double random_duration = duration_dist(gen);
            
            // Forward simulate using robust RK4 integration 
            // This ensures no time steps are dropped (e.g., 0.1005 duration with 0.001 dt)
            Eigen::VectorXd x_new;
            
            try {
                x_new = integrateRK4_robust(tree[nearest_idx].state, u_rand, random_duration, dt, agent);
            } catch (...) {
                // Fallback to agent propagate if RK4 fails
                x_new = tree[nearest_idx].state;
                agent.propagate(x_new, u_rand, random_duration);
            }
            
            // Check if new state is valid (collision-free and within bounds)
            if (!isValidState(x_new)) {
                continue;
            }
            
            // Check if trajectory segment from nearest to new state is collision-free
            if (!isValidTrajectorySegment(tree[nearest_idx].state, x_new)) {
                continue;
            }
            
            // Check if this extension makes progress toward x_rand
            double progress = min_dist - stateDistance(x_new, x_rand);
            if (progress > best_progress) {
                best_progress = progress;
                best_new_state = x_new;
                best_control = u_rand;
                best_duration = random_duration;  // Store the actual duration used
                found_valid_extension = true;
            }
        }
        
        // Step 4: Add best extension to tree
        if (found_valid_extension) {
            tree.emplace_back(best_new_state, nearest_idx, best_control, best_duration);
            
            // Step 5: Check if goal is reached
            if (isInGoalRegion(best_new_state)) {
                // Reconstruct path from goal back to start
                amp::KinoPath path;
                std::vector<Eigen::VectorXd> waypoints;
                std::vector<Eigen::VectorXd> controls;
                std::vector<double> durations;
                
                int current_idx = tree.size() - 1;
                while (current_idx != -1) {
                    waypoints.push_back(tree[current_idx].state);
                    if (tree[current_idx].parent_idx != -1) {
                        controls.push_back(tree[current_idx].control);
                        durations.push_back(tree[current_idx].duration);
                    }
                    current_idx = tree[current_idx].parent_idx;
                }
                
                // Reverse to get path from start to goal
                std::reverse(waypoints.begin(), waypoints.end());
                std::reverse(controls.begin(), controls.end());
                std::reverse(durations.begin(), durations.end());
                
                path.waypoints = waypoints;
                path.controls = controls;
                path.durations = durations;
                path.valid = true;
                
                return path;
            }
        }
    }
    
    // Final goal connection attempt: try to connect closest node to goal
    // Find the node closest to the goal region
    int closest_to_goal_idx = 0;
    double min_goal_dist = std::numeric_limits<double>::max();
    
    // Sample a point in the goal region to connect to
    Eigen::VectorXd goal_target = Eigen::VectorXd(problem.q_init.size());
    for (size_t i = 0; i < std::min(problem.q_goal.size(), (size_t)goal_target.size()); ++i) {
        goal_target[i] = (problem.q_goal[i].first + problem.q_goal[i].second) / 2.0; // Goal center
    }
    // Set remaining dimensions to reasonable values
    for (int i = problem.q_goal.size(); i < goal_target.size(); ++i) {
        goal_target[i] = 0.0; // Neutral values for angle/velocity
    }
    
    for (size_t i = 0; i < tree.size(); ++i) {
        double dist = stateDistance(tree[i].state, goal_target);
        if (dist < min_goal_dist) {
            min_goal_dist = dist;
            closest_to_goal_idx = i;
        }
    }
    
    // Try aggressive goal connection with more control samples
    const int aggressive_control_samples = std::max(m_control_samples * 3, 15);
    double best_progress = -1.0;
    Eigen::VectorXd best_new_state, best_control;
    bool found_goal_connection = false;
    
    for (int ctrl_attempt = 0; ctrl_attempt < aggressive_control_samples; ++ctrl_attempt) {
        // Generate random control u ∈ U respecting problem bounds
        Eigen::VectorXd u_rand(problem.u_bounds.size());
        
        // For all agents: use the bounds specified in the problem
        for (size_t u_dim = 0; u_dim < problem.u_bounds.size(); ++u_dim) {
            std::uniform_real_distribution<double> u_dist(
                problem.u_bounds[u_dim].first, 
                problem.u_bounds[u_dim].second
            );
            u_rand[u_dim] = u_dist(gen);
        }
        
        // Sample random duration for goal connection
        std::uniform_real_distribution<double> duration_dist(
            problem.dt_bounds.first, problem.dt_bounds.second
        );
        double goal_duration = duration_dist(gen);
        
        try {
            Eigen::VectorXd x_new = integrateRK4_robust(tree[closest_to_goal_idx].state, u_rand, goal_duration, dt, agent);
            
            // Check bounds and collision
            if (isValidState(x_new)) {
                double progress = -stateDistance(x_new, goal_target);
                if (progress > best_progress) {
                    best_progress = progress;
                    best_new_state = x_new;
                    best_control = u_rand;
                    found_goal_connection = true;
                }
                
                // Check if this reaches the goal
                if (isInGoalRegion(x_new)) {
                    // Add final node and reconstruct path
                    tree.emplace_back(x_new, closest_to_goal_idx, u_rand, dt);
                    
                    amp::KinoPath path;
                    std::vector<Eigen::VectorXd> waypoints;
                    std::vector<Eigen::VectorXd> controls;
                    std::vector<double> durations;
                    
                    int current_idx = tree.size() - 1;
                    while (current_idx != -1) {
                        waypoints.push_back(tree[current_idx].state);
                        if (tree[current_idx].parent_idx != -1) {
                            controls.push_back(tree[current_idx].control);
                            durations.push_back(tree[current_idx].duration);
                        }
                        current_idx = tree[current_idx].parent_idx;
                    }
                    
                    std::reverse(waypoints.begin(), waypoints.end());
                    std::reverse(controls.begin(), controls.end());
                    std::reverse(durations.begin(), durations.end());
                    
                    path.waypoints = waypoints;
                    path.controls = controls;
                    path.durations = durations;
                    path.valid = true;
                    
                    return path;
                }
            }
        } catch (...) {
            continue;
        }
    }
    
    // If we found a better connection but didn't reach goal, add it anyway for next iterations
    if (found_goal_connection) {
        tree.emplace_back(best_new_state, closest_to_goal_idx, best_control, dt);
    }
    
    // No path found - return empty path
    path.valid = false;
    return path;
}
