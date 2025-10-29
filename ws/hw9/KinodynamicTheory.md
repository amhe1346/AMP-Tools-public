// ============================================================================
// KINODYNAMIC SYSTEMS THEORY - HW9 REFERENCE
// ============================================================================
//
// MATHEMATICAL FRAMEWORK: ẋ = f(x, u)
// • x ∈ X: state vector, X: state space
// • u ∈ U: control input, U: control space  
// • f(·,·): continuous integrable function defining system dynamics
//
// ============================================================================
// CLASSIFICATION TABLE
// ============================================================================
//
// │ System Type │ State Space X │ Control Space U │ Constraints │
// │─────────────┼───────────────┼─────────────────┼─────────────│
// │ First-Order │ X = C         │ U = velocities  │ Kinematic   │
// │             │ (config only) │                 │             │
// │─────────────┼───────────────┼─────────────────┼─────────────│
// │ Second-Order│ X ⊃ C + Ċ     │ U = accelerations│ Dynamic     │
// │             │ (+ derivatives)│                 │             │
// └─────────────┴───────────────┴─────────────────┴─────────────┘
//
// ============================================================================
// HW9 IMPLEMENTATIONS MAPPED TO THEORY
// ============================================================================
//
// 1. SINGLE INTEGRATOR (First-Order, Holonomic)
//    ┌─────────────────────────────────────────────────────────────┐
//    │ ẋ = f(x, u) = u                                             │
//    │ X = ℝ² (position only)                                     │  
//    │ U = ℝ² (velocity control)                                  │
//    │ f(x, u) = [u₁, u₂]ᵀ                                        │
//    └─────────────────────────────────────────────────────────────┘
//
// 2. FIRST-ORDER UNICYCLE (First-Order, Nonholonomic)  
//    ┌─────────────────────────────────────────────────────────────┐
//    │ ẋ = f(x, u)                                                 │
//    │ X = ℝ² × S¹ (position + orientation)                       │
//    │ U = ℝ² (pedaling speed + rotational velocity)              │
//    │ f(x, u) = [u_σ·r·cos(θ), u_σ·r·sin(θ), u_ω]ᵀ              │
//    │ Constraint: cannot move sideways                            │
//    └─────────────────────────────────────────────────────────────┘
//
// 3. SECOND-ORDER UNICYCLE (Second-Order, Nonholonomic)
//    ┌─────────────────────────────────────────────────────────────┐
//    │ ẋ = f(x, u)                                                 │  
//    │ X = ℝ² × S¹ × ℝ² (position + orientation + velocities)     │
//    │ U = ℝ² (linear + angular acceleration)                     │
//    │ f(x, u) = [v·cos(θ), v·sin(θ), ω, a, α]ᵀ                  │
//    │ Constraint: acceleration → velocity → position             │
//    └─────────────────────────────────────────────────────────────┘
//
// 4. SECOND-ORDER CAR (Second-Order, Nonholonomic)
//    ┌─────────────────────────────────────────────────────────────┐
//    │ ẋ = f(x, u)                                                 │
//    │ X = ℝ² × S¹ × ℝ × [-π/2,π/2] (pos + orient + speed + steer)│
//    │ U = ℝ² (acceleration + steering rate)                      │  
//    │ f(x, u) = [v·cos(θ), v·sin(θ), (v/L)·tan(φ), u₁, u₂]ᵀ    │
//    │ Constraints: bicycle kinematics + acceleration dynamics    │
//    └─────────────────────────────────────────────────────────────┘
//
// ============================================================================
// KINODYNAMIC PLANNING IMPLICATIONS
// ============================================================================
//
// GEOMETRIC PLANNING (Traditional):
// • Plans in configuration space C  
// • Assumes robot can follow any path
// • No consideration of dynamics
//
// KINODYNAMIC PLANNING (HW9):
// • Plans in state space X (which includes C)
// • Must respect differential constraints ẋ = f(x, u)
// • Controls u must be feasible
// • Resulting trajectories must be physically achievable
//
// RRT MODIFICATIONS NEEDED:
// • Sample in state space X, not just configuration space C
// • Extend tree using feasible control inputs u ∈ U
// • Forward integrate ẋ = f(x, u) to get reachable states
// • Distance metrics must account for full state (position + velocities)
// • Goal regions defined in state space X
//
// ============================================================================