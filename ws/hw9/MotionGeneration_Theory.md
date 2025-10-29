// ============================================================================
// MOTION GENERATION IN KINODYNAMIC SYSTEMS - EDUCATIONAL REFERENCE
// ============================================================================
//
// FUNDAMENTAL PRINCIPLE:
// Robot motions are obtained by applying input controls and integrating 
// the equations of motion over time.
//
//                           t
//    x(t) = x₀ + ∫ f(x(τ), u) dτ     ← Core integration formula
//                           0
//
// ============================================================================
// INTEGRATION METHODS
// ============================================================================
//
// CLOSED-FORM INTEGRATION (Analytical):
// ┌─────────────────────────────────────────────────────────────────────────┐
// │ Available when f(x, u) has known analytical solution                   │
// │ Example: Single integrator ẋ = u → x(t) = x₀ + u·t                     │
// │ Advantages: Exact, no numerical errors                                 │
// │ Disadvantages: Rarely available for complex systems                    │
// └─────────────────────────────────────────────────────────────────────────┘
//
// NUMERICAL INTEGRATION (Computational):
// ┌─────────────────────────────────────────────────────────────────────────┐
// │ Forward Euler Method (used in our propagate() functions):              │
// │   x_{k+1} = x_k + dt · f(x_k, u_k)                                     │
// │                                                                         │
// │ Process:                                                                │
// │ 1. Start with initial state x₀                                         │
// │ 2. Apply control u for time dt                                         │
// │ 3. Compute derivative ẋ = f(x, u)                                      │
// │ 4. Update state: x_new = x_old + dt · ẋ                                │
// │                                                                         │
// │ Advantages: General method, works for any f(x, u)                      │
// │ Disadvantages: Approximation, small time steps needed for accuracy     │
// └─────────────────────────────────────────────────────────────────────────┘
//
// ============================================================================
// IMPLEMENTATION IN HW9
// ============================================================================
//
// Each propagate() function implements numerical integration:
//
// 1. SINGLE INTEGRATOR:
//    ẋ = u → x(t+dt) = x(t) + dt·u
//    
// 2. FIRST-ORDER UNICYCLE:
//    [ẋ]   [u_σ·r·cos(θ)]       [x]       [u_σ·r·cos(θ)]
//    [ẏ] = [u_σ·r·sin(θ)]  →    [y] += dt·[u_σ·r·sin(θ)]
//    [θ̇]   [u_ω        ]       [θ]       [u_ω        ]
//
// 3. SECOND-ORDER UNICYCLE:
//    [ẋ]   [v·cos(θ)]           [x]       [v·cos(θ)]
//    [ẏ]   [v·sin(θ)]           [y]       [v·sin(θ)]
//    [θ̇] = [ω      ]      →    [θ] += dt·[ω      ]
//    [v̇]   [a      ]           [v]       [a      ]
//    [ω̇]   [α      ]           [ω]       [α      ]
//
// 4. SECOND-ORDER CAR:
//    [ẋ]   [v·cos(θ)    ]       [x]       [v·cos(θ)    ]
//    [ẏ]   [v·sin(θ)    ]       [y]       [v·sin(θ)    ]
//    [θ̇] = [(v/L)·tan(φ)]  →   [θ] += dt·[(v/L)·tan(φ)]
//    [v̇]   [u₁          ]       [v]       [u₁          ]
//    [φ̇]   [u₂          ]       [φ]       [u₂          ]
//
// ============================================================================
// CONTROL BOUNDS (Tricycle/Car Constraints)
// ============================================================================
//
// Control inputs must respect physical limitations:
// • u_v ∈ [-1, 1]      ← Velocity/acceleration bounds
// • u_φ ∈ [-π/2, π/2]  ← Steering angle bounds
//
// These bounds ensure:
// 1. Realistic motion (no infinite accelerations)
// 2. Physical constraints (steering limits)
// 3. Stable numerical integration
//
// ============================================================================
// KINODYNAMIC RRT INTEGRATION
// ============================================================================
//
// The RRT algorithm uses this integration process:
//
// 1. Sample random control u ∈ U (respecting bounds)
// 2. Apply control for time dt: u → agent.propagate(state, u, dt)
// 3. Get new reachable state through integration
// 4. Build tree of kinodynamically feasible states
// 5. Connect states with feasible control trajectories
//
// Result: Paths that respect robot dynamics ẋ = f(x, u)
//
// ============================================================================