# Runge-Kutta 4th Order Integration Implementation

## Mathematical Foundation

The Runge-Kutta 4th order (RK4) method provides significantly higher accuracy than Forward Euler integration for solving differential equations of the form:

```
ẋ = f(x, u)
```

### Forward Euler (Previous Implementation)
- **Formula**: x(t+Δt) ≈ x(t) + Δt · f(x(t), u)
- **Accuracy**: O(Δt) - first-order accuracy
- **Simple**: Single function evaluation per step

### Runge-Kutta 4th Order (New Implementation) 
- **Formula**: x(Δt) ≈ x(0) + (Δt/6)(w₁ + 2w₂ + 2w₃ + w₄)
- **Accuracy**: O(Δt⁴) - fourth-order accuracy  
- **More Complex**: Four function evaluations per step

where:
- **w₁ = f(x(0), u)** - slope at beginning of interval
- **w₂ = f(x(0) + (Δt/2)w₁, u)** - slope at midpoint using w₁
- **w₃ = f(x(0) + (Δt/2)w₂, u)** - slope at midpoint using w₂  
- **w₄ = f(x(0) + Δt·w₃, u)** - slope at end using w₃

## Implementation Details

### Dynamic Function f(x,u) Computation

For each agent type, we implement the exact dynamics:

**Single Integrator**: f(x,u) = u
```cpp
return control;  // Direct velocity control
```

**First-Order Unicycle**: f(x,u) = [u_σ·r·cos(θ), u_σ·r·sin(θ), u_ω]ᵀ
```cpp
f[0] = control[0] * r * cos(state[2]);  // ẋ = u_σ * r * cos(θ)
f[1] = control[0] * r * sin(state[2]);  // ẏ = u_σ * r * sin(θ)  
f[2] = control[1];                      // θ̇ = u_ω
```

**Second-Order Unicycle**: f(x,u) = [v·cos(θ), v·sin(θ), ω, a, α]ᵀ
```cpp
f[0] = state[3] * cos(state[2]);  // ẋ = v * cos(θ)
f[1] = state[3] * sin(state[2]);  // ẏ = v * sin(θ)
f[2] = state[4];                  // θ̇ = ω
f[3] = control[0];                // v̇ = a
f[4] = control[1];                // ω̇ = α
```

**Simple Car**: f(x,u) = [v·cos(θ), v·sin(θ), (v/L)·tan(φ), u₁, u₂]ᵀ
```cpp
f[0] = state[3] * cos(state[2]);           // ẋ = v * cos(θ)
f[1] = state[3] * sin(state[2]);           // ẏ = v * sin(θ)
f[2] = (state[3] / L) * tan(state[4]);     // θ̇ = (v/L) * tan(φ)
f[3] = control[0];                         // v̇ = u₁
f[4] = control[1];                         // φ̇ = u₂
```

## Advantages of RK4 for Kinodynamic Planning

1. **Higher Accuracy**: O(Δt⁴) vs O(Δt) for Forward Euler
2. **Better Trajectory Quality**: More accurate state predictions  
3. **Numerical Stability**: Better handling of nonlinear dynamics
4. **Faithful Dynamics**: More accurate representation of robot motion

## Performance Considerations

- **Computational Cost**: 4x more function evaluations per step
- **Memory Usage**: Requires storage of intermediate states w₁, w₂, w₃, w₄
- **Fallback Strategy**: Graceful degradation to Forward Euler if RK4 fails

## Integration with RRT

The RK4 integration is used in the main RRT loop:

```cpp
// Sample random control u ∈ U
Eigen::VectorXd u_rand = sampleRandomControl();

// Integrate from x_near toward x_rand using RK4
Eigen::VectorXd x_new = integrateRK4(x_near, u_rand, dt, agent);

// Add to tree if valid
if (isValidState(x_new)) {
    tree.add(x_new, x_near, u_rand, dt);
}
```

This provides much more accurate kinodynamic motion planning compared to the simple Forward Euler approach previously used.