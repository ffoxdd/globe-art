# Capacity-Constrained Voronoi Tessellation (CCVT) Algorithm

This document describes the CCVT algorithm used for generating globe art with density-adaptive point distributions.

## Overview

The goal is to generate a Voronoi diagram on a sphere where each cell has equal **mass** (integral of density function), not equal spatial area. This allows points to cluster in high-density regions and spread out in low-density regions.

**Reference**: Balzer, M., Schlömer, T., & Deussen, O. (2009). "Capacity-constrained point distributions: A variant of Lloyd's method." *ACM Transactions on Graphics (TOG)*, 28(3), 86.

## The Problem

Given:
- A set S of n sites (points) on a sphere
- A capacity constraint C = {c₁, ..., cₙ} (target masses for each cell)
- A density function ρ(x) over the sphere
- Total mass: Σ cᵢ = ∫_Ω ρ(x) dx

Find:
- Site positions that create a Voronoi diagram where each cell's actual mass matches its target capacity

## Why Not Lloyd's Algorithm?

Standard Lloyd's algorithm (moving sites to centroids) minimizes spatial variance and creates **equal-area cells**. For non-uniform density:
- Equal area → unequal mass
- Sites don't cluster in dense regions
- Not suitable for capacity constraints

CCVT addresses this by directly optimizing for equal mass rather than equal area.

## The Objective Function

The algorithm minimizes the global error function:

```
D(V(S), C) = Σᵢ (|V(sᵢ)| - cᵢ)²
```

Where:
- `V(sᵢ)` is the Voronoi cell for site sᵢ
- `|V(sᵢ)|` is the mass of that cell: ∫_V(sᵢ) ρ(x) dx
- `cᵢ` is the target capacity (mass) for that cell
- The sum is over **all** sites

The global minimum D = 0 is achieved when all cells have exactly their target mass.

## Algorithm Structure

```
Input: Sites S = {s₁, ..., sₙ}
       Capacities C = {c₁, ..., cₙ}
       Density function ρ(x)

Compute initial Voronoi diagram V(S)

repeat:
    stable := true
    I := {1, ..., n}  // Indices of sites to optimize

    while I ≠ ∅:
        // Pick site with most negative mass error
        Choose sᵢ with i ∈ I where |V(sᵢ)| - cᵢ is minimal

        // Optimize this site to minimize TOTAL diagram error
        Minimize D(V(S), C) by adjusting location of sᵢ

        if location of sᵢ changed:
            stable := false

        Remove i from I

until stable = true
```

## Key Implementation Details

### 1. Sequential Optimization with Priority Queue

Sites are optimized one at a time, prioritized by mass error:
- **Undersized cells** (|V(sᵢ)| < cᵢ) are optimized first
- This is more efficient than random order
- Undersized sites move toward oversized neighbors
- Oversized sites naturally shrink as undersized sites expand into them

### 2. Global Error Minimization

**Critical**: When optimizing site sᵢ, the objective function computes error across **all** cells:

```cpp
double objective(Point3 new_position_for_si) {
    voronoi.update_site(i, new_position_for_si);

    double total_error = 0;
    for (size_t j = 0; j < n; j++) {
        double mass_j = integrate(voronoi.cell(j), density);
        total_error += pow(mass_j - target_mass, 2);
    }

    return total_error;
}
```

This is expensive but essential:
- Moving sᵢ affects all neighboring cells
- Triangle flips can propagate changes far away
- Optimizing only cell i would ignore effects on neighbors
- Could create degenerate configurations (tiny cells, cells with mass ≈ 0)

### 3. Computational Complexity

For each objective function evaluation:
- Update one site: O(log n) in CGAL's Delaunay triangulation
- Compute mass of all n cells: O(n) × (Monte Carlo integration cost)
- Total per evaluation: O(log n + n × M) where M is Monte Carlo samples

For a complete iteration:
- Process all n sites
- Each site requires multiple objective evaluations (optimizer-dependent)
- Typical: O(n² × M) per iteration

**Example from paper**: 100 sites, 245 iterations, 3.2 million function samples = 320 million cell mass computations

### 4. Partial Optimization

The paper notes:
> "It is not necessary that the minimization for each individual site is performed until it finds a local minimum. Rather, a few minimization steps are sufficient."

This suggests limiting the optimizer to a small number of iterations per site rather than running to convergence.

### 5. Penalty Functions

To prevent degenerate configurations, the paper adds penalty terms:

**Boundary Penalty P₁** (sites outside valid region):
```
P₁(sᵢ) = 0                    if sᵢ ∈ Ω
       = (||sᵢ - Ω|| × cᵢ)²   if sᵢ ∉ Ω
```

**Proximity Penalty P₂** (sites too close together):
```
P₂(sᵢ) = Σⱼ penalty(sᵢ, sⱼ, cᵢ, cⱼ)

where penalty = 0                if i = j or δ ≥ 1
              = (1 - δ)² × cᵢ×cⱼ  if δ < 1

δ = ||sᵢ - sⱼ|| / sqrt(cᵢ + cⱼ)
```

The proximity penalty P₂ prevents sites from clustering too tightly, which would create very small cells that are numerically unstable.

## Monte Carlo Integration

Since we're working with arbitrary density functions on a sphere, we use Monte Carlo integration to compute cell masses:

```
|V(sᵢ)| = ∫_V(sᵢ) ρ(x) dx ≈ (Area of V(sᵢ)) × (1/N) × Σ ρ(sample_k)
```

**Challenge**: Monte Carlo estimates are noisy
- Each objective function call gets a different random estimate
- Optimizer sees a noisy/stochastic objective function
- Can lead to suboptimal moves or slow convergence

**Solutions**:
- Use more samples (higher N) → slower but more accurate
- Use quasi-random sequences (Sobol, Halton) → lower variance
- Use stratified sampling → more uniform coverage
- Implement variance reduction techniques

## Convergence

The algorithm shows **logarithmic convergence** toward D = 0 in practice:
- Early iterations make large progress
- Later iterations make small refinements
- Typically converges to D ≈ 0 (all cells within tolerance of target mass)

**Local minima**: The paper claims they never observed convergence to a local minimum D > 0, though it's theoretically possible. If it occurs, the solution is to randomly perturb undersized sites and continue.

## Spherical Parameterization

For optimization on a sphere, we use stereographic projection to convert to 2D:

```cpp
// Map from tangent plane (u, v) to sphere
Point3 plane_to_sphere(double u, double v) {
    double r² = u² + v²
    double scale = 4.0 / (4.0 + r²)

    Point3 result = south + scale × (u×tangent_u + v×tangent_v - south×r²/4)

    return normalize(result)  // Project to unit sphere
}
```

This allows unconstrained optimization in (u, v) space while automatically maintaining the sphere constraint.


## Performance Expectations


Our implementation should expect:
- O(n²) per iteration
- Logarithmic convergence → ~O(log(1/ε)) iterations for tolerance ε
- Monte Carlo cost dominates for complex density functions

## Future Optimizations

1. **Caching**: Reuse Monte Carlo samples across objective calls
   - Reduces variance but introduces bias
   - Trade-off between speed and accuracy

2. **Adaptive sampling**: Use more samples for high-error cells
   - Focus computational effort where needed
   - Requires dynamic sample count per cell

3. **Parallel computation**: Compute cell masses in parallel
   - n independent integrals per objective call
   - Good parallelization opportunity

4. **Progressive refinement**: Start with few samples, increase over iterations
   - Early iterations: fast but noisy
   - Late iterations: slow but accurate
   - Matches convergence behavior (large moves early, refinement late)

## References

1. Balzer, M., Schlömer, T., & Deussen, O. (2009). Capacity-constrained point distributions: A variant of Lloyd's method. *ACM SIGGRAPH 2009*.

2. Lloyd, S. P. (1982). Least squares quantization in PCM. *IEEE Transactions on Information Theory*.

3. Du, Q., Faber, V., & Gunzburger, M. (1999). Centroidal Voronoi tessellations: Applications and algorithms. *SIAM Review*.
