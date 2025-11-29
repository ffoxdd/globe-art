# Gradient-Based Density Optimizer Design

This document captures the design for a new global gradient-based optimizer for capacity-constrained Voronoi tessellation on S².

## Problem Statement

We want equal-mass Voronoi cells on S²:
- Continuous density function ρ(x) on S²
- N generator sites sᵢ ∈ S²
- Each site defines an unweighted spherical Voronoi cell Vᵢ(s)
- Target: Mᵢ ≈ m for all i, where m = (1/N)∫_{S²} ρ(x) dΩ

**Key constraint**: We use unweighted Voronoi (not power diagrams) for aesthetics.

## Energy Function

```
E(s) = ½ Σᵢ (Mᵢ(s) - m)²
```

where Mᵢ(s) = ∫_{Vᵢ(s)} ρ(x) dΩ

## Gradient Computation

The gradient simplifies to a sum over edges:

```
∂E/∂sₖ = Σⱼ∈neighbors(k) (mass_error[k] - mass_error[j]) · edge_grad(k,j)
```

where:
- `mass_error[i] = Mᵢ - m`
- `edge_grad(k,j)` = contribution from edge between cells k and j

The edge gradient decomposes into:
- **Field part**: `∫_edge ρ dl` (density integrated along edge)
- **Geometry part**: `edge_sensitivity` (how edge moves when site moves)

```
edge_grad = edge_integral · edge_sensitivity
```

## Moment-Based Integration

For polynomial fields ρ(x) = c + a·x + x^T Q x, mass can be computed analytically:

```
mass = c·area + a·first_moment + tr(Q·second_moment)
```

where:
- `area` = ∫_V 1 dΩ
- `first_moment` = ∫_V x dΩ (Vector3)
- `second_moment` = ∫_V xx^T dΩ (Matrix3)

Same pattern for edge integrals with ArcMoments (length, first_moment, second_moment along arc).

## Data Structures

### PolygonMoments
```cpp
struct PolygonMoments {
    double area;            // ∫_cell 1 dΩ
    Vector3 first_moment;   // ∫_cell x dΩ
    Matrix3 second_moment;  // ∫_cell xx^T dΩ
};
```

### ArcMoments
```cpp
struct ArcMoments {
    double length;          // ∫_edge 1 dl
    Vector3 first_moment;   // ∫_edge x dl
    Matrix3 second_moment;  // ∫_edge xx^T dl
};
```

## SphericalField Concept

```cpp
concept SphericalField {
    double value(const Point3& x) const;
    double mass(const PolygonMoments& moments) const;
    double edge_integral(const ArcMoments& moments) const;
    Vector3d edge_gradient_integral(const ArcMoments& moments) const;  // ∫ρ(x) x dl
};
```

### Implementations

| Field | ρ(x) | mass() | edge_integral() | edge_gradient_integral() |
|-------|------|--------|-----------------|-------------------------|
| Constant | c | c×area | c×length | c×first_moment |
| Linear | a·z+b | a×M_z+b×area | a×M_z+b×length | a×second_moment.col(2)+b×first_moment |
| Quadratic | x^TQx | tr(Q×second) | tr(Q×second) | (needs third moment) |
| Polynomial | c+a·x+x^TQx | all three | all three | (combined) |

## Edge Sensitivity (Exact Derivation)

When site sₖ moves by displacement d, edges between cell k and neighbors move. The key insight is that only the **normal velocity** matters for mass change, and we can derive it exactly.

### Bisector Geometry

The edge between cells k and j lies on the bisector plane: all points x equidistant from sₖ and sⱼ. For a point x on the edge:

```
|x - sₖ|² = |x - sⱼ|²
```

When sₖ moves by d, the new constraint becomes:

```
|x' - (sₖ + d)|² = |x' - sⱼ|²
```

Expanding and taking the first-order perturbation, the edge moves with velocity:

```
v(x) = [(x · d) / |sⱼ - sₖ|²] (sⱼ - sₖ)
```

### Normal Velocity

The outward normal to the edge (pointing from cell k toward cell j) is:

```
n̂ = (sⱼ - sₖ) / |sⱼ - sₖ|
```

The normal velocity (what determines mass change) is:

```
v_n(x) = v(x) · n̂ = (x · d) / |sⱼ - sₖ|
```

### Mass Gradient

The mass change when the edge moves is:

```
δMₖ = ∫_edge ρ(x) v_n(x) dl
    = ∫_edge ρ(x) (x · d) / |n| dl
    = (1/|n|) [∫_edge ρ(x) x dl] · d
```

where |n| = |sⱼ - sₖ|.

Therefore:

```
∂Mₖ/∂sₖ = (1/|n|) ∫_edge ρ(x) x dl
```

This is the **ρ-weighted first moment** divided by the inter-site distance.

### Implementation

The `SphericalField` concept provides `edge_gradient_integral(arc_moments)` which computes ∫ρ(x) x dl:

| Field | edge_gradient_integral(moments) |
|-------|--------------------------------|
| Constant ρ = c | c × first_moment |
| Linear ρ = a·z + b | a × second_moment.col(2) + b × first_moment |

The gradient computation becomes:
```cpp
Eigen::Vector3d rho_weighted_moment = field.edge_gradient_integral(arc_moments);
Eigen::Vector3d n_vec = site_j - site_k;
Eigen::Vector3d edge_grad = rho_weighted_moment / n_vec.norm();
```

## Optimizer Flow

```cpp
template<SphericalField Field>
void optimize_sites(const Field& field, std::vector<Point3>& sites, int iterations) {
    double target_mass = field.mass(full_sphere_moments) / sites.size();

    for (int iter = 0; iter < iterations; ++iter) {
        // 1. Build Voronoi
        auto voronoi = build_voronoi(sites);

        // 2. Compute mass errors
        std::vector<double> mass_error(n);
        for (int i = 0; i < n; ++i) {
            auto moments = compute_polygon_moments(voronoi.cell(i));
            mass_error[i] = field.mass(moments) - target_mass;
        }

        // 3. Compute energy gradient per site
        std::vector<Vector3> gradient(n, Vector3::Zero());
        for (int k = 0; k < n; ++k) {
            for (const Edge& edge : voronoi.cell(k).edges()) {
                int j = edge.neighbor_index();

                auto arc_moments = compute_arc_moments(edge.v1(), edge.v2());
                double rho_integral = field.edge_integral(arc_moments);
                Vector3 sensitivity = compute_edge_sensitivity(sites[k], sites[j], edge.v1(), edge.v2());
                Vector3 edge_grad = rho_integral * sensitivity;

                gradient[k] += (mass_error[k] - mass_error[j]) * edge_grad;
            }
        }

        // 4. Step (project gradient to tangent plane, then step)
        for (int k = 0; k < n; ++k) {
            Vector3 g = gradient[k];
            g -= dot(g, sites[k]) * sites[k];  // tangent projection

            sites[k] -= step_size * g;
            sites[k] = normalize(sites[k]);     // back to S²
        }
    }
}
```

## Approximations Made

The current implementation uses exact edge sensitivity (see "Edge Sensitivity (Exact Derivation)" section above). No approximations are made in the gradient computation for polynomial density fields.

## File Organization (Implemented)

```
src/globe/geometry/spherical/moments/
    arc_moments.hpp              # ArcMoments struct + compute_arc_moments()
    arc_moments_test.cpp
    polygon_moments.hpp          # PolygonMoments struct + compute_polygon_moments()
    polygon_moments_test.cpp

src/globe/fields/spherical/
    spherical_field.hpp          # SphericalField concept
    constant_spherical_field.hpp # ConstantSphericalField implementation
    constant_spherical_field_test.cpp

src/globe/geometry/spherical/
    edge_sensitivity.hpp         # compute_edge_sensitivity()
    edge_sensitivity_test.cpp

src/globe/voronoi/core/
    voronoi_sphere.hpp           # Added CellEdgeInfo struct and cell_edges() method

src/globe/voronoi/optimizers/
    gradient_density_optimizer.hpp
    gradient_density_optimizer_test.cpp
```

## Implementation Status

All components implemented and tested:
1. ✅ ArcMoments + compute_arc_moments() - using Eigen for Matrix3
2. ✅ PolygonMoments + compute_polygon_moments() - triangulation-based
3. ✅ SphericalField concept + ConstantSphericalField + LinearSphericalField
4. ✅ compute_edge_sensitivity()
5. ✅ GradientDensityOptimizer with backtracking line search

## Current Optimizer Implementation

The optimizer uses gradient descent with backtracking line search:

```cpp
// Parameters
INITIAL_STEP_SIZE = 0.1   // Starting step size
MAX_STEP_SIZE = 1.0       // Maximum step size
MIN_STEP_SIZE = 1e-10     // Minimum before giving up
STEP_SHRINK_FACTOR = 0.5  // Halve step on failure
STEP_GROW_FACTOR = 1.2    // Grow step on success
```

**Algorithm:**
1. Compute projected gradient (tangent to sphere)
2. Backtracking line search: try step, accept if error decreases, else halve and retry
3. On success, grow step size by 1.2x (up to max 1.0)
4. Stall detection: stop after 10 consecutive low-improvement iterations

**Results:**
- **Constant field**: Converges to RMS error ~1e-8 in ~25 iterations (very fast)
- **Linear field**: Stalls at RMS error ~0.001 (improved from ~0.009 with old approximation)

## Perturbation and Escape from Local Minima

The gradient optimizer includes perturbation logic to escape local minima (inspired by Balzer's capacity-constrained Voronoi paper):

1. **Perturbation**: When stuck (step size → 0 for 10+ iterations), perturb a cell with high mass error toward a random direction
2. **Checkpoint/restore**: Save best configuration; restore after 5 perturbations without improvement
3. **Progress tracking**: Limit to 50 perturbation attempts, 3 restores per checkpoint

**Parameters** (in gradient_density_optimizer.hpp):
```cpp
PERTURBATION_ANGLE = 0.05                  // Radians to move site
MAX_PERTURBATION_ATTEMPTS = 50             // Total perturbations before giving up
MAX_PERTURBATIONS_BEFORE_RESTORE = 5       // Perturbations before restoring to best
MAX_RESTORES_PER_CHECKPOINT = 3            // Max restores per best checkpoint
```

With perturbation, linear field convergence improved from ~0.001 to ~0.0016 RMS error.

## Future Extensions

- Full analytic velocity integration (non-constant v_n along edge)
- Higher-order polynomial fields
- Composite fields (linear combinations)
- Per-site adaptive step sizes based on cell geometry
