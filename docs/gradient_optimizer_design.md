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
};
```

### Implementations

| Field | ρ(x) | mass() uses | edge_integral() uses |
|-------|------|-------------|---------------------|
| Constant | c | area | length |
| Linear | a·x | first_moment | first_moment |
| Quadratic | x^TQx | second_moment | second_moment |
| Polynomial | c + a·x + x^TQx | all three | all three |

## Edge Sensitivity

The sensitivity vector S encodes how the edge between sites i and j moves when site i moves:

```
edge_movement = dot(S, δ)
```

where δ is the site displacement.

Direction is determined by (sⱼ - sᵢ) projected to tangent plane at sᵢ.
Magnitude is roughly 0.5 × edge_length.

```cpp
Vector3 compute_edge_sensitivity(
    const Point3& site_i,
    const Point3& site_j,
    const Point3& edge_v1,
    const Point3& edge_v2
) {
    Vector3 toward_j = site_j - site_i;
    Vector3 tangent = project_to_tangent_plane(toward_j, site_i);
    tangent = normalize(tangent);

    double edge_length = arc_length(edge_v1, edge_v2);

    return 0.5 * edge_length * tangent;
}
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

The formula `∫_edge ρ(x) v_n(x) dl ≈ (∫_edge ρ(x) dl) · v̄_n` assumes edge velocity is approximately constant along the edge.

This is accurate for:
- Small cells (curvature negligible)
- Edges where velocity doesn't vary much

The interface allows upgrading to full analytic velocity integration later by changing the internals of `edge_mass_gradient()`.

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
3. ✅ SphericalField concept + ConstantSphericalField
4. ✅ compute_edge_sensitivity()
5. ✅ GradientDensityOptimizer - converges in ~14 iterations for constant field

## Future Extensions

- L-BFGS instead of gradient descent
- Full analytic velocity integration (non-constant v_n)
- Higher-order polynomial fields
- Composite fields (linear combinations)
