- Rename DensityVoronoiSphereOptimizer to something mentioning CCVD
- Make GradientDensityOptimizer mention that it's for VoronoiSpheres
- Make scalar fields act on Euclidean vectors
- See if we can use Arc#interpolate instead of the free helper
- Consider making CircularInterval::hull non-commutative
- Consider not providing concrete default template arguments
- Make our own precondition macro


## Future Refactoring

### Duplicated Checkpoint/Restore Pattern
Both `GradientDensityOptimizer` and `SphericalFieldDensityOptimizer` implement identical checkpoint/restore logic:
- `gradient_density_optimizer.hpp:363-382`
- `spherical_field_density_optimizer.hpp:613-634`

Consider extracting to a shared utility or base class.

### Inconsistent Epsilon Usage
Different epsilon values used throughout for similar geometric checks:
- `GEOMETRIC_EPSILON` (1e-10) from types.hpp
- `1e-10` hardcoded in sampled_spherical_field.hpp, monte_carlo_spherical_field.hpp
- `1e-20` in CGAL squared distance comparisons
- `1e-5` in dlib optimization
- `1e-4` (ZERO_ERROR_TOLERANCE) in spherical_field_density_optimizer.hpp

Should standardize on `GEOMETRIC_EPSILON` for geometric comparisons.

### Duplicated Perturbation Logic
Nearly identical tangent-based perturbation code in both optimizers:
- `gradient_density_optimizer.hpp:418-448`
- `spherical_field_density_optimizer.hpp:584-610`

Same algorithm with slightly different type handling. Could be extracted to a shared utility.

## Future Features

### Spherical Harmonic Decomposition from Sampled Patterns
Approximate arbitrary patterns (e.g., images mapped to the sphere) with integrable polynomial fields:
- Sample the pattern on a spherical grid
- Compute spherical harmonic coefficients via numerical integration
- Use Boost.Math for Legendre polynomials and spherical harmonics
- Use Boost.Math quadrature for Gauss-Legendre weights
- Store coefficients in a HarmonicField-like class that can evaluate and integrate exactly
- Higher harmonic degree L gives more detail but more terms (L² coefficients)

