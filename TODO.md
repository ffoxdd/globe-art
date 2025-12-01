- Rename DensityVoronoiSphereOptimizer to something mentioning CCVD
- Make GradientDensityOptimizer mention that it's for VoronoiSpheres
- Make scalar fields act on Euclidean vectors

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
