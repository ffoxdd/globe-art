# TODO

## Code Quality & Refactoring

- Inject the random number stuff into mass_calculator and centroid_calculator to avoid reinitializing
- Make type aliases for the spherical types in mass_calculator_test, test_spherical_polygon, etc.
- Combine MassCalculator and CentroidCalculator
- Stop using the underscore prefix convention for private data members
- Use the term "density" instead of "noise" where appropriate

## Integration Performance Optimization (Future)

Currently using Monte Carlo integration for scalar field integration over spherical polygons. This is fast (~4ms per polygon) and accurate for one-time integration.

**Future opportunity**: If we implement iterative vertex optimization (moving vertices slightly and re-integrating), caching could provide significant speedups.

### Potential Approaches for Incremental Integration

1. **Differential Integration** (Recommended)
   - Track polygon boundary changes when vertices move
   - Compute: `new_integral = old_integral + integral(added_region) - integral(removed_region)`
   - Only integrate over small slivers where boundary moved
   - Requires tracking previous polygon state

2. **Incremental Grid Sample Cache**
   - Pre-compute Fibonacci sphere samples + densities (one-time)
   - Cache which samples are inside/outside each polygon
   - When vertex moves by δ, only retest samples within distance δ of edges
   - Samples far from boundary reuse previous classification
   - Combines grid sampling with incremental updates

3. **Hierarchical Spatial Cache**
   - Cache integral values at multiple resolutions (quadtree/octree)
   - Cells fully inside/outside rarely need recomputation
   - Only recompute cells intersecting moved boundary
   - Most of polygon unchanged, only boundary regions recomputed

4. **Spatial Hash Grid**
   - Pre-compute uniform grid cells covering sphere
   - Cache inside/outside status per cell per polygon
   - When vertex moves, update only potentially-flipped cells
   - Coarse grid for membership, fine Monte Carlo within affected cells

### Implementation Notes

- Current grid sampling approach (GridSampleIntegrableField) is 3.5x slower than Monte Carlo for one-time queries
- Problem: 2000+ point-in-polygon tests per query dominate runtime
- Solution: Incremental approach avoids retesting unchanged regions
- Expected speedup: 10-100x for small vertex movements

### When to Implement

Only worth implementing if we add iterative vertex optimization. Current one-time integration works well with Monte Carlo.
