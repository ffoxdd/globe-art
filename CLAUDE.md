# Claude Notes

## Meta
- When the user says "take a note" or similar, update this file (CLAUDE.md)
- Only record higher-order ideas and principles
- Never include specific code details or implementation specifics
- Never include historically contingent information (e.g., "X was renamed to Y")
- When adding or modifying style rules in this file, also update `.cursorrules` to mirror the changes

## Testing and Development

### Expensive and Integration Tests
Tests that are statistical, slow, or integration-focused should use the `EXPENSIVE_` prefix with environment variable gating:
- Prefix expensive test names with `EXPENSIVE_`
- Use `REQUIRE_EXPENSIVE()` macro at the start of the test (defined in `src/globe/testing/geometric_assertions.hpp`)
- All tests are always compiled
- Skipped by default (normal runs skip them)
- Opt-in to run by setting environment variable: `EXPENSIVE=1`
- No recompilation needed

Pattern:
```cpp
TEST(ComponentTest, EXPENSIVE_StatisticalValidation) {
    REQUIRE_EXPENSIVE();

    // Test with 100k+ samples, multiple runs, etc.
}
```

Running tests:
```bash
# Normal run (skips expensive tests automatically)
ctest

# Run expensive tests too
EXPENSIVE=1 ctest
```

What makes a test expensive:
- Large sample counts (10k+ samples)
- Statistical validation requiring multiple runs
- Integration tests involving multiple subsystems
- Performance benchmarks

### Pre-Commit Checklist
Before making a commit, always verify:
1. The code builds successfully: `cmake --build .` (or `make`)
2. All tests pass, including expensive tests: `EXPENSIVE=1 ctest`

These checks ensure that expensive tests are not accidentally broken and remain executable.

### Build System Behavior
- CMakeLists.txt automatically picks up all `*_test.cpp` files using `GLOB_RECURSE`
- New test files are discovered at cmake configuration time, not build time
- After adding a new test file, run `cmake ..` from the build directory to reconfigure
- The build system will then automatically compile and register the new test

### Dependency Injection via Concepts
**ALL injected dependencies must use concepts** - not just random/slow ones. Any dependency injected via template parameter should be constrained by a concept:
- Use concepts to constrain ALL injected template parameters (e.g., `SpherePointGenerator`, `IntervalSampler`, `ScalarField`, `IntegrableField`)
- Concepts define the interface without specifying implementation
- Template parameters must use the concept as the constraint, never `typename`
- Provide concrete default template arguments for production use
- This enables dependency injection while maintaining type safety and clear interfaces
- Benefits: testability (inject test doubles), type safety (compiler enforces interface), documentation (concept shows requirements)

Pattern:
```cpp
template<
    ScalarField ScalarFieldType,
    SpherePointGenerator GeneratorType = RandomSpherePointGenerator<>,
    IntervalSampler IntervalSamplerType = UniformIntervalSampler
>
class MyClass {
 public:
    MyClass() = default;

    MyClass(
        ScalarFieldType field,
        GeneratorType generator,
        IntervalSamplerType sampler = IntervalSamplerType()
    );

 private:
    ScalarFieldType _field;
    GeneratorType _generator;
    IntervalSamplerType _sampler;
};
```

Key principles:
- **Concept as constraint**: Use the concept name (e.g., `IntervalSampler`) not `typename` for ALL injected template parameters
- **Concrete default**: Provide a concrete type as the default (e.g., `UniformIntervalSampler`)
- **Constructor injection**: Allow injecting dependencies via constructor with default-constructed fallback
- **Default constructor**: Provide a default constructor that uses default-constructed dependencies
- **Never create internally**: Never instantiate dependencies as uninjected private members - ALL dependencies must be injectable

### Unit Test Design Principles
Unit tests should be minimal, deterministic, and fast:
- Use 1-3 data points to create contrived situations that exhibit specific behaviors
- Use test doubles (mocks, sequences, seeded samplers) instead of real random dependencies
- Each test should verify a single behavior with a minimal scenario
- Avoid loops and large sample counts in unit tests
- Place pure unit tests at the top of test files

What belongs in unit tests:
- Edge cases with minimal data (empty inputs, single elements, boundary values)
- Specific behaviors with contrived scenarios (e.g., "filters outside points" with 1 inside + 1 outside)
- Error conditions and validation
- Deterministic calculations with known inputs/outputs

### Integration Test Design Principles
Integration tests should use real collaborators and realistic scenarios:
- Mark with `EXPENSIVE_` prefix and `REQUIRE_EXPENSIVE()` macro
- Use real random generators (optionally seeded for reproducibility)
- Use realistic sample counts (10k+ samples)
- Test convergence, statistical properties, and system-level behaviors
- Place integration tests at the bottom of test files

What belongs in integration tests:
- Statistical validation requiring many samples
- Convergence tests for numerical algorithms
- System-level behaviors involving multiple components working together
- Performance benchmarks
- End-to-end workflows with realistic data volumes

### Terminal Testing Flags
The `generate_globe` executable supports command-line flags for terminal/AI-assisted testing:
- `--points <N>`: Number of points to generate (default: 10)
- `--density-function <type>`: Density function type - 'constant' or 'noise' (default: noise)
- `--no-render`: Run without Qt rendering for terminal-only testing
- `--help`: Show usage information

Examples:
- Fast terminal test with constant density: `./generate_globe --no-render --density-function constant --points 10`
- Test with more points and noise: `./generate_globe --no-render --points 50`
- Visual debugging with fewer points: `./generate_globe --points 10`

## Code Style Preferences

### Comments
- Do not use verbose documentation comments
- Avoid TODOs in code comments
- Avoid inline explanatory comments
- Keep code clean and minimal - let it speak for itself
- This applies even when writing CGAL-style code

### Control Flow
- Avoid the "V antipattern" - large nested conditional blocks that create a V-shape indentation
- Prefer private helper functions with early returns for exceptional cases
- Keep main functions flat and linear when possible

### Function and Method Organization
- Always organize functions/methods with highest abstraction first, followed by lower abstraction
- In implementation files (.cpp), place main() or the highest-level entry point first
- Use forward declarations when necessary to maintain this ordering (never reorganize to avoid forward declarations)
- Public methods come before private methods in class definitions
- Within each section, order by abstraction level (high-level orchestration methods before low-level utility methods)

### Formatting Multi-line Expressions
- Continuation operators (like `<<`) go at the end of the line, not the beginning
- Opening elements (parenthesis, bracket, etc.) stay at end of line
- Arguments/content on new lines, indented once (not aligned with previous line)
- Closing elements (parenthesis, bracket, etc.) on their own line for nested structures
- Never align with elements from previous lines
- Separate consecutive multi-line blocks with a blank line before and after
- This applies to all multi-line expressions including chained stream operations (`<<`)

Examples:
```cpp
// Function call
double length = std::sqrt(
    candidate.x() * candidate.x() +
    candidate.y() * candidate.y() +
    candidate.z() * candidate.z()
);

Point3 normalized(
    candidate.x() / length,
    candidate.y() / length,
    candidate.z() / length
);

// Stream operations - simple case (no nested structure)
std::cout << "Usage: " << program_name << " [options]\n"
    << "Options:\n"
    << "  --points <N>  Number of points\n"
    << "  --help        Show help\n"
    << std::endl;

// Stream operations - with nested structure (opening/closing on own lines)
std::cout << "Centroid: (" <<
    info.centroid.x() << ", " << info.centroid.y() << ", " << info.centroid.z() <<
    ")" << std::endl;
```
