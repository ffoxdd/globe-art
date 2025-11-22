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
- Use `SKIP_IF_EXPENSIVE()` macro at the start of the test (defined in `src/globe/testing/geometric_assertions.hpp`)
- All tests are always compiled
- Skipped by default (normal runs skip them)
- Opt-in to run by setting environment variable: `RUN_EXPENSIVE_TESTS=1`
- No recompilation needed

Pattern:
```cpp
TEST(ComponentTest, EXPENSIVE_StatisticalValidation) {
    SKIP_IF_EXPENSIVE();

    // Test with 100k+ samples, multiple runs, etc.
}
```

Running tests:
```bash
# Normal run (skips expensive tests automatically)
ctest

# Run expensive tests too
RUN_EXPENSIVE_TESTS=1 ctest

# Run ONLY expensive tests
RUN_EXPENSIVE_TESTS=1 ctest -R "EXPENSIVE"
```

What makes a test expensive:
- Large sample counts (10k+ samples)
- Statistical validation requiring multiple runs
- Integration tests involving multiple subsystems
- Performance benchmarks

### Pre-Commit Checklist
Before making a commit, always verify:
1. The code builds successfully: `cmake --build .` (or `make`)
2. All tests pass, including expensive tests: `RUN_EXPENSIVE_TESTS=1 ctest`

These checks ensure that expensive tests are not accidentally broken and remain executable.

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
