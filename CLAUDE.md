# Claude Notes

## Meta
- When the user says "take a note" or similar, update this file (CLAUDE.md)
- Only record higher-order ideas and principles
- Never include specific code details or implementation specifics
- Never include historically contingent information (e.g., "X was renamed to Y")

## Code Style Preferences

### Comments
- Do not use verbose documentation comments
- Avoid TODOs in code comments
- Avoid inline explanatory comments
- Keep code clean and minimal - let it speak for itself
- This applies even when writing CGAL-style code

### Formatting Multi-line Expressions
- Opening parenthesis stays at end of line
- Arguments on new lines, indented once (not aligned with previous line)
- Closing parenthesis on its own line for symmetry
- Never align with elements from previous lines
- Separate consecutive multi-line expressions with a blank line

Example:
```cpp
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
```
