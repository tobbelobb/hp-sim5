# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Commands
- Run all tests: `npm test`
- Run a single test file: `npx jest tests/path/to/file.test.js`
- Run a specific test: `npx jest -t "test description"`

## Code Style Guidelines
- **Architecture**: Follow Entity Component System (ECS) pattern
- **Classes**: Use ES6 classes with PascalCase naming (e.g., `Vector2`, `CableJointComponent`)
- **Methods/Variables**: Use camelCase naming (e.g., `addVectors`, `restLength`)
- **Documentation**: Use JSDoc-style comments for functions
- **Vector Math**: Use the built-in `Vector2` class for vector operations
- **Error Handling**: Use console.warn for warnings; return fallback values when operations fail
- **Immutability**: Prefer creating new instances rather than mutating existing ones
- **Testing**: Use Jest's describe/test structure with expect assertions
- **Dependencies**: Minimize external dependencies, use vanilla JS when possible

# Rules for Better Code Collaboration

## 1. Follow Instructions Precisely
- Read instructions carefully and follow them exactly as written.
- Prioritize explicit instructions over inferences from existing code or comments.
- Don't add functionality that wasn't requested.

## 2. Remove Unused Code
- If instructed that certain functionality is no longer needed, remove all related code.
- Don't leave unused variables, functions, or parameters in the codebase.

## 3. Trust User Parameters
- If a user provides parameters like `segment_length` or `capsule_height`, use them directly.
- Don't recalculate values that are already provided as parameters.
- Don't add "safety" calculations (like min/max) unless explicitly requested.
- Don't add default values anywhere in the code unless the user asks for it.

## 4. Avoid Unnecessary Calculations
- Don't calculate values that can be directly derived from user parameters.
- For example, if `segment_length` is provided, use `segment_length/2` directly instead of calculating half the distance between points.

## 5. Remove Obsolete Parameters
- If a parameter like `total_length` is no longer used, remove it from the class/function definition.
- Don't keep parameters "just in case" they might be useful later.

## 6. Clean Up Related Comments
- Remove comments that reference removed or changed functionality.
- Don't leave comments about concepts (like "number of segments") that are no longer relevant.

## 7. Focus on the Current Task
- Don't be influenced by the apparent intent of the original code.
- The user's current instructions supersede any implied purpose in the existing code.
