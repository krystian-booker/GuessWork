# Repository Guidelines

## Project Structure & Module Organization
Core public headers live in `include/posest/`; keep new interfaces and value types there. Implementations are split by domain under `src/` (`core/`, `camera/`, `config/`, `runtime/`, `fusion/`, `teensy/`, `v4l2/`, `mock/`). Tests live in `test/` as focused GTest translation units such as `test_mock_pipeline.cpp`. Runtime configuration is stored in SQLite through `posest_config`.

Mirror the existing module layout when adding code: public API in `include/posest/...`, implementation in the matching `src/<area>/...`.

## Build, Test, and Development Commands
Install dependencies first:

```bash
conan install . --build=missing -s build_type=Release
cmake --preset conan-release
cmake --build --preset conan-release -j
ctest --preset conan-release --output-on-failure
```

Use `./build/Release/posest_tests --gtest_filter='Suite.Name'` for one test case.

## Coding Style & Naming Conventions
This project uses C++20 with warnings enabled in `CMakeLists.txt` (`-Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wsign-conversion`). Match the existing style:

- 4-space indentation, braces on the same line, and short explanatory comments only where needed.
- Types and test suites use `PascalCase`; functions use `camelCase`; namespaces stay lowercase.
- Private data members use a trailing underscore, for example `running_`.
- Keep Linux-specific code isolated to `src/v4l2/` and guarded in CMake.

## Testing Guidelines
Tests use GoogleTest. Add new coverage in `test/test_<feature>.cpp` and register the file in `CMakeLists.txt` if needed. Prefer small, deterministic tests around threading, timing, and configuration boundaries. Run `ctest --preset conan-release --output-on-failure` before opening a PR; use targeted `--gtest_filter` runs while iterating.

## Commit & Pull Request Guidelines
DO NOT COMMIT

## Configuration Notes
OpenCV features are intentionally trimmed in `conanfile.txt` for headless Linux builds. Before enabling new OpenCV options or external integrations, verify the Conan dependency graph. Camera device paths stored in SQLite should use stable `/dev/v4l/by-id/` or `/dev/v4l/by-path/` names when possible.
