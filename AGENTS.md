# AGENTS.md

Instructions for AI coding agents (Kilo Code, Claude Code, opencode, etc.) working in this
repository. See [README.md](README.md) for the full project pitch and key concepts — this file
only covers what an agent needs and a README skim wouldn't give you.

## Overview

trackinglib is an academic, header-only C++17 library implementing Kalman-filter-family object
tracking (EKF, IF; UKF stubbed) with a choice of full or UDU-factored covariance representations
and built-in ego-motion compensation. It's held to AUTOSAR C++14 safety constraints (see
Constraints below) despite being a research/education target, not a shipped product.

## Commands

```bash
# Configure + build + test (CI does this across g++/clang++ x C++17/20 x Debug/Release)
mkdir -p build && cd build
cmake .. -DCMAKE_CXX_STANDARD=17 -DCMAKE_BUILD_TYPE=Debug
cmake --build .
ctest --output-on-failure

# Self-contained-header check (every header must compile standalone; CI job "verify-self-contained-headers")
cmake .. -DBUILD_HEADER_TESTS=ON && cmake --build . --target header_tests && ctest -R "header_test_" --output-on-failure

# Coverage report -> build_cov/coverage/index.html
./coverage_report.sh   # from repo root

# Docs -> doxydoc/html/index.html
doxygen   # from repo root
```

Branches: `feat/<kebab-case-description>` (e.g. `feat/math-optimization-analysis-and-improvements`).

## General instructions

- For **symbol lookups** specifically, use `rg` (not `grep`) against `.repo.tags` first — only
  fall back to a full-tree `rg` across `include/` if you need call sites/usages, which the index
  doesn't track. Never read `.repo.tags` directly (it's large).
- **Scoped reads by default**: once you know which lines you need (from a grep/rg hit,
  `.repo.tags`, or a prior read), read that file with an offset/limit range rather than the whole
  file — this applies to the *first* read of a large header, not just re-reads. Read a file in
  full only when the task genuinely needs the whole thing (a full-class refactor, the
  self-contained-header check, a doxygen pass) — don't default to whole-file reads just because
  it's convenient. Once you've read a file in full, don't re-read it again unless it changed.
- **Context budget awareness**: large reads accumulate in conversation history and are not
  reclaimed later — nothing shrinks content once it's a few turns old, even if you no longer need
  it. When a task or plan is clearly complete and verified, say so explicitly and suggest starting
  a fresh session (or compacting) before moving to the next unrelated task, rather than continuing
  to build on an increasingly large conversation.
- **Plan location**: all plans created in planning mode shall be stored in the folder  plans/recent

## Architecture

Header-only C++ library, layered under `include/trackingLib/`:

```
base/     Foundation types & contracts. first_include.h must be the first include in every header.
env/      Environment models (EgoMotion) for moving-sensor compensation.
math/     Self-contained linear algebra (Matrix, SquareMatrix, TriangularMatrix, DiagonalMatrix,
          Vector, CovarianceMatrixFull/Factored) + analysis functions.
          math/linalg/conversions/ centralizes all cross-type conversions.
filter/   KalmanFilter (EKF), InformationFilter, UnscentedKalmanFilter (stub, unimplemented).
motion/   MotionModelCV / MotionModelCA, built via CRTP on generic::Predict / generic::Update,
          policy-based over the covariance representation (full vs. UDU-factored).
observation/  Observation models (position, velocity, range-bearing, range-bearing-doppler),
          built via CRTP on ExtendedObservationModel, feeding generic::Update.
```

Prediction flow (CV + EKF, representative of the other combinations): `MotionModelCV::predict()`
→ `generic::Predict::predict()` → `compensateEgoMotion()` → `applyProcessModel()` →
`computeA/G/Q()` → `KalmanFilter::predictCovariance()`. Factored covariance updates go through
`CovarianceMatrixFactored::thornton()` (Thornton's algorithm, keeps the UDU factorization intact).

Measurement-update flow (mirrors prediction): `MotionModelCV::update()` → `generic::Update::run()`
→ observation model's `predictMeasurement()`/`computeJacobian()` → `KalmanFilter::updateState()` /
`InformationFilter::updateState()`, selectable at compile time via `filter/update_mode.h`
(`Block` vs. `Sequential`, defaulted per covariance policy) and composable across multiple
observation models in one call. Sequential (scalar/rank-1) updates support a correlated R by
decorrelating first via UDU (`filter/measurement_decorrelation.hpp`); an already-diagonal R takes
a no-transform fast path.

In-flight design docs for larger features live in `plans/recent/` (moved to `plans/archive/`
once done); this directory is gitignored, so treat it as local working notes, not source of truth.

Note: file-level maps of `include/trackingLib/` go stale fast in this codebase (verified: the old
memory-bank's file listing was already missing 12+ headers that exist today) — use `.repo.tags`
or `find`/`grep` for the current file set rather than trusting a hardcoded list, including the
folder summary above.

## Dependencies & environment

- **GoogleTest v1.16.x** and **tl::expected v1.0.0** ([TartanLlama/expected](https://github.com/TartanLlama/expected))
  are fetched automatically via CMake `FetchContent` — not vendored, no manual install step.
- CMake install exports the `trackingLib::` namespace (`NAMESPACE trackingLib::` in `CMakeLists.txt`)
  for `find_package()` consumers.
- **Primary dev environment**: `.devcontainer/devcontainer.json` (VS Code Dev Containers), image
  `trackinglib:latest`. Its `postCreateCommand` is what (re)generates the `.repo.tags` index via
  `universal-ctags` — if `.repo.tags` looks stale or missing, that's the command to rerun, or
  rebuild the devcontainer. Also builds/runs manually via `Dockerfile` (Linux) / `DockerfileMac`
  (macOS) / `dev-env.sh`, all at repo root.

## Code style & conventions

- **Error handling**: `tl::expected<T, Errors>` (Rust-style `Result`) everywhere in core code —
  no exceptions.
- **Conversions**: centralized in `math/linalg/conversions/` using a `<target>From<source>`
  naming convention (e.g. `DiagonalFromSquare`, `VectorFromMatrixColumnView`). Don't add ad-hoc
  conversion logic elsewhere.
- **Stream output**: template `operator<<` per matrix type in `matrix_io.h` — no `print()` methods.
- **Parameter order**: output/in-out parameters first, then inputs (e.g. `computeA(A, dt)`,
  `KalmanFilter::updateState(x, P, innovation, H, R)`). Applies to free/static functions with
  explicit output params; doesn't apply to ordinary member functions (the receiver `*this` is
  already the implicit output) or constructors. If a variadic parameter pack is also present, it
  must still trail every other parameter (a C++ requirement), which output-first already satisfies.
- **Formatting**: clang-format, Microsoft base style, 130 col limit, pointer-left, no bin-packing,
  `first_include.h` sorted first (see `.clang-format`). Run it before committing.
- **Static analysis**: clang-tidy with `WarningsAsErrors: "*"` (see `.clang-tidy` for the enabled
  families and the explicit disables, e.g. `readability-magic-numbers`, `readability-identifier-length`).
  Identifier-naming conventions are present in `.clang-tidy` but currently commented out /
  not enforced — don't assume they're active.
- **Doxygen**: triple-slash `///`, backslash commands, never `@`-style:
  ```cpp
  /// \brief Element read-only access with bounds checking
  /// \param[in] row Row index
  /// \param[in] col Column index
  /// \return Element value or an error if the index is out of bounds
  ```
  Every class/function needs `\brief`; every template parameter needs `\tparam`; every parameter
  needs `\param[in]`/`\param[out]`.
- **Self-contained headers**: every `.h`/`.hpp` must compile standalone (includable with no
  prerequisite includes) — enforced by the `header_tests` CMake target (see Commands above), not
  just a style preference.

## Testing

- GoogleTest, entrypoint `tests/test.cpp`, layout mirrors `include/trackingLib/` under `tests/`.
- **Naming convention**: `<operation>__<expected_result>`, e.g. `ctor_Zeros__Success`,
  `op_at__FailBadRowIdx`. Add `// NOLINT` where clang-tidy objects to the test name format.
- Use `TYPED_TEST`/`TYPED_TEST_SUITE` for template instantiations (e.g. row-major vs.
  column-major), `TEST_P`/`INSTANTIATE_TEST_SUITE_P` for parameterized value sweeps.
- Test-only macros flip access for testability and must never leak into non-test builds:
  `TEST_REMOVE_FINAL`, `TEST_REMOVE_PROTECTED`, `TEST_REMOVE_PRIVATE`, `TEST_VIRTUAL`.
- Wrap matrix literal blocks in `// clang-format off` / `// clang-format on` for readability.

## Constraints & boundaries

Never:
- Violate **AUTOSAR C++14 compliance** — see the
  [guidelines](https://sbmueller.github.io/autosar_cpp_guidelines/index.html). Not a style
  preference; it's why there's no dynamic memory allocation and no exceptions in core algorithms
  (`tl::expected` instead) — both required for deterministic, safety-relevant behavior.
- Rely on C++20 features outside the experimental contracts path — **C++17 is the floor**.
- Rely on GCC- or Clang-specific extensions — CI builds both compilers × C++17/20 × Debug/Release
  (8 combinations; see Commands).
- Reintroduce OpenMP/thread-based parallelism without revisiting the tradeoff explicitly: it was
  deliberately removed — typical matrix sizes here (≤15×15) are too small to benefit, thread
  overhead exceeded the gain, and non-deterministic scheduling conflicts with AUTOSAR determinism.

Ask first:
- Before adding a new external dependency (there are currently exactly two: GoogleTest, tl::expected).
- Before changing `.clang-format` / `.clang-tidy` — note neither is actually run in CI (verified:
  no such step in `.github/workflows/`), so nothing upstream will catch a violation either way;
  compliance is on convention only.
- Before removing or renaming a public header — this is a header-only lib consumed via
  `find_package(trackingLib)`, so that's a breaking API change for downstream consumers.

## Known limitations

- out-of-sequence-measurements
- block update of multiple synchronized sensors without exploding branching
- motion and observation models just in 2D
- No maneuvering/turning target motion model: `MotionModelCV`/`MotionModelCA` are both
  Cartesian-decoupled (no heading/turn-rate state), so neither can represent a curvature reversal
  — demonstrated by the documented NEES spike at each self-intersection in
  `examples/single_nonlinear_figure8_object_tracking.cpp`. Tested empirically: CA substantially
  reduces both lag and NEES inconsistency versus CV (mean NEES ~1.73 vs. CV's 9.08, nominal ~2.0;
  fraction of samples exceeding the 99% χ² bound drops from 38.5% to 0.8%), but still shows a
  smaller residual NEES spike at each reversal since it still has no jerk mechanism. CTRV/CTRA
  would fix the reversal itself directly. See `plans/recent/figure-8-exploration-ideas.md` for the
  full analysis.
- CTRV/CTRA target motion models not implemented. Architecture is largely ready (`computeA`/
  `applyProcessModel` are already non-static, and `env/ego_motion.h(pp)` has ready-to-adapt
  circular-arc kinematics/Jacobian math) except one contract gap: `computeQ`/`computeG` are
  static-asserted `static` in `contracts/motion_model_intf.h`, which blocks the state-dependent
  (heading-projected) process noise a turning model naturally needs.
- `UnscentedKalmanFilter` is a header stub — not implemented. Non-trivial because the current
  motion-model contract is EKF-shaped by construction (`computeA`/Jacobian is a required method
  that UKF never uses); needs either a parallel UKF-specific contract or a new
  `generic::PredictUnscented` path. Only pays off once paired with a genuinely nonlinear model
  (i.e. sequence after CTRV/CTRA, not before).
- No IMM (Interacting Multiple Model) wrapper, and no precedent for one in the repo. Feasible
  without violating the no-dynamic-allocation constraint (a compile-time tuple-of-models fits
  better than runtime type erasure), and `state_vec_converter.h`/`state_cov_converter.h` already
  give pairwise CV↔CA conversion as a starting primitive — but IMM needs N-way probability-weighted
  mixing, a genuine generalization, not reuse. Only useful once a turning model exists to mix in.
