#!/usr/bin/env bash
#
# Verify every header under include/trackingLib/ compiles standalone (no prerequisite includes),
# via the header_tests CMake target.
#
# Usage:
#   ./scripts/verify_self_contained_headers.sh
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="${BUILD_DIR:-${REPO_ROOT}/build}"

cmake -S "${REPO_ROOT}" -B "${BUILD_DIR}" -DBUILD_HEADER_TESTS=ON
cmake --build "${BUILD_DIR}" --target header_tests
ctest --test-dir "${BUILD_DIR}" -R "header_test_" --output-on-failure
