#!/usr/bin/env bash
#
# Deterministically (re)generate the example tracking GIFs in doc/media/.
#
# The whole pipeline is fixed: build each example -> run it to emit a CSV -> render the CSV to a
# GIF with examples/viz/render.py. No decisions are made here, so given the SAME pinned environment
# (the .devcontainer image trackinglib:latest, which pins matplotlib/numpy/Pillow/fonts) the output
# is byte-for-byte reproducible. Byte-identity is only guaranteed WITHIN that pinned environment -
# a different matplotlib/Pillow/font stack may re-render pixel-identical scenes to different bytes.
#
# Usage:
#   ./scripts/regenerate_media.sh            # regenerate the GIFs in place under doc/media/
#   ./scripts/regenerate_media.sh --check    # regenerate into a temp dir and fail if it differs from
#                                             # what's committed (no writes to doc/media/) - for CI
#
# Determinism is not the C++/Python's job alone; this script pins the ambient locale/timezone/
# backend/seed knobs that can otherwise perturb rendering or number formatting.
set -euo pipefail

# --- pinned, reproducible environment ------------------------------------------------------------
export SOURCE_DATE_EPOCH=0 # reproducible timestamps for any tool that embeds them
export MPLBACKEND=Agg      # headless matplotlib backend, no display-dependent rendering
export PYTHONHASHSEED=0    # deterministic Python hashing
export TZ=UTC              # stable timezone
export LC_ALL=C            # stable locale => stable number formatting

# --- locations -----------------------------------------------------------------------------------
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="${BUILD_DIR:-${REPO_ROOT}/build}"
MEDIA_DIR="${REPO_ROOT}/doc/media"
RENDER="${REPO_ROOT}/examples/viz/render.py"

# --- the fixed pipeline table: <cmake target> <csv basename> <gif basename> ----------------------
# Single source of truth for the example -> csv -> gif mapping. Add a row here to add an example.
EXAMPLES=(
  "single_linear_object_tracking            single_linear_track.csv            single_linear_tracking.gif"
  "single_nonlinear_object_tracking         single_nonlinear_track.csv         single_nonlinear_tracking.gif"
  "single_nonlinear_figure8_object_tracking single_nonlinear_figure8_track.csv single_nonlinear_figure8_tracking.gif"
)

CHECK_MODE=0
if [[ "${1:-}" == "--check" ]]; then
  CHECK_MODE=1
elif [[ -n "${1:-}" ]]; then
  echo "error: unknown argument '${1}' (expected nothing or --check)" >&2
  exit 2
fi

# --- build the example targets (idempotent) ------------------------------------------------------
targets=()
for row in "${EXAMPLES[@]}"; do
  read -r target _csv _gif <<<"${row}"
  targets+=("${target}")
done

echo ">> configuring + building ${#targets[@]} example target(s) in ${BUILD_DIR}"
cmake -S "${REPO_ROOT}" -B "${BUILD_DIR}" -DCMAKE_BUILD_TYPE=Debug >/dev/null
cmake --build "${BUILD_DIR}" --target "${targets[@]}" -j"$(nproc)" >/dev/null

# --- run each example and render its GIF into a scratch dir --------------------------------------
WORK_DIR="$(mktemp -d)"
trap 'rm -rf "${WORK_DIR}"' EXIT

for row in "${EXAMPLES[@]}"; do
  read -r target csv gif <<<"${row}"
  echo ">> ${target}"
  ( cd "${WORK_DIR}" && "${BUILD_DIR}/examples/${target}" "${csv}" >/dev/null )
  python3 "${RENDER}" "${WORK_DIR}/${csv}" "${WORK_DIR}/${gif}"
done

# --- either verify (--check) or install the freshly rendered GIFs --------------------------------
if [[ "${CHECK_MODE}" -eq 1 ]]; then
  drift=0
  for row in "${EXAMPLES[@]}"; do
    read -r _target _csv gif <<<"${row}"
    if ! cmp -s "${WORK_DIR}/${gif}" "${MEDIA_DIR}/${gif}"; then
      echo "DRIFT: doc/media/${gif} is stale - re-run ./scripts/regenerate_media.sh and commit" >&2
      drift=1
    fi
  done
  if [[ "${drift}" -eq 1 ]]; then
    exit 1
  fi
  echo ">> check OK: doc/media is up to date"
else
  for row in "${EXAMPLES[@]}"; do
    read -r _target _csv gif <<<"${row}"
    cp "${WORK_DIR}/${gif}" "${MEDIA_DIR}/${gif}"
  done
  echo ">> wrote ${#EXAMPLES[@]} GIF(s) to doc/media/"
  echo ">> git status doc/media:"
  git -C "${REPO_ROOT}" status --short -- doc/media || true
fi