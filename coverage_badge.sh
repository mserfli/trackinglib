#!/usr/bin/env bash
set -euo pipefail

LCOV_INFO="build_cov/lcov.info"
OUT_DIR="build_cov/coverage"
OUT_FILE="${OUT_DIR}/coverage-badge.json"

if [ ! -f "${LCOV_INFO}" ]; then
    echo "error: ${LCOV_INFO} not found — run ./coverage_report.sh first" >&2
    exit 1
fi

PCT_RAW=$(lcov --summary "${LCOV_INFO}" 2>&1 | grep -oP 'lines\.*:\s*\K[0-9.]+') || {
    echo "error: could not parse line coverage percentage from lcov summary" >&2
    exit 1
}

PCT=$(printf '%.0f' "${PCT_RAW}")

if [ "${PCT}" -ge 90 ]; then
    COLOR="brightgreen"
elif [ "${PCT}" -ge 75 ]; then
    COLOR="yellowgreen"
elif [ "${PCT}" -ge 60 ]; then
    COLOR="yellow"
elif [ "${PCT}" -ge 40 ]; then
    COLOR="orange"
else
    COLOR="red"
fi

mkdir -p "${OUT_DIR}"
cat > "${OUT_FILE}" <<EOF
{
  "schemaVersion": 1,
  "label": "Coverage",
  "message": "${PCT}%",
  "color": "${COLOR}"
}
EOF

echo "wrote ${OUT_FILE}: ${PCT}% (${COLOR})"
