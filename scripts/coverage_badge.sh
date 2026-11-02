#!/usr/bin/env bash
set -euo pipefail

LCOV_INFO="build_cov/lcov.info"
OUT_DIR="build_cov/coverage"
OUT_FILE="${OUT_DIR}/coverage-badge.json"

if [ ! -f "${LCOV_INFO}" ]; then
    echo "error: ${LCOV_INFO} not found — run ./scripts/coverage_report.sh first" >&2
    exit 1
fi

# Read the tracefile's line totals directly. A line-coverage badge does not
# require lcov's version-dependent summary output or function metadata checks.
PCT_RAW=$(LC_ALL=C awk -F: '
    /^LF:/ {
        if ($2 !~ /^[0-9]+$/) exit 1
        found += $2
        found_records++
    }
    /^LH:/ {
        if ($2 !~ /^[0-9]+$/) exit 1
        hit += $2
        hit_records++
    }
    END {
        if (found_records == 0 || found_records != hit_records || found <= 0 || hit > found) exit 1
        printf "%.1f", 100 * hit / found
    }
' "${LCOV_INFO}") || {
    echo "error: could not read valid line coverage totals from ${LCOV_INFO}" >&2
    exit 1
}

PCT=$(LC_ALL=C printf '%.0f' "${PCT_RAW}")

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
