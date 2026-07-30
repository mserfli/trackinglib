#!/usr/bin/env bash
#
# Rebuild .repo.tags at the repo root — single source of truth for the ctags invocation, also used
# by .devcontainer/setup.sh.
#
# Usage:
#   ./scripts/regenerate_ctags.sh
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

cd "${REPO_ROOT}"
# too large: ctags -R --languages=C++ --map-C++=+.h.hpp.tcc.cpp.cxx --kinds-C++=+p+l+x+t+u+v --fields=+iaSzn --extras=+q+r --exclude=build --exclude=.git --output-format=u-ctags -f .repo.tags .
# optimized: ctags -R --languages=C++ --map-C++=+.h.hpp.tcc.cpp.cxx --kinds-C++=+p+t+u+v-l --fields=+iaS --extras=+q --exclude=build --exclude=.git --exclude=tests -f .repo.tags .
ctags -R --languages=C++ --map-C++=+.h.hpp.tcc.cpp.cxx --kinds-C++=+p+t+u+v-l --fields=+iaS \
  --extras=+q --exclude=build --exclude=.git --exclude=tests -f .repo.tags .
