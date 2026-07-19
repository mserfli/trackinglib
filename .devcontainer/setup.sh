#!/bin/bash
set -e # Exit immediately if any command fails

echo "=== Installing System Dependencies ==="
apt-get update && apt-get install -y universal-ctags

echo "=== Generating Ctags ==="
# too large: ctags -R --languages=C++ --map-C++=+.h.hpp.tcc.cpp.cxx --kinds-C++=+p+l+x+t+u+v --fields=+iaSzn --extras=+q+r --exclude=build --exclude=.git --output-format=u-ctags -f .repo.tags .
# optimized: ctags -R --languages=C++ --map-C++=+.h.hpp.tcc.cpp.cxx --kinds-C++=+p+t+u+v-l --fields=+iaS --extras=+q --exclude=build --exclude=.git --exclude=tests -f .repo.tags .
ctags -R --languages=C++ --map-C++=+.h.hpp.tcc.cpp.cxx --kinds-C++=+p+t+u+v-l --fields=+iaS --extras=+q --exclude=build --exclude=.git --exclude=tests -f .repo.tags .

echo "=== Installing Claude Code ==="
curl -fsSL https://claude.ai/install.sh | bash

echo "=== Installing Python Dependencies ==="
pip3 install fastapi "httpx[http2]" "headroom-ai[mcp]"

echo "=== Configuring Headroom MCP ==="
headroom mcp install

echo "=== Configuring Workspace Aliases ==="
if ! grep -q "aliases.sh" ~/.bashrc; then
    echo 'source /workspace/.devcontainer/aliases.sh' >> ~/.bashrc
fi

echo "=== Setup Complete! ==="