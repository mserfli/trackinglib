#!/bin/bash
set -e # Exit immediately if any command fails

echo "=== Installing System Dependencies ==="
apt-get update && apt-get install -y universal-ctags clang-format clang-tidy

echo "=== Generating Ctags ==="
./scripts/regenerate_ctags.sh

echo "=== Installing Claude Code ==="
curl -fsSL https://claude.ai/install.sh | bash

echo "=== Installing OpenAI Codex ==="
curl -fsSL https://chatgpt.com/codex/install.sh | sh

echo "=== Installing Rust Token Killer ==="
curl -fsSL https://raw.githubusercontent.com/rtk-ai/rtk/refs/heads/master/install.sh | BINDIR=/usr/local/bin sh

echo "=== Registering RTK Claude/Codex Hook ==="
rtk init -g --auto-patch

echo "=== Installing Python Dependencies ==="
pip3 install --break-system-packages matplotlib numpy fastapi "httpx[http2]" "headroom-ai[mcp,proxy,ml,code]"

echo "=== Configuring Headroom MCP ==="
headroom mcp install

echo "=== Configuring Workspace Aliases ==="
if ! grep -q "aliases.sh" ~/.bashrc; then
    echo 'source /workspace/.devcontainer/aliases.sh' >> ~/.bashrc
fi

echo "=== Setup Complete! ==="
