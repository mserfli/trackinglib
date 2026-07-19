#!/bin/bash
# 1. Seamless headroom wrapping with the MCP proxy URL built right in
alias claude="ANTHROPIC_BASE_URL=http://127.0.0.1:8787 headroom wrap claude"

# 2. A quick shortcut to spin up the background proxy process in your first terminal
alias hr-proxy="headroom proxy"
