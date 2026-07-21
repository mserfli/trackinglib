#!/bin/bash
# 1. The real proxy — this is the one process that should exist; flags configured here
alias hr-proxy="headroom proxy --code-aware"

# 2. Reuse the running proxy; don't let wrap spawn its own or re-trigger rtk init
alias claude="ANTHROPIC_BASE_URL=http://127.0.0.1:8787 headroom wrap claude --no-proxy --no-rtk"
