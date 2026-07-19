ARG DEFAULT_PLATFORM=linux/arm64
FROM --platform=${DEFAULT_PLATFORM} ubuntu:24.04
LABEL Description="Build environment"

# Standard key=value syntax fixes the LegacyKeyValueFormat warning
ENV HOME=/root

SHELL ["/bin/bash", "-c"]

# Pure system dependencies - keeping python3-pip for Headroom
RUN apt-get update && apt-get -y --no-install-recommends install \
    build-essential \
    clang \
    clang-format \
    clangd \
    cmake \
    doxygen \
    graphviz \
    octave \
    gdb \
    wget \
    git \
    gh \
    ca-certificates \
    python3 \
    python3-dev \
    python3-pip \
    lcov \
    curl \
    ripgrep \
    && rm -rf /var/lib/apt/lists/*
    