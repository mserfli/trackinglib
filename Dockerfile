FROM --platform=linux/amd64 ubuntu:22.04
LABEL Description="Build environment"

ENV HOME /root

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get -y --no-install-recommends install \
    build-essential \
    clang \
    clangd \
    cmake \
    doxygen \
    graphviz \
    octave \
    gdb \
    wget \
    git \
    ca-certificates \
    python3 \
    python3-dev \
    lcov

RUN cd ${HOME} && \
    wget --no-check-certificate --quiet \
        https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh && \
        sh Miniconda3-latest-Linux-x86_64.sh -b -p ${HOME}/miniconda3 && \
        rm -rf Miniconda3-latest-Linux-x86_64.sh && \
        miniconda3/bin/conda init bash
