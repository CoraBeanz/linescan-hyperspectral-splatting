#!/bin/bash
# Checks from a PC that splat/ builds and passes its tests with the Jetson Nano's toolchain,
# using jetson/Dockerfile and jetson/splat.sh as the Nano does, in two ways:
#
#   cuda   x86 Ubuntu 18.04 with CUDA 10.2 and gcc 7.5, JetPack 4.6's compilers: splat with
#          the CUDA rasterizer for sm_53, a check that every kernel fits the Nano's GPU, and
#          the tests (the GPU ones skip: this checks the build, not a GPU)
#   arm64  arm64 Ubuntu 18.04 under emulation, the Nano's CPU and distro: the CPU build and
#          the tests
#
#   jetson/check.sh              # both
#   jetson/check.sh cuda         # or arm64
#
# Needs Docker. arm64 needs QEMU registered with the kernel: Docker Desktop has it, and on Linux
#   docker run --privileged --rm tonistiigi/binfmt --install arm64
# The builds go to build/jetson-check/ in the repo. To start from other images:
#   LINESPLAT_CHECK_CUDA_BASE   Ubuntu 18.04 with CUDA 10.2 in /usr/local/cuda, instead of
#                               building check/cuda102.Dockerfile
#   LINESPLAT_CHECK_ARM64_BASE  instead of ubuntu:18.04
set -euo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(cd "$HERE/.." && pwd)
OUT=$REPO/build/jetson-check
JOBS=${LINESPLAT_JOBS:-$(nproc)}

step() { printf '\n=== %s\n\n' "$*"; }

check_cuda() {
    local base=${LINESPLAT_CHECK_CUDA_BASE:-}
    if [ -z "$base" ]; then
        base=linesplat-check:cuda10.2
        step "cuda: Ubuntu 18.04 with CUDA 10.2"
        docker build -t "$base" -f "$HERE/check/cuda102.Dockerfile" "$HERE/check"
    fi
    splat() {
        LINESPLAT_BASE=$base LINESPLAT_IMAGE=linesplat:check-cuda10.2 LINESPLAT_GPU=0 \
            LINESPLAT_HOME=$OUT/cuda LINESPLAT_JOBS=$JOBS "$HERE/splat.sh" "$@"
    }
    step "cuda: the Nano's image, from that"
    splat image
    step "cuda: splat for sm_53"
    splat build
    splat nvcc --version
    step "cuda: does every kernel fit the Nano's GPU?"
    splat python3 /src/jetson/check/kernels.py /data/build/liblinesplat_cuda.a
    step "cuda: tests"
    splat test
}

check_arm64() {
    local base=${LINESPLAT_CHECK_ARM64_BASE:-ubuntu:18.04}
    if [ "$(docker run --rm --platform linux/arm64 "$base" uname -m)" != aarch64 ]; then
        echo "Can't run arm64 containers: register QEMU first (see the top of $0)" >&2
        exit 1
    fi
    splat() {
        LINESPLAT_BASE=$base LINESPLAT_IMAGE=linesplat:check-arm64 LINESPLAT_GPU=0 \
            LINESPLAT_PLATFORM=linux/arm64 LINESPLAT_HOME=$OUT/arm64 LINESPLAT_JOBS=$JOBS \
            "$HERE/splat.sh" "$@"
    }
    step "arm64: the Nano's image, from Ubuntu 18.04"
    splat image
    step "arm64: splat, CPU only (emulated, so slow)"
    splat build
    step "arm64: tests"
    splat test
}

case "${1:-all}" in
    cuda) check_cuda ;;
    arm64) check_arm64 ;;
    all) check_cuda; check_arm64 ;;
    *) sed -n '2,13p' "$0" | sed 's/^# \{0,1\}//'; exit 2 ;;
esac
step "Passed: ${1:-all}"
