#!/bin/bash
# splat/ on the Jetson Nano: builds it for the GPU (CUDA 10.2, sm_53) and runs it, in a
# container that gets JetPack's CUDA from the NVIDIA runtime.
#
#   jetson/splat.sh build                              # the image (first time), then splat
#   jetson/splat.sh test                               # the unit tests, the GPU ones included
#   jetson/splat.sh splat_synth synth --preset small   # any splat tool, or any command
#   jetson/splat.sh shell                              # a shell in the container
#   jetson/splat.sh image                              # rebuild the image
#
# It all happens in ~/linesplat (/data in the container): the build goes to ~/linesplat/build,
# and the tools run in ~/linesplat, so datasets and results land there. The repo is mounted
# read-only at /src.
#
# Settings, from the environment:
#   LINESPLAT_HOME       ~/linesplat        where the build and the data go
#   LINESPLAT_JOBS       2                  compiles at a time (the Nano has 4 GB)
#   LINESPLAT_CUDA_ARCH  53                 the GPU: 53 is the Nano's Maxwell
#   LINESPLAT_GPU        1                  0: no NVIDIA runtime, so CPU only unless the image
#                                           has its own CUDA (jetson/check.sh uses both)
#   LINESPLAT_IMAGE      linesplat:r32.7.1
#   LINESPLAT_BASE       nvcr.io/nvidia/l4t-base:r32.7.1   what the image is built from
#   LINESPLAT_PLATFORM   (none)             e.g. linux/arm64, to emulate the Nano on a PC
set -euo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(cd "$HERE/.." && pwd)
DATA=${LINESPLAT_HOME:-$HOME/linesplat}
JOBS=${LINESPLAT_JOBS:-2}
ARCH=${LINESPLAT_CUDA_ARCH:-53}
GPU=${LINESPLAT_GPU:-1}
IMAGE=${LINESPLAT_IMAGE:-linesplat:r32.7.1}
BASE=${LINESPLAT_BASE:-nvcr.io/nvidia/l4t-base:r32.7.1}
PLATFORM=${LINESPLAT_PLATFORM:-}

if [ $# -eq 0 ]; then
    sed -n '2,10p' "$0" | sed 's/^# \{0,1\}//'
    exit 2
fi

build_image() {
    echo "Building $IMAGE from $BASE"
    docker build ${PLATFORM:+--platform "$PLATFORM"} --build-arg BASE="$BASE" -t "$IMAGE" "$HERE"
}

# Run a command in the container, as you rather than root so that ~/linesplat stays yours.
# The GPU's device nodes belong to the video group, so the container gets their groups too.
run() {
    docker image inspect "$IMAGE" >/dev/null 2>&1 || build_image
    mkdir -p "$DATA"
    local args=(--rm -v "$REPO:/src:ro" -v "$DATA:/data" -w /data
                --user "$(id -u):$(id -g)" -e HOME=/data
                -e LINESPLAT_GPU="$GPU" -e LINESPLAT_JOBS="$JOBS" -e LINESPLAT_CUDA_ARCH="$ARCH")
    if [ -t 0 ] && [ -t 1 ]; then args+=(-it); fi
    if [ "$GPU" = 1 ]; then
        args+=(--runtime nvidia)
        local gid
        for gid in $(stat -c %g /dev/nvhost-ctrl /dev/nvhost-ctrl-gpu /dev/nvmap 2>/dev/null | sort -u); do
            args+=(--group-add "$gid")
        done
    fi
    if [ -n "$PLATFORM" ]; then args+=(--platform "$PLATFORM"); fi
    docker run "${args[@]}" "$IMAGE" "$@"
}

case "$1" in
    image)
        build_image
        ;;
    build)
        # shellcheck disable=SC2016  # expanded in the container
        run bash -c '
            set -e
            if [ -x /usr/local/cuda/bin/nvcc ]; then
                cuda="-DCMAKE_CUDA_COMPILER=/usr/local/cuda/bin/nvcc -DCMAKE_CUDA_ARCHITECTURES=$LINESPLAT_CUDA_ARCH"
            elif [ "$LINESPLAT_GPU" = 1 ]; then
                echo "No CUDA in the container: the NVIDIA runtime mounts JetPack'"'"'s CUDA 10.2 into it," >&2
                echo "so check that this is a Jetson with JetPack 4.6 (jetson/README.md)." >&2
                exit 1
            else
                cuda=-DLINESPLAT_CUDA=OFF
            fi
            cmake -S /src/splat -B /data/build -DCMAKE_BUILD_TYPE=Release $cuda
            cmake --build /data/build -j "$LINESPLAT_JOBS"'
        ;;
    test)
        # With the GPU expected, a CUDA test that skips for want of a device is a failure.
        log=$(mktemp)
        trap 'rm -f "$log"' EXIT
        status=0
        run /data/build/splat_tests "${@:2}" | tee "$log" || status=$?
        if [ "$GPU" = 1 ] && grep -q "no CUDA device" "$log"; then
            echo "The GPU isn't visible in the container, so the CUDA tests were skipped." >&2
            exit 1
        fi
        exit "$status"
        ;;
    shell)
        run bash
        ;;
    *)
        # shellcheck disable=SC2016  # expanded in the container
        run bash -c 'PATH=/data/build:$PATH; exec "$@"' bash "$@"
        ;;
esac
