#!/bin/bash
# ROS 2 on the Jetson Nano: ros2/docker/run.sh, with its image built in a way that JetPack 4's
# Docker can manage.
#
#   jetson/ros.sh                         # a shell; anything else goes to run.sh as a command:
#   jetson/ros.sh colcon build
#   jetson/ros.sh ros2 launch so101_scan_bringup scan_arm.launch.py use_mock_hardware:=true
#   jetson/ros.sh image                   # build the image again
#
# Humble's image is Ubuntu 22.04, which starts threads and processes with a system call
# (clone3) that Docker before 20.10.10 refuses: JetPack 4.6 comes with 20.10.7. run.sh runs
# its containers without that filter, but `docker build` can't turn it off, so apt fails while
# building the image. On such a Docker this builds the image with a current BuildKit running
# in a container, which answers clone3 the way newer Dockers do (Ubuntu then falls back to
# the old call), and loads the result into Docker. On a newer Docker it's a plain docker build.
#
#   SO101_IMAGE          so101_scan:humble     image name (run.sh reads it too)
#   SO101_BUILD_WITH     buildkit or docker    to choose instead of going by Docker's version
#   SO101_BUILDKIT_IMAGE moby/buildkit:v0.33.1
set -euo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(cd "$HERE/.." && pwd)
CONTEXT="$REPO/ros2/docker"
export SO101_IMAGE=${SO101_IMAGE:-so101_scan:humble}
BUILDKIT=${SO101_BUILDKIT_IMAGE:-moby/buildkit:v0.33.1}

docker_too_old() {
    local v
    v=$(docker version --format '{{.Server.Version}}')
    [ "$(printf '%s\n' 20.10.10 "$v" | sort -V | head -n1)" != 20.10.10 ]
}

build_image() {
    local with=${SO101_BUILD_WITH:-}
    if [ -z "$with" ]; then
        if docker_too_old; then with=buildkit; else with=docker; fi
    fi
    if [ "$with" = buildkit ]; then
        echo "Building $SO101_IMAGE with BuildKit in a container (this Docker can't build Ubuntu 22.04 images)"
        # The volume keeps BuildKit's layer cache between builds (docker volume rm so101_buildkit
        # frees the space).
        docker run --rm --privileged \
            -v "$CONTEXT:/context:ro" -v so101_buildkit:/var/lib/buildkit \
            --entrypoint buildctl-daemonless.sh "$BUILDKIT" \
            build --frontend dockerfile.v0 --local context=/context --local dockerfile=/context \
                  --output "type=docker,name=$SO101_IMAGE" \
            | docker load
    else
        docker build -t "$SO101_IMAGE" "$CONTEXT"
    fi
}

if [ "${1:-}" = image ]; then
    build_image
    exit
fi
docker image inspect "$SO101_IMAGE" >/dev/null 2>&1 || build_image
exec "$REPO/ros2/docker/run.sh" "$@"
