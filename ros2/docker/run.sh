#!/bin/bash
# Open a ROS 2 Humble shell for the scan arm, building the image the first time.
#
#   ros2/docker/run.sh                    # a shell: colcon build, ros2 launch ..., ros2 run ...
#   ros2/docker/run.sh ros2 launch so101_scan_bringup scan_arm.launch.py
#
# Run it again in another terminal for a second shell: every container shares the host's
# network, so they all see the same ROS topics.
#
# On the host:  ~/so101_scan  calibration.yaml and the scans   (/data in the container)
#               ~/so101_ws    colcon's build/install/log       (/ws)
#               this repo                                      (/ws/src/linescan-hyperspectral-splatting)
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(cd "$HERE/../.." && pwd)
IMAGE=${SO101_IMAGE:-so101_scan:humble}
DATA=${SO101_SCAN_DATA:-$HOME/so101_scan}
WS=${SO101_WS:-$HOME/so101_ws}
mkdir -p "$DATA" "$WS/src"

if ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
    echo "Building $IMAGE (once; about 10 minutes on a Jetson Nano)"
    docker build -t "$IMAGE" "$HERE"
fi

TTY=-i
[ -t 0 ] && TTY=-it

# --network host --ipc host   ROS 2 discovery and Foxglove (port 8765) on the host's network
# --security-opt seccomp=...  JetPack 4's Docker refuses a call Ubuntu 22.04 uses to start
#                             threads (clone3); without this, ROS nodes fail to start
# -v /dev:/dev + cgroup rules USB serial ports (ttyACM: 166, ttyUSB: 188), including after
#                             unplugging and plugging back in, which --device would not survive
exec docker run --rm $TTY \
    --network host --ipc host \
    --security-opt seccomp=unconfined \
    -v /dev:/dev --device-cgroup-rule 'c 166:* rmw' --device-cgroup-rule 'c 188:* rmw' \
    -v "$WS:/ws" \
    -v "$REPO:/ws/src/linescan-hyperspectral-splatting" \
    -v "$DATA:/data" \
    -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
    "$IMAGE" "$@"
