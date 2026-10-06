# ROS 2 Humble for the SO-101 scan arm.
#
# Humble needs Ubuntu 22.04 and the Jetson Nano's JetPack 4 is Ubuntu 18.04, so ROS runs in
# this container; the host only needs Docker (JetPack has it). Nothing here needs the GPU.
# The same image works on any 64-bit Linux PC. ros2/docker/run.sh builds and starts it.
FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-ros2-control \
        ros-humble-ros2-controllers \
        ros-humble-xacro \
        ros-humble-robot-state-publisher \
        ros-humble-joint-state-publisher \
        ros-humble-foxglove-bridge \
        python3-colcon-common-extensions \
        python3-numpy \
        python3-pytest \
        python3-serial \
        python3-yaml \
    && rm -rf /var/lib/apt/lists/*

# `colcon build` in /ws builds the repo's ros2/ packages (run.sh mounts the repo under /ws/src).
# Two jobs at a time: the Nano has 4 GB of memory.
RUN mkdir -p /root/.colcon && printf '%s\n' \
    'build:' \
    '  base-paths: [/ws/src/linescan-hyperspectral-splatting/ros2]' \
    '  symlink-install: true' \
    '  parallel-workers: 2' \
    '  cmake-args: [-DCMAKE_BUILD_TYPE=Release]' \
    'test:' \
    '  base-paths: [/ws/src/linescan-hyperspectral-splatting/ros2]' \
    > /root/.colcon/defaults.yaml
ENV MAKEFLAGS=-j2
# calibration.yaml and the scans go here (run.sh mounts ~/so101_scan)
ENV SO101_SCAN_DATA=/data

COPY entrypoint.sh /entrypoint.sh
WORKDIR /ws
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
