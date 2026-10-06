#!/bin/bash
# ROS in every shell, plus this workspace once it has been built.
source /opt/ros/humble/setup.bash
[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash
grep -q "ros/humble" /root/.bashrc 2>/dev/null || cat >> /root/.bashrc <<'RC'
source /opt/ros/humble/setup.bash
[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash
RC
exec "$@"
