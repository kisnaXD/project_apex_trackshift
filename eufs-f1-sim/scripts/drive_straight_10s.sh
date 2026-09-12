#!/usr/bin/env bash
# Drive the EUFS car straight for 10 seconds, then zero /eufs/cmd_vel.
# Unpauses the existing gzserver. Does not start Gazebo, RViz, or a second stack.
#
# Host:  sg docker -c './scripts/drive_straight_10s.sh'
# Inside the container the same file runs the ROS commands directly.
set -euo pipefail

SPEED="${SPEED:-1.0}"
DURATION="${DURATION:-10}"

_inside() {
  source /opt/ros/humble/setup.bash
  source /opt/eufs_ws/install/setup.bash
  echo "Unpausing physics, then publishing linear.x=${SPEED} for ${DURATION}s on /eufs/cmd_vel"
  ros2 service call /unpause_physics std_srvs/srv/Empty >/dev/null || true
  timeout "${DURATION}" ros2 topic pub -r 10 /eufs/cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: ${SPEED}, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" \
    || true
  ros2 topic pub --once /eufs/cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
  echo "Stopped: zero /eufs/cmd_vel"
}

if [[ -f /.dockerenv ]] || [[ "${EUFS_INSIDE:-}" == "1" ]]; then
  _inside
else
  docker exec eufs-f1-sim bash -lc 'EUFS_INSIDE=1 /opt/eufs-f1-sim/scripts/drive_straight_10s.sh'
fi
