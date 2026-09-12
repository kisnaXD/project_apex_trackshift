#!/usr/bin/env bash
# Drive the EUFS car straight for 10 seconds, then zero commands.
# Real command path: AckermannDriveStamped on /eufs/cmd (accel + steer) and
# Twist on /eufs/cmd_vel (target speed for gazebo_ros_ackermann_drive).
# Unpauses the existing gzserver. Does not start Gazebo, RViz, or a second stack.
#
# Host:  sg docker -c './scripts/drive_straight_10s.sh'
# Inside the container the same file runs the ROS commands directly.
set -eo pipefail

SPEED="${SPEED:-8.0}"
ACCEL="${ACCEL:-8.0}"
DURATION="${DURATION:-10}"

_inside() {
  set +u
  source /opt/ros/humble/setup.bash
  source /opt/eufs_ws/install/setup.bash
  set -u
  echo "Unpausing physics, then 10s front throttle:"
  echo "  /eufs/cmd  AckermannDriveStamped speed=${SPEED} accel=${ACCEL} steer=0"
  echo "  /eufs/cmd_vel Twist linear.x=${SPEED} angular.z=0"
  ros2 service call /unpause_physics std_srvs/srv/Empty >/dev/null || true
  timeout "${DURATION}" ros2 topic pub -r 20 /eufs/cmd ackermann_msgs/msg/AckermannDriveStamped \
    "{drive: {steering_angle: 0.0, acceleration: ${ACCEL}, speed: ${SPEED}}}" \
    >/dev/null &
  ack_pid=$!
  timeout "${DURATION}" ros2 topic pub -r 20 /eufs/cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: ${SPEED}, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" \
    || true
  wait "${ack_pid}" 2>/dev/null || true
  ros2 topic pub --once /eufs/cmd ackermann_msgs/msg/AckermannDriveStamped \
    "{drive: {steering_angle: 0.0, acceleration: 0.0, speed: 0.0}}" >/dev/null || true
  ros2 topic pub --once /eufs/cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
  echo "Stopped: zero /eufs/cmd and /eufs/cmd_vel"
}

if [[ -f /.dockerenv ]] || [[ "${EUFS_INSIDE:-}" == "1" ]]; then
  _inside
else
  docker exec eufs-f1-sim bash -lc 'EUFS_INSIDE=1 /opt/eufs-f1-sim/scripts/drive_straight_10s.sh'
fi
