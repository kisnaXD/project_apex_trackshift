#!/usr/bin/env bash
# Drive the EUFS car straight for 10 seconds, then zero commands.
# Real command path: AckermannDriveStamped on /eufs/cmd (accel + steer).
# The launch/bridge owner is responsible for any plant-specific conversion.
# Unpauses the existing gzserver. Does not start Gazebo, RViz, or a second stack.
#
# Host:  sg docker -c './scripts/drive_straight_10s.sh'
# Inside the container the same file runs the ROS commands directly.
set -eo pipefail

SPEED="${SPEED:-2.0}"
ACCEL="${ACCEL:-0.5}"
DURATION="${DURATION:-10}"

_inside() {
  set +u
  source /opt/ros/humble/setup.bash
  source /opt/eufs_ws/install/setup.bash
  set -u
  echo "Waiting for /eufs/cmd, unpausing physics, then 10 simulated seconds:"
  echo "  /eufs/cmd AckermannDriveStamped speed=${SPEED} accel=${ACCEL} steer=0"
  python3 /opt/eufs-f1-sim/scripts/drive_straight_10s.py \
    --topic /eufs/cmd --speed "${SPEED}" --accel "${ACCEL}" --duration "${DURATION}"
  echo "Stopped: zero /eufs/cmd"
}

if [[ -f /.dockerenv ]] || [[ "${EUFS_INSIDE:-}" == "1" ]]; then
  _inside
else
  docker exec eufs-f1-sim bash -lc 'EUFS_INSIDE=1 /opt/eufs-f1-sim/scripts/drive_straight_10s.sh'
fi
