FROM osrf/ros:humble-desktop-full

ENV DEBIAN_FRONTEND=noninteractive
ENV EUFS_MASTER=/opt/eufs_ws
WORKDIR /opt/eufs-f1-sim

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-xacro \
    ros-humble-joint-state-publisher \
    ros-humble-robot-state-publisher \
    ros-humble-rqt-gui \
    ros-humble-ackermann-msgs \
    git \
    && rm -rf /var/lib/apt/lists/*

COPY . /opt/eufs-f1-sim/

RUN chmod +x /opt/eufs-f1-sim/scripts/*.sh \
    && /opt/eufs-f1-sim/scripts/prepare-workspace.sh \
    && rosdep init || true \
    && rosdep update \
    && cd /opt/eufs_ws \
    && rosdep install --from-paths src --ignore-src -r -y \
    && . /opt/ros/humble/setup.bash \
    && colcon build --symlink-install

COPY scripts/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENV GAZEBO_MODEL_PATH=${EUFS_MASTER}/install/eufs_tracks/share/eufs_tracks/models
WORKDIR /opt/eufs_ws

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash", "-lc", "ros2 launch eufs_tracks small_track.launch.py gazebo_gui:=true show_rqt_gui:=false vehicleModelConfig:=configDry.yaml"]
