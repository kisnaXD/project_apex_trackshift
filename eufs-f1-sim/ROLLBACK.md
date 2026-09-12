# EUFS F1 rollback record

## Immediate prior baseline

The prior usable baseline is image `eufs-f1-sim:repaired-20260912`, preserved
in container `eufs-f1-sim-before-wheel-fix-20260912`. The source backup is the
authored racecar overlay only: `.tmp/before-wheel-repair-20260912`.

To preserve an active candidate without a name collision and restore the
baseline container:

```bash
archive="eufs-f1-sim-candidate-archive-$(date +%Y%m%d-%H%M%S)"
docker stop eufs-f1-sim
docker rename eufs-f1-sim "$archive"
docker rename eufs-f1-sim-before-wheel-fix-20260912 eufs-f1-sim
docker start eufs-f1-sim
```

If that preserved container is unavailable, recreate the baseline image
directly (without Compose container discovery):

```bash
docker run -d --name eufs-f1-sim --network host --ipc host --privileged \
  -e DISPLAY="${DISPLAY:-:0}" -e QT_X11_NO_MITSHM=1 \
  -e EUFS_MASTER=/opt/eufs_ws -e ROS_LOCALHOST_ONLY=1 \
  -e GAZEBO_IP=127.0.0.1 -e GAZEBO_MASTER_URI=http://127.0.0.1:11345 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  eufs-f1-sim:repaired-20260912 ros2 launch eufs_racecar \
  load_car.launch.py track:=cota cars:=1 vehicleModel:=DynamicBicycle \
  commandMode:=acceleration gazebo_gui:=false show_rqt_gui:=false rviz:=false
```

## Current wheel-fix result

The final candidate image is `eufs-f1-sim:forward-rolling-20260912`.
Headless artifact `.tmp/wheel-inertia-pass-20260912.json` records distance
`20.050475 m`, maximum speed `1.99720947 m/s`, omega error at most
`0.005829 rad/s`, phase residual at most `0.000079012 rad`, no backwards wheel
steps, stopped phase variation below `2e-15 rad` for `9.42 s`, zero stopped
omega, and maximum steering `0.000162423 rad` (`0.009306 deg`).

The accepted source aligns the chassis nose with +X, advances native wheel
joints by `vx/r * dt`, uses native-profile `mu=0` and Ackermann `mu=0.8`, and
uses corrected wheel/hub inertias. GUI and dashboard acceptance is recorded
separately after final runtime verification.

## Earliest fallback

The original preserved image is `eufs-f1-sim:rollback-20260912-194453`
(`sha256:4fa3e1b4aa87c29156eb30f2589bc3b7831ea1a2ac0163c713b98877e9126151`),
with preserved container `eufs-f1-sim-rollback-20260912-194453`. Use the same
unique archive procedure before assigning the canonical container name.
