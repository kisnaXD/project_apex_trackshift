# EUFS F1 backend repair record

The final authored stack uses one Ackermann command contract on
`/eufs/cmd`. DynamicBicycle consumes that command through the native racecar
plugin with `commandMode:=acceleration`; the authored Ackermann alternative
remains available explicitly. Clock subscriptions use Gazebo-compatible
BEST_EFFORT QoS, and local ROS/Gazebo discovery is pinned to localhost.

TF has one authority: for native EUFS odometry, `map→odom` is the immutable
COTA spawn transform and `odom→base_link` is the native spawn-relative pose.
The plugin TF output is disabled for this path. The Ackermann alternative
retains identity `map→odom` with world odometry.

The accepted motion source aligns the chassis visual nose with +X, advances
native wheel joints by `vx/r * dt`, uses native-profile `mu=0` while retaining
Ackermann `mu=0.8`, and uses corrected wheel and steering-hub inertias.

Headless validation artifact: `.tmp/wheel-inertia-pass-20260912.json`.

- Distance: `20.050475 m`
- Maximum speed: `1.99720947 m/s`
- Wheel omega error: `≤0.005829 rad/s`
- Wheel phase residual: `≤0.000079012 rad`
- Backwards wheel steps: none
- Stopped phase variation: `<2e-15 rad` for `9.42 s`
- Stopped omega: zero
- Maximum steering: `0.000162423 rad` (`0.009306 deg`)

The final candidate image is `eufs-f1-sim:forward-rolling-20260912`.
Dashboard/Gazebo/RViz acceptance and operator button verification are recorded
after the final GUI runtime check.
