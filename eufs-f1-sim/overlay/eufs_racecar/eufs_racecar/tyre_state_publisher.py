"""Motion-driven tyre thermal and wear model for the lean EUFS F1 sim.

Gazebo does not publish tyre temps or life. This node integrates a simple
Pacejka-free thermal/wear model from /odom, /cmd_vel and /distance so the
dashboard can show changing values instead of permanent N/A.
"""

from __future__ import annotations

import math

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray, Float64
import rclpy


WHEEL_RADIUS_M = 0.346
LAP_LENGTH_M = 5513.0
T_AMB_C = 28.0
T_OPT_C = 90.0
MASS_KG = 788.0


class TyreStatePublisher(Node):
    def __init__(self):
        super().__init__('tyre_state_publisher')
        self.declare_parameter('namespace', 'eufs')
        self.declare_parameter('lap_length_m', LAP_LENGTH_M)
        self.declare_parameter('wheel_radius_m', WHEEL_RADIUS_M)
        namespace = str(self.get_parameter('namespace').value).strip('/')
        ns = f'/{namespace}' if namespace else ''

        self.lap_length = max(100.0, float(self.get_parameter('lap_length_m').value))
        self.wheel_radius = max(0.1, float(self.get_parameter('wheel_radius_m').value))

        self.temps = [T_AMB_C + 4.0, T_AMB_C + 3.5, T_AMB_C + 3.0, T_AMB_C + 3.2]
        self.life = [1.0, 1.0, 1.0, 1.0]
        self.lap_wear = 0.0
        self.deg_rate = 0.0
        self._cmd_vx = 0.0
        self._act_vx = 0.0
        self._ax = 0.0
        self._yawrate = 0.0
        self._distance = 0.0
        self._lap_start_distance = 0.0
        self._last_stamp = None
        self._last_v = 0.0

        self.create_subscription(Twist, f'{ns}/cmd_vel' if ns else '/cmd_vel', self._on_cmd, 10)
        self.create_subscription(Odometry, f'{ns}/odom' if ns else '/odom', self._on_odom, 10)
        self.create_subscription(
            Float64, f'{ns}/distance' if ns else '/distance', self._on_distance, 10,
        )

        self.pub_temps = self.create_publisher(Float32MultiArray, f'{ns}/tyres/temps', 10)
        self.pub_rpm = self.create_publisher(Float32MultiArray, f'{ns}/tyres/wheel_rpm', 10)
        self.pub_rate = self.create_publisher(Float32, f'{ns}/tyres/degradation_rate', 10)
        self.pub_lap = self.create_publisher(Float32, f'{ns}/tyres/lap_degradation', 10)
        self.pub_life = self.create_publisher(Float32, f'{ns}/tyres/life', 10)

        self.create_timer(0.1, self._publish)
        self.get_logger().info(
            f'tyre model on {ns or "/"} from odom/cmd_vel/distance → {ns}/tyres/*'
        )

    def _on_cmd(self, msg: Twist):
        self._cmd_vx = float(msg.linear.x)

    def _on_distance(self, msg: Float64):
        self._distance = float(msg.data)

    def _on_odom(self, msg: Odometry):
        vx = float(msg.twist.twist.linear.x)
        self._act_vx = vx
        self._yawrate = float(msg.twist.twist.angular.z)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self._last_stamp is not None and stamp > self._last_stamp + 1e-4:
            dt = stamp - self._last_stamp
            self._ax = (vx - self._last_v) / dt
            self._step(dt)
        self._last_stamp = stamp
        self._last_v = vx

    def _step(self, dt: float):
        dt = max(1e-4, min(dt, 0.2))
        speed = abs(self._act_vx)
        slip = abs(self._cmd_vx - self._act_vx)
        lat = abs(self._yawrate) * max(speed, 0.1)
        load = 1.0 + abs(self._ax) / 9.81
        # Front tyres take more long-acc load; rears more drive slip.
        axle_load = (1.10, 1.10, 0.95, 0.95)
        drive_bias = (0.85, 0.85, 1.20, 1.20)
        for i in range(4):
            gen = (
                0.35 * speed * load * axle_load[i]
                + 1.8 * slip * drive_bias[i]
                + 0.08 * lat
                + 0.012 * speed * speed
            )
            cool = 0.22 * (self.temps[i] - T_AMB_C)
            self.temps[i] = max(T_AMB_C, min(130.0, self.temps[i] + (gen - cool) * dt))
            thermal = (self.temps[i] / T_OPT_C) ** 2
            wear_dot = 2.5e-6 * speed * load * thermal * drive_bias[i] * axle_load[i]
            self.life[i] = max(0.0, self.life[i] - wear_dot * dt)
        self.deg_rate = (
            100.0 * sum(
                2.5e-6 * speed * load * ((self.temps[i] / T_OPT_C) ** 2)
                for i in range(4)
            )
            / 4.0
        )
        distance_into_lap = self._distance - self._lap_start_distance
        if distance_into_lap >= self.lap_length:
            self._lap_start_distance = self._distance
            self.lap_wear = 0.0
        self.lap_wear = min(100.0, self.lap_wear + self.deg_rate * dt)

    def _publish(self):
        temps = Float32MultiArray()
        temps.data = [float(t) for t in self.temps]
        rpm = Float32MultiArray()
        wheel_omega = abs(self._act_vx) / self.wheel_radius
        wheel_rpm = wheel_omega * 60.0 / (2.0 * math.pi)
        # Tiny left/right split from yaw so the four values are not identical.
        rpm.data = [
            float(wheel_rpm * (1.0 + 0.02 * self._yawrate)),
            float(wheel_rpm * (1.0 - 0.02 * self._yawrate)),
            float(wheel_rpm * (1.0 + 0.015 * self._yawrate)),
            float(wheel_rpm * (1.0 - 0.015 * self._yawrate)),
        ]
        self.pub_temps.publish(temps)
        self.pub_rpm.publish(rpm)
        rate = Float32()
        rate.data = float(self.deg_rate)
        self.pub_rate.publish(rate)
        lap = Float32()
        lap.data = float(self.lap_wear)
        self.pub_lap.publish(lap)
        life = Float32()
        life.data = float(100.0 * sum(self.life) / 4.0)
        self.pub_life.publish(life)


def main(args=None):
    rclpy.init(args=args)
    node = TyreStatePublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
