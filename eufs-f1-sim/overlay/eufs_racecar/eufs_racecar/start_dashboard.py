"""Red/black EUFS F1 telemetry dashboard. Node on load_car.launch.py.

Start/Stop open gzclient+RViz against the existing gzserver and set physics
ONCE. Physics state is the last user action, not a /clock sample.
"""

import math
import os
import signal
import subprocess
import sys
import time
from collections import deque
from pathlib import Path

import yaml

from ackermann_msgs.msg import AckermannDriveStamped
from ament_index_python.packages import get_package_share_directory
from eufs_msgs.msg import CanState
from eufs_msgs.srv import SetCanState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from PyQt5.QtCore import QLineF, Qt, QTimer
from PyQt5.QtGui import QColor, QFont, QPainter, QPalette, QPen
from PyQt5.QtWidgets import (
    QApplication,
    QComboBox,
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSpinBox,
    QStyleFactory,
    QTabWidget,
    QTableWidget,
    QTableWidgetItem,
    QSizePolicy,
    QVBoxLayout,
    QWidget,
)
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import BatteryState, JointState
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float32, Float32MultiArray, Float64, String
from std_srvs.srv import Empty
from tf2_ros import Buffer, TransformListener

from .drive_contract import (
    BRAKE_S,
    DURATION_S,
    MAX_ACCEL_MPS2,
    MAX_SPEED_MPS,
    RATE_HZ,
    SimTimedDrive,
    braking_acceleration,
    forward_feedback_acceleration,
    telemetry_fresh,
)
from .telemetry_schema import ERS, Ego, Frame, Kinematics, Opponent, Tire
from .telemetry_geometry import SchemaTrackProjector
from .stack_manager import StackManager
from .rviz_grid import write_rviz_grid
from .track_select import available_tracks, resolve_track

# Keep the UI and telemetry projector in lockstep with the launch resolver.
# The fallback is useful for source checkouts where ament has not been built.
try:
    TRACKS = tuple(available_tracks())
except Exception:  # pragma: no cover - only applies before ROS package setup
    TRACKS = ('cota', 'small_track')
WHEEL_JOINTS = (
    'left_front_wheel_joint',
    'right_front_wheel_joint',
    'left_rear_wheel_joint',
    'right_rear_wheel_joint',
)
THROTTLE_SPEED_MPS = MAX_SPEED_MPS
THROTTLE_ACCEL_MPS2 = MAX_ACCEL_MPS2
THROTTLE_HZ = RATE_HZ
THROTTLE_S = DURATION_S
BRAKE_SETTLE_S = 0.3
BRAKE_TIMEOUT_S = 10.0

THEME = """
QWidget#demoRoot {
  background-color: #0b0b0c;
  color: #f3f3f3;
  font-size: 13px;
  font-family: "DejaVu Sans";
}
QLabel#title { color: #e10600; font-size: 22px; font-weight: 700; background: transparent; }
QLabel#section { color: #e10600; font-size: 13px; font-weight: 700; background: transparent; }
QLabel#key { color: #9a9a9a; background: transparent; }
QLabel#val { color: #ffffff; font-weight: 600; font-family: "DejaVu Sans Mono"; background: transparent; }
QLabel#badge { color: #ffffff; font-weight: 700; background: transparent; }
QLabel#status { color: #d0d0d0; background: transparent; }
QPushButton {
  background: #e10600; color: #fff; border: 0; padding: 8px 16px; font-weight: 700;
}
QPushButton:hover { background: #ff2a1f; }
QPushButton#stop { background: #2a2a2a; }
QPushButton#stop:hover { background: #444; }
QPushButton#throttle { background: #9b0000; }
QPushButton#throttle:hover { background: #c40000; }
QComboBox, QSpinBox {
  background: #161616; color: #fff; border: 1px solid #5a1010; padding: 4px 8px;
}
QFrame#card { background: #141416; border: 1px solid #3d0c0c; }
QTabWidget::pane { border: 1px solid #3d0c0c; background: #0f0f11; top: -1px; }
QTabBar::tab { background: #171719; color: #bcbcc2; border: 1px solid #3d0c0c; padding: 7px 14px; }
QTabBar::tab:selected { background: #e10600; color: #ffffff; }
QTableWidget { background: #141416; color: #f3f3f3; gridline-color: #3d3d42; border: 1px solid #3d0c0c; }
QHeaderView::section { background: #1d1d20; color: #d0d0d0; border: 0; padding: 5px; }
"""


def _pgrep(pattern):
    result = subprocess.run(
        ['pgrep', '-f', pattern],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        check=False,
    )
    return result.returncode == 0


def _fmt(value, unit='', digits=2):
    if value is None:
        return '—'
    try:
        if isinstance(value, float) and (math.isnan(value) or math.isinf(value)):
            return '—'
        return f'{value:.{digits}f}{unit}'
    except (TypeError, ValueError):
        return str(value)


def _finite(value):
    """Return a finite float or None for unavailable ROS measurements."""
    try:
        value = float(value)
    except (TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


class StartDashboard(Node):
    def __init__(self):
        super().__init__('start_dashboard')
        # Container defaults are supplied through environment variables so
        # ROS never infers an integer for the string-valued cars parameter.
        self.declare_parameter('track', os.environ.get('TRACK', 'cota'))
        self.declare_parameter('cars', os.environ.get('CARS', '1'))
        self.declare_parameter('namespace', 'eufs')
        self.declare_parameter('manage_stack', False)
        self.declare_parameter('vehicle_model', 'Ackermann')
        self.declare_parameter('rviz_config', '')
        self.declare_parameter('forgez_mode', 'Nominal')
        self.declare_parameter('forgez_T_core', '40.0')
        self.declare_parameter('forgez_E_lap', '220.0')
        self.declare_parameter('forgez_R_OT', '0.040')
        self.pause_cli = self.create_client(Empty, '/pause_physics')
        self.unpause_cli = self.create_client(Empty, '/unpause_physics')
        self.mission_cli = self.create_client(SetCanState, '/ros_can/set_mission')
        namespace = str(self.get_parameter('namespace').value).strip('/')
        ns = f'/{namespace}' if namespace else ''
        self.ack_topic = f'{ns}/cmd' if ns else '/cmd'
        self.ack_pub = self.create_publisher(AckermannDriveStamped, self.ack_topic, 10)
        self._gui_procs = []
        # Launch starts gzserver paused. Only Start / 10s throttle / Stop change this.
        self._physics_wanted = False
        self._managed_stack = bool(self.get_parameter('manage_stack').value)
        self.stack_manager = StackManager() if self._managed_stack else None
        self._managed_config = None
        self._shutdown_pending_clear = False

        self.cmd_vx = None
        self.act_vx = None
        self.steer_cmd = None
        self.accel_x = None
        self.pose_x = None
        self.pose_y = None
        self.yaw = None
        self.soc = None
        self.pack_temp = None
        self.current = None
        self.voltage = None
        self.charge_wh = None
        self.deploy_w = None
        self.signed_w = None
        self.lap_remain_wh = None
        self.derate = None
        self.wheel_rpm = {name: None for name in WHEEL_JOINTS}
        self._last_v = None
        self._last_v_t = None
        self._last_odom_wall = None
        self._odom_seq = 0
        self._mission_state = None
        self._last_battery_wall = None
        self.cell_soc = None
        self.cell_temps = None
        self._cell_temp_values = []
        self.tyre_temps = None
        self.tyre_deg_rate = None
        self.tyre_lap_deg = None
        self.tyre_life = None
        self.tyre_rpm = None
        self._last_tyre_wall = None
        self._throttle_active = False
        self._sim_time_s = None
        self._last_sim_time_s = None
        self._session_start_sim = None
        self._world_x = None
        self._world_y = None
        self._world_yaw = None
        self._act_vy = None
        self._yaw_rate = None
        self._accel_lat = None
        self._last_vy = None
        self._last_odom_yaw = None
        self._tf_ready = False
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._history = deque(maxlen=600)
        self._last_history_wall = None
        self._projector = None
        self._frenet_s = None
        self._frenet_y = None
        self._lap_number = None
        self._last_frenet_s = None
        self._frenet_forward_m = 0.0
        self._lap_wrap_armed = False
        self._opponents = {}
        self._opponent_subscriptions = []
        self._load_projector()

        # cmd_vel remains telemetry for the bridge/plant's effective target;
        # dashboard control itself is Ackermann-only on /eufs/cmd.
        cmd_vel_topic = f'{ns}/cmd_vel' if ns else '/cmd_vel'
        self.create_subscription(Twist, cmd_vel_topic, self._on_cmd, 10)
        clock_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Clock, '/clock', self._on_clock, clock_qos)
        self.create_subscription(CanState, '/ros_can/state', self._on_mission_state, 10)
        self.create_subscription(Odometry, f'{ns}/odom', self._on_odom, 10)
        self._configure_opponents(self.cars())
        self.create_subscription(JointState, f'{ns}/joint_states', self._on_joints, 10)
        self.create_subscription(
            BatteryState, f'{ns}/forgez/battery_state', self._on_battery, qos_profile_sensor_data,
        )
        self.create_subscription(Float64, f'{ns}/forgez/charge_level_wh', self._on_charge_wh, 10)
        self.create_subscription(Float64, f'{ns}/forgez/deploy_power_w', self._on_deploy, 10)
        self.create_subscription(Float64, f'{ns}/forgez/signed_power_w', self._on_signed, 10)
        self.create_subscription(
            Float64, f'{ns}/forgez/lap_energy_remaining_wh', self._on_lap_remain, 10,
        )
        self.create_subscription(String, f'{ns}/forgez/derate_reason', self._on_derate, 10)
        self.create_subscription(Float32MultiArray, f'{ns}/tyres/temps', self._on_tyre_temps, 10)
        self.create_subscription(Float32MultiArray, f'{ns}/tyres/wheel_rpm', self._on_tyre_rpm, 10)
        self.create_subscription(Float32, f'{ns}/tyres/degradation_rate', self._on_tyre_rate, 10)
        self.create_subscription(Float32, f'{ns}/tyres/lap_degradation', self._on_tyre_lap, 10)
        self.create_subscription(Float32, f'{ns}/tyres/life', self._on_tyre_life, 10)
        self.get_logger().info(
            f'drive topic: AckermannDriveStamped {self.ack_topic} (accel+steer); '
            f'cmd_vel is telemetry only'
        )
        if self.stack_manager is not None:
            self.stack_manager.start(self.track(), self.cars())

    def track(self):
        value = str(self.get_parameter('track').value).strip()
        return value if value in TRACKS else 'cota'

    def cars(self):
        try:
            return max(1, int(self.get_parameter('cars').value))
        except (TypeError, ValueError):
            return 1

    def _load_projector(self):
        try:
            assets = resolve_track(self.track())
            centerline = Path(assets['centerline'])
            self._projector = (
                SchemaTrackProjector(centerline) if centerline.is_file() else None
            )
        except (OSError, ValueError, RuntimeError, KeyError):
            self._projector = None

    @staticmethod
    def _new_opponent_state():
        return {
            'world_x': None, 'world_y': None, 'world_yaw': None,
            'vx': None, 'vy': None, 'yaw_rate': None,
            'accel_long': None, 'accel_lat': None,
            's': None, 'lateral': None, 'last_wall': None,
            'last_vx': None, 'last_vy': None, 'last_stamp': None,
        }

    def _configure_opponents(self, count=None):
        """Rebuild per-car odometry subscriptions for the selected car count."""
        for subscription in getattr(self, '_opponent_subscriptions', []):
            try:
                self.destroy_subscription(subscription)
            except Exception:  # noqa: BLE001 - tolerate shutdown races
                pass
        self._opponent_subscriptions = []
        namespace = str(self.get_parameter('namespace').value).strip('/')
        self._opponents = {}
        for index in range(1, max(1, int(count if count is not None else self.cars()))):
            opponent_id = f'{namespace}{index + 1}'
            self._opponents[opponent_id] = self._new_opponent_state()
            self._opponent_subscriptions.append(self.create_subscription(
                Odometry, f'/{opponent_id}/odom',
                self._opponent_callback(opponent_id), 10,
            ))

    def reset_session(self, track=None, cars=None):
        """Clear telemetry after a backend restart or before a new spawn."""
        # tf2 buffers retain dynamic transforms across a Gazebo restart. Clear
        # them before the new odometry stream arrives so old-session samples
        # cannot satisfy map→odom lookups during the first seconds of a run.
        try:
            self._tf_buffer.clear()
        except AttributeError:
            pass
        if track is not None:
            track = str(track).strip()
            if track in TRACKS:
                self.set_parameters([Parameter('track', Parameter.Type.STRING, track)])
        if cars is not None:
            self.set_parameters([Parameter('cars', Parameter.Type.STRING, str(max(1, int(cars))))])
        self._load_projector()
        self._configure_opponents(self.cars())
        self.cmd_vx = self.act_vx = self.accel_x = self.steer_cmd = None
        self.pose_x = self.pose_y = self.yaw = None
        self.soc = self.pack_temp = self.current = self.voltage = None
        self.charge_wh = self.deploy_w = self.signed_w = self.lap_remain_wh = None
        self.derate = None
        self.wheel_rpm = {name: None for name in WHEEL_JOINTS}
        self._last_v = self._last_vy = self._last_v_t = None
        self._last_odom_wall = self._last_battery_wall = self._last_tyre_wall = None
        self._odom_seq = 0
        self._mission_state = None
        self._throttle_active = False
        self.cell_soc = self.cell_temps = None
        self._cell_temp_values = []
        self.tyre_temps = self.tyre_deg_rate = self.tyre_lap_deg = self.tyre_life = None
        self.tyre_rpm = None
        self._sim_time_s = self._last_sim_time_s = self._session_start_sim = None
        self._world_x = self._world_y = self._world_yaw = None
        self._act_vy = self._yaw_rate = self._accel_lat = None
        self._tf_ready = False
        self._history.clear()
        self._last_history_wall = None
        self._frenet_s = self._frenet_y = self._last_frenet_s = None
        self._lap_number = None
        self._frenet_forward_m = 0.0
        self._lap_wrap_armed = False

    def configure_telemetry(self, track=None, cars=None):
        """Lifecycle hook: configure subscriptions and start a clean session."""
        self.reset_session(track=track, cars=cars)

    def forgez_mode(self):
        return str(self.get_parameter('forgez_mode').value)

    def forgez_t_core(self):
        return str(self.get_parameter('forgez_T_core').value)

    def forgez_e_lap(self):
        return str(self.get_parameter('forgez_E_lap').value)

    def forgez_r_ot(self):
        return str(self.get_parameter('forgez_R_OT').value)

    def physics_state(self):
        if self.stack_manager is not None and self.stack_manager.state == 'stopped':
            return 'OFF'
        return 'RUNNING' if self._physics_wanted else 'PAUSED'

    def managed_stack(self):
        return self.stack_manager is not None

    def tick_stack(self):
        """Advance managed process cleanup/restart without blocking Qt."""
        if self.stack_manager is not None:
            return self.stack_manager.tick()
        return None

    def shutdown_sim(self):
        """Begin asynchronous managed-stack cleanup when enabled."""
        self._physics_wanted = False
        self._throttle_active = False
        self.publish_stop_cmd()
        if self.stack_manager is not None:
            # Do not leave stale values visible while owned processes drain.
            self.reset_session()
            self._managed_config = None
            self._shutdown_pending_clear = True
            self.stack_manager.shutdown()
            return True
        result = self.stop_sim()
        self.reset_session()
        return result

    def _on_cmd(self, msg):
        self.cmd_vx = _finite(msg.linear.x)
        self.steer_cmd = _finite(msg.angular.z)

    def _on_clock(self, msg):
        new_time = msg.clock.sec + msg.clock.nanosec * 1e-9
        if self._sim_time_s is not None and new_time < self._sim_time_s:
            try:
                self._tf_buffer.clear()
            except AttributeError:
                pass
            self._session_start_sim = new_time
            self._last_v = self._last_vy = self._last_v_t = None
            self._last_frenet_s = self._frenet_s = self._frenet_y = None
            self._lap_number = None
            self._frenet_forward_m = 0.0
            self._lap_wrap_armed = False
        self._last_sim_time_s = self._sim_time_s
        self._sim_time_s = new_time
        if self._session_start_sim is None:
            self._session_start_sim = self._sim_time_s

    def _on_mission_state(self, msg):
        self._mission_state = msg

    def odom_fresh(self, max_age=1.0):
        return telemetry_fresh(self._last_odom_wall, time.monotonic(), max_age)

    def mission_state_ready(self):
        if self._mission_state is None:
            return False
        # Manual is source-supported and can drive immediately; all other
        # missions must reach AS_DRIVING through the traditional handshake.
        return (
            self._mission_state.ami_state == CanState.AMI_MANUAL
            or self._mission_state.as_state == CanState.AS_DRIVING
        )

    def request_manual_mission(self, timeout=4.0):
        """Select source-supported manual mode when the traditional gate exists.

        The authored Ackermann plant has no /ros_can service or state topic, so
        absence of both is explicitly treated as a valid alternate path.
        """
        if not self.mission_cli.wait_for_service(timeout_sec=0.5):
            state_publishers = self.count_publishers('/ros_can/state')
            if state_publishers:
                return False
            return str(self.get_parameter('vehicle_model').value) == 'Ackermann'
        request = SetCanState.Request()
        request.ami_state = CanState.AMI_MANUAL
        future = self.mission_cli.call_async(request)
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if future.done():
                result = future.result()
                if result is None or not result.success:
                    return False
                break
        else:
            return False
        state_deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < state_deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.mission_state_ready():
                return True
        return False

    def sim_time(self):
        return self._sim_time_s

    def command_ready(self):
        """Return true once the active vehicle plant subscribes to /eufs/cmd."""
        return self.ack_pub.get_subscription_count() > 0 and self.odom_fresh()

    def _on_odom(self, msg):
        self._last_odom_wall = time.monotonic()
        self._odom_seq += 1
        self.act_vx = _finite(msg.twist.twist.linear.x)
        self._act_vy = _finite(msg.twist.twist.linear.y)
        self._yaw_rate = _finite(msg.twist.twist.angular.z)
        self.pose_x = _finite(msg.pose.pose.position.x)
        self.pose_y = _finite(msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        quaternion = tuple(_finite(value) for value in (q.w, q.x, q.y, q.z))
        if all(value is not None for value in quaternion):
            siny = 2.0 * (q.w * q.z + q.x * q.y)
            cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            self.yaw = math.atan2(siny, cosy)
        else:
            self.yaw = None
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        stamp = _finite(stamp)
        dt = (stamp - self._last_v_t
              if stamp is not None and self._last_v_t is not None else None)
        if (dt is not None and 1e-4 < dt <= 1.0 and self._last_v is not None
                and self._last_vy is not None and self.act_vx is not None
                and self._act_vy is not None and self._yaw_rate is not None):
            # Convert body-frame velocity derivatives to body acceleration.
            self.accel_x = ((self.act_vx - self._last_v) / dt
                            - self._yaw_rate * self._act_vy)
            self._accel_lat = ((self._act_vy - self._last_vy) / dt
                               + self._yaw_rate * self.act_vx)
        else:
            self.accel_x = None
            self._accel_lat = None
        self._last_v = self.act_vx
        self._last_vy = self._act_vy
        self._last_v_t = stamp if stamp is not None else None
        if (self.act_vx is None or self._act_vy is None or self.yaw is None
                or self.pose_x is None or self.pose_y is None):
            self._tf_ready = False
            self._world_x = self._world_y = self._world_yaw = None
        else:
            self._update_world_pose(msg)

    def _opponent_callback(self, opponent_id):
        def callback(msg):
            self._on_opponent_odom(opponent_id, msg)
        return callback

    def _on_opponent_odom(self, opponent_id, msg):
        state = self._opponents[opponent_id]
        values = tuple(_finite(value) for value in (
            msg.pose.pose.position.x, msg.pose.pose.position.y,
            msg.twist.twist.linear.x, msg.twist.twist.linear.y,
            msg.twist.twist.angular.z,
        ))
        if any(value is None for value in values):
            state.update(world_x=None, world_y=None, world_yaw=None,
                         vx=None, vy=None, yaw_rate=None, accel_long=None,
                         accel_lat=None, s=None, lateral=None,
                         last_wall=time.monotonic())
            return
        state['last_wall'] = time.monotonic()
        state['vx'], state['vy'], state['yaw_rate'] = values[2:]
        stamp = _finite(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
        dt = (stamp - state['last_stamp']
              if stamp is not None and state['last_stamp'] is not None else None)
        if (dt is not None and 1e-4 < dt <= 1.0
                and state['last_vx'] is not None and state['last_vy'] is not None):
            state['accel_long'] = ((state['vx'] - state['last_vx']) / dt
                                   - state['yaw_rate'] * state['vy'])
            state['accel_lat'] = ((state['vy'] - state['last_vy']) / dt
                                  + state['yaw_rate'] * state['vx'])
        else:
            state['accel_long'] = state['accel_lat'] = None
        state['last_vx'], state['last_vy'], state['last_stamp'] = (
            state['vx'], state['vy'], stamp,
        )
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        quaternion = tuple(_finite(value) for value in (q.w, q.x, q.y, q.z))
        if any(value is None for value in quaternion):
            state.update(world_x=None, world_y=None, world_yaw=None, s=None, lateral=None)
            return
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        source = msg.header.frame_id or 'odom'
        try:
            transform = self._tf_buffer.lookup_transform('map', source, rclpy.time.Time())
            t = transform.transform.translation
            tq = transform.transform.rotation
            if any(_finite(value) is None for value in (t.x, t.y, tq.w, tq.x, tq.y, tq.z)):
                raise ValueError('non-finite map transform')
            tf_yaw = math.atan2(2.0 * (tq.w * tq.z + tq.x * tq.y),
                                1.0 - 2.0 * (tq.y * tq.y + tq.z * tq.z))
            c, s = math.cos(tf_yaw), math.sin(tf_yaw)
            state['world_x'] = t.x + c * p.x - s * p.y
            state['world_y'] = t.y + s * p.x + c * p.y
            state['world_yaw'] = math.atan2(math.sin(tf_yaw + yaw),
                                            math.cos(tf_yaw + yaw))
        except Exception:
            state.update(world_x=None, world_y=None, world_yaw=None, s=None, lateral=None)

    def update_derived_telemetry(self):
        """Update expensive projection at UI rate rather than odom rate."""
        if self._projector is not None and self._world_x is not None and self._world_y is not None:
            try:
                s, lateral = self._projector.project(self._world_x, self._world_y,
                                                     self._last_frenet_s)
                continuity_reset = False
                if self._last_frenet_s is None:
                    self._lap_number = 1
                else:
                    raw_delta = s - self._last_frenet_s
                    if raw_delta < -0.5 * self._projector.length:
                        delta = raw_delta + self._projector.length
                        if self._lap_wrap_armed:
                            self._lap_number = (self._lap_number or 1) + 1
                    elif raw_delta > 0.5 * self._projector.length:
                        delta = raw_delta - self._projector.length
                    else:
                        delta = raw_delta
                    if delta < -25.0 or delta > 25.0:
                        # A backend reset/teleport invalidates lap continuity.
                        self._lap_number = None
                        self._frenet_forward_m = 0.0
                        self._lap_wrap_armed = False
                        continuity_reset = True
                    elif delta > 0.0:
                        self._frenet_forward_m += delta
                        if self._frenet_forward_m >= 100.0:
                            self._lap_wrap_armed = True
                self._frenet_s, self._frenet_y = s, lateral
                self._last_frenet_s = None if continuity_reset else s
            except (TypeError, ValueError, ZeroDivisionError):
                self._frenet_s = self._frenet_y = self._lap_number = None
                self._last_frenet_s = None
            for state in self._opponents.values():
                if (state['world_x'] is not None and state['world_y'] is not None
                        and state['last_wall'] is not None
                        and time.monotonic() - state['last_wall'] <= 1.0):
                    try:
                        state['s'], state['lateral'] = self._projector.project(
                            state['world_x'], state['world_y'], state['s'])
                    except (TypeError, ValueError, ZeroDivisionError):
                        state['s'] = state['lateral'] = None
                else:
                    state['s'] = state['lateral'] = None
        self._record_history()

    def _update_world_pose(self, msg):
        """Apply the spawn's map→odom transform to each odometry sample."""
        source = msg.header.frame_id or 'odom'
        try:
            transform = self._tf_buffer.lookup_transform('map', source, rclpy.time.Time())
            t = transform.transform.translation
            q = transform.transform.rotation
            if any(_finite(value) is None for value in (
                    t.x, t.y, q.w, q.x, q.y, q.z)):
                raise ValueError('non-finite map transform')
            tf_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            c, s = math.cos(tf_yaw), math.sin(tf_yaw)
            self._world_x = t.x + c * self.pose_x - s * self.pose_y
            self._world_y = t.y + s * self.pose_x + c * self.pose_y
            self._world_yaw = math.atan2(math.sin(tf_yaw + self.yaw),
                                         math.cos(tf_yaw + self.yaw))
            self._tf_ready = True
        except Exception:
            self._tf_ready = False
            self._world_x = self._world_y = self._world_yaw = None

    def _record_history(self):
        now = time.monotonic()
        if self._last_history_wall is not None and now - self._last_history_wall < 0.09:
            return
        self._last_history_wall = now
        frame = self.telemetry_frame()
        ers_power = None
        if frame.ego.ers.voltage_v is not None and frame.ego.ers.current_demand_a is not None:
            ers_power = frame.ego.ers.voltage_v * frame.ego.ers.current_demand_a / 1000.0
        self._history.append((
            frame.timestamp,
            frame.ego.kinematics.speed_kmh,
            frame.ego.kinematics.accel_long_g,
            frame.ego.ers.soc_pct,
            ers_power,
        ))

    @staticmethod
    def _percent(value):
        value = _finite(value)
        if value is None:
            return None
        return value * 100.0

    def telemetry_frame(self) -> Frame:
        """Return the current schema frame with unavailable values as None."""
        battery_live = self.battery_fresh()
        tyres_live = self.tyres_fresh()
        odom_live = self.odom_fresh()
        projection_live = (odom_live and self._tf_ready
                           and self._world_x is not None and self._world_y is not None)
        cell_temps = list(self._cell_temp_values or []) if battery_live else []
        cell_temps = [value for value in (_finite(value) for value in cell_temps)
                      if value is not None]
        pack_temp = _finite(self.pack_temp) if battery_live else None
        voltage = _finite(self.voltage) if battery_live else None
        current = _finite(self.current) if battery_live else None
        ers = ERS(
            soc_pct=self._percent(self.soc) if battery_live else None,
            soh_pct=None,
            pack_temp_c=pack_temp,
            max_cell_temp_c=max(cell_temps) if cell_temps else None,
            # BatteryState.current is signed.  Dashboard demand follows the
            # explicit source convention current_demand = -current.
            current_demand_a=-current if current is not None else None,
            voltage_v=voltage,
            mguk_power_kw=None,
            lap_deployed_mj=None,
        )
        if not odom_live:
            kin = Kinematics()
        else:
            speed = (math.hypot(self.act_vx, self._act_vy) * 3.6
                     if self.act_vx is not None and self._act_vy is not None else None)
            kin = Kinematics(
                s=self._frenet_s if projection_live else None,
                y=self._frenet_y if projection_live else None,
                x_world=_finite(self._world_x),
                y_world=_finite(self._world_y),
                heading_rad=_finite(self._world_yaw),
                speed_kmh=speed,
                accel_long_g=(_finite(self.accel_x) / 9.80665
                              if _finite(self.accel_x) is not None else None),
                accel_lat_g=(_finite(self._accel_lat) / 9.80665
                             if _finite(self._accel_lat) is not None else None),
                yaw_rate_rads=_finite(self._yaw_rate),
            )
        temps = self.tyre_temps or [] if tyres_live else []
        corners = ('FL', 'FR', 'RL', 'RR')
        temp_values = [_finite(value) for value in temps]
        life = _finite(self.tyre_life) if tyres_live else None
        tires = Tire(
            wear_pct=(100.0 - life) if life is not None else None,
            # The custom publisher exposes percent/second, not percent/lap.
            deg_rate_pct_per_lap=None,
            surface_temp_c={corner: (temp_values[index] if index < len(temp_values) else None)
                            for index, corner in enumerate(corners)},
            carcass_temp_c={corner: None for corner in corners},
        )
        ego = Ego(
            session_time=(_finite(self._sim_time_s - self._session_start_sim)
                          if self._sim_time_s is not None and self._session_start_sim is not None
                          else None),
            lap_number=self._lap_number,
            track_status=self.physics_state(),
            kinematics=kin,
            ers=ers,
            tires=tires,
            throttle_pct=None,
            brake_pressure_bar=None,
            gear=None,
            drs_active=None,
            fuel_remaining_kg=None,
        )
        opponents = {}
        for opponent_id, state in self._opponents.items():
            opponent_live = (state['last_wall'] is not None
                              and time.monotonic() - state['last_wall'] <= 1.0)
            if (not opponent_live or state['s'] is None or self._frenet_s is None
                    or self._projector is None or not odom_live):
                gap_m = gap_s = speed_gap = None
            else:
                gap_m = self._projector.signed_shortest_gap(
                    state['s'], self._frenet_s, self._projector.length,
                )
                ego_speed = (math.hypot(self.act_vx, self._act_vy)
                             if self.act_vx is not None and self._act_vy is not None else None)
                gap_s = gap_m / ego_speed if ego_speed is not None and ego_speed > 0.5 else None
                other_speed = (math.hypot(state['vx'], state['vy'])
                               if state['vx'] is not None and state['vy'] is not None else None)
                speed_gap = ((ego_speed - other_speed) * 3.6
                             if ego_speed is not None and other_speed is not None else None)
            opponent_kin = Kinematics()
            if opponent_live:
                other_speed = (math.hypot(state['vx'], state['vy']) * 3.6
                               if state['vx'] is not None and state['vy'] is not None else None)
                opponent_kin = Kinematics(
                    s=state['s'],
                    y=state['lateral'],
                    x_world=_finite(state['world_x']),
                    y_world=_finite(state['world_y']),
                    heading_rad=_finite(state['world_yaw']),
                    speed_kmh=other_speed,
                    accel_long_g=(_finite(state['accel_long']) / 9.80665
                                  if _finite(state['accel_long']) is not None else None),
                    accel_lat_g=(_finite(state['accel_lat']) / 9.80665
                                 if _finite(state['accel_lat']) is not None else None),
                    yaw_rate_rads=_finite(state['yaw_rate']),
                )
            opponents[opponent_id] = Opponent(
                car_id=opponent_id,
                kinematics=opponent_kin,
                gap_to_ego_m=gap_m,
                gap_to_ego_s=gap_s,
                speed_gap_kmh=speed_gap,
            )
        return Frame(timestamp=self._sim_time_s, ego=ego, opponents=opponents)

    def _on_joints(self, msg):
        velocities = list(msg.velocity) if msg.velocity else []
        for index, name in enumerate(msg.name):
            if name not in self.wheel_rpm or index >= len(velocities):
                continue
            velocity = _finite(velocities[index])
            self.wheel_rpm[name] = (
                abs(velocity) * 60.0 / (2.0 * math.pi) if velocity is not None else None
            )

    def _on_battery(self, msg):
        self._last_battery_wall = time.monotonic()
        self.voltage = _finite(msg.voltage)
        self.current = _finite(msg.current)
        self.soc = _finite(msg.percentage)
        self.pack_temp = _finite(msg.temperature)
        cell_pct = list(getattr(msg, 'cell_percentage', None) or [])
        self.cell_soc = (
            ', '.join(f'{p * 100.0:.1f}%'
                      for p in (_finite(value) for value in cell_pct) if p is not None)
            if cell_pct else None
        )
        cell_temps = list(getattr(msg, 'cell_temperature', None) or [])
        self._cell_temp_values = [_finite(value) for value in cell_temps]
        self.cell_temps = (
            ', '.join(f'{t:.1f}°C' for t in self._cell_temp_values if t is not None)
            if cell_temps else None
        )

    def _on_charge_wh(self, msg):
        self.charge_wh = _finite(msg.data)

    def _on_deploy(self, msg):
        self.deploy_w = _finite(msg.data)

    def _on_signed(self, msg):
        self.signed_w = _finite(msg.data)

    def _on_lap_remain(self, msg):
        self.lap_remain_wh = _finite(msg.data)

    def _on_derate(self, msg):
        self.derate = msg.data

    def _on_tyre_temps(self, msg):
        self._last_tyre_wall = time.monotonic()
        self.tyre_temps = [_finite(value) for value in msg.data]

    def _on_tyre_rpm(self, msg):
        self._last_tyre_wall = time.monotonic()
        self.tyre_rpm = [_finite(value) for value in msg.data]

    def _on_tyre_rate(self, msg):
        self._last_tyre_wall = time.monotonic()
        self.tyre_deg_rate = _finite(msg.data)

    def _on_tyre_lap(self, msg):
        self._last_tyre_wall = time.monotonic()
        self.tyre_lap_deg = _finite(msg.data)

    def _on_tyre_life(self, msg):
        self._last_tyre_wall = time.monotonic()
        self.tyre_life = _finite(msg.data)

    def battery_fresh(self, max_age=1.0):
        return (self._last_battery_wall is not None and
                time.monotonic() - self._last_battery_wall <= max_age)

    def tyres_fresh(self, max_age=1.0):
        return (self._last_tyre_wall is not None and
                time.monotonic() - self._last_tyre_wall <= max_age)

    def _gui_env(self):
        env = os.environ.copy()
        env['DISPLAY'] = env.get('DISPLAY') or ':0'
        env.setdefault('QT_X11_NO_MITSHM', '1')
        env.setdefault('LIBGL_DRI3_DISABLE', '1')
        return env

    def _rviz_config(self):
        configured = str(self.get_parameter('rviz_config').value).strip()
        if configured:
            return configured
        return os.path.join(
            get_package_share_directory('eufs_racecar'), 'config', 'eufs_f1.rviz',
        )

    def _managed_rviz_config(self, cars):
        """Create a per-run RViz grid while preserving the base displays."""
        base_path = Path(self._rviz_config())
        output = Path('/tmp') / f'eufs_rviz_grid_{int(cars)}.rviz'
        try:
            base = yaml.safe_load(base_path.read_text(encoding='utf-8'))
            write_rviz_grid(output, int(cars), self._namespace(), base, resolve_track(self.track()))
            return str(output)
        except (OSError, TypeError, ValueError, yaml.YAMLError) as exc:
            self.get_logger().warn(f'could not generate RViz grid: {exc}')
            return str(base_path)

    def _namespace(self):
        return str(self.get_parameter('namespace').value).strip('/') or 'eufs'

    def _start_guis(self):
        env = self._gui_env()
        if not _pgrep('gzclient'):
            # No extra GUI plugins: libgazebo_ros_eol_gui can abort gzserver
            # if the client attaches while COTA is still loading.
            proc = subprocess.Popen(
                ['gzclient'],
                env=env,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            self._gui_procs.append(proc)
            self.get_logger().info(f'gzclient pid={proc.pid} DISPLAY={env["DISPLAY"]}')
        if not _pgrep('rviz2'):
            proc = subprocess.Popen(
                [
                    'rviz2', '-d', self._rviz_config(),
                    '--ros-args', '-p', 'use_sim_time:=true', '-r', '__node:=rviz',
                ],
                env=env,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            self._gui_procs.append(proc)
            self.get_logger().info(f'rviz2 pid={proc.pid} DISPLAY={env["DISPLAY"]}')

    def physics_services_ready(self):
        return self.unpause_cli.service_is_ready() and self.pause_cli.service_is_ready()

    def spawn_finished(self):
        return not _pgrep('spawn_entity.py')

    def gazebo_ready(self):
        if self.stack_manager is not None and self.stack_manager.state != 'running':
            return False
        if not (_pgrep('gzserver') and self.physics_services_ready() and self.spawn_finished()):
            return False
        # A completed spawn process alone can race the ROS graph. Require the
        # command bridge and every selected car's odometry publisher first.
        if self.count_subscribers(self.ack_topic) < 1:
            return False
        namespace = self._namespace()
        return all(
            self.count_publishers(f'/{namespace if index == 1 else namespace + str(index)}/odom') >= 1
            for index in range(1, self.cars() + 1)
        )

    def _call_empty(self, client):
        if not client.wait_for_service(timeout_sec=0.5):
            self.get_logger().warn(f'{client.srv_name} is not ready')
            return False
        client.call_async(Empty.Request())
        return True

    def publish_forward_cmd(self):
        self.cmd_vx = THROTTLE_SPEED_MPS
        ack = AckermannDriveStamped()
        ack.header.stamp = self.get_clock().now().to_msg()
        ack.drive.steering_angle = 0.0
        ack.drive.acceleration = forward_feedback_acceleration(self.act_vx, THROTTLE_SPEED_MPS)
        ack.drive.speed = THROTTLE_SPEED_MPS
        self.ack_pub.publish(ack)

    def publish_stop_cmd(self):
        self.cmd_vx = 0.0
        ack = AckermannDriveStamped()
        ack.header.stamp = self.get_clock().now().to_msg()
        ack.drive.steering_angle = 0.0
        ack.drive.acceleration = 0.0
        ack.drive.speed = 0.0
        self.ack_pub.publish(ack)

    def publish_brake_cmd(self):
        """Request a bounded negative acceleration in acceleration-mode plants."""
        ack = AckermannDriveStamped()
        ack.header.stamp = self.get_clock().now().to_msg()
        ack.drive.steering_angle = 0.0
        velocity = self.act_vx or 0.0
        ack.drive.acceleration = braking_acceleration(velocity)
        ack.drive.speed = 0.0
        self.ack_pub.publish(ack)

    def start_sim(self):
        if self.stack_manager is not None:
            if not self.gazebo_ready():
                self.get_logger().warn('managed EUFS stack is not ready')
                return False
            self._physics_wanted = True
            unpaused = self._call_empty(self.unpause_cli)
            viewers = self.stack_manager.open_viewers(
                self._managed_rviz_config(self.cars())
            )
            return unpaused and viewers
        if not self.gazebo_ready():
            self.get_logger().warn('gzserver not ready; not starting gzclient')
            return False
        # Unpause first so /clock and odom→base_link TF exist before RViz.
        self._physics_wanted = True
        unpaused = self._call_empty(self.unpause_cli)
        self._start_guis()
        return unpaused

    def stop_sim(self):
        self._throttle_active = False
        self.publish_stop_cmd()
        self._physics_wanted = False
        return self._call_empty(self.pause_cli)

    def ensure_running(self):
        if not self.gazebo_ready():
            self.get_logger().warn('Gazebo is not ready; drive request rejected')
            return False
        if self._physics_wanted:
            return True
        self._physics_wanted = True
        return self._call_empty(self.unpause_cli)


def _card():
    frame = QFrame()
    frame.setObjectName('card')
    frame.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Maximum)
    layout = QGridLayout(frame)
    layout.setContentsMargins(12, 10, 12, 10)
    layout.setHorizontalSpacing(18)
    layout.setVerticalSpacing(8)
    return frame, layout


def _kv(layout, row, col, key):
    k = QLabel(key)
    k.setObjectName('key')
    k.setTextFormat(Qt.PlainText)
    k.setWordWrap(False)
    v = QLabel('—')
    v.setObjectName('val')
    v.setTextFormat(Qt.PlainText)
    v.setWordWrap(False)
    v.setMinimumWidth(140)
    layout.addWidget(k, row, col * 2)
    layout.addWidget(v, row, col * 2 + 1)
    return v


class HistoryPlot(QWidget):
    """Small dependency-free live plot for speed, acceleration, SOC, and power."""

    def __init__(self, node):
        super().__init__()
        self.node = node
        self.setMinimumHeight(190)
        self.setObjectName('historyPlot')

    def paintEvent(self, _event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.fillRect(self.rect(), QColor('#101012'))
        margin = 28
        plot = self.rect().adjusted(margin, 14, -12, -24)
        painter.setPen(QPen(QColor('#3d3d42'), 1))
        painter.drawRect(plot)
        points = list(self.node._history)
        if len(points) < 2:
            painter.setPen(QColor('#8f8f95'))
            painter.drawText(plot, Qt.AlignCenter, 'Waiting for telemetry')
            return

        series = (
            ('Speed: 0–120 km/h', 1, QColor('#f2c14e'), 120.0, False),
            ('Long accel: −2…+2 g', 2, QColor('#4ecdc4'), 2.0, True),
            ('SOC: 0–100%', 3, QColor('#e10600'), 100.0, False),
            ('Pack power: −100…+100 kW', 4, QColor('#a78bfa'), 100.0, True),
        )
        for label, index, color, maximum, signed in series:
            values = [row[index] if len(row) > index else None for row in points]
            if not any(value is not None for value in values):
                continue
            pen = QPen(color, 2)
            painter.setPen(pen)
            previous = None
            for offset, value in enumerate(values):
                if value is None:
                    previous = None
                    continue
                x = plot.left() + plot.width() * offset / max(1, len(values) - 1)
                normalized = (
                    0.5 + float(value) / (2.0 * maximum)
                    if signed else float(value) / maximum
                )
                normalized = max(0.0, min(1.0, normalized))
                y = plot.bottom() - plot.height() * normalized
                current = (x, y)
                if previous is not None:
                    painter.drawLine(QLineF(previous[0], previous[1], current[0], current[1]))
                previous = current
            painter.drawText(plot.left() + 6, plot.top() + 16 + series.index(
                (label, index, color, maximum, signed)
            ) * 16, label)


class DemoWindow(QWidget):
    def __init__(self, node: StartDashboard):
        super().__init__()
        self.node = node
        self.setObjectName('demoRoot')
        self.setWindowTitle('EUFS F1 Demo')
        self.setStyleSheet(THEME)
        self.setAutoFillBackground(True)
        self.setMinimumSize(960, 720)

        title = QLabel('EUFS F1')
        title.setObjectName('title')
        title.setTextFormat(Qt.PlainText)

        track_lbl = QLabel('Track')
        track_lbl.setObjectName('key')
        track_lbl.setTextFormat(Qt.PlainText)
        cars_lbl = QLabel('Cars')
        cars_lbl.setObjectName('key')
        cars_lbl.setTextFormat(Qt.PlainText)

        self.physics = QLabel('PAUSED')
        self.physics.setObjectName('badge')
        self.physics.setTextFormat(Qt.PlainText)

        self.track = QComboBox()
        self.track.addItems(TRACKS)
        self.track.setCurrentText(node.track())
        self.cars = QSpinBox()
        self.cars.setMinimum(1)
        self.cars.setMaximum(20)
        self.cars.setValue(min(20, node.cars()))
        self.track.currentTextChanged.connect(self._on_selection_changed)
        self.cars.valueChanged.connect(self._on_selection_changed)

        start = QPushButton('Start')
        self.start_btn = start
        start.setEnabled(False)
        stop = QPushButton('Stop')
        self.stop_btn = stop
        stop.setObjectName('stop')
        shutdown = QPushButton('Shutdown')
        self.shutdown_btn = shutdown
        shutdown.setObjectName('stop')
        throttle = QPushButton('10s front throttle')
        throttle.setObjectName('throttle')
        start.clicked.connect(self._on_start)
        stop.clicked.connect(self._on_stop)
        shutdown.clicked.connect(self._on_shutdown)
        throttle.clicked.connect(self._on_throttle)

        header = QHBoxLayout()
        header.setSpacing(10)
        header.addWidget(title)
        header.addStretch(1)
        header.addWidget(track_lbl)
        header.addWidget(self.track)
        header.addWidget(cars_lbl)
        header.addWidget(self.cars)
        header.addWidget(self.physics)
        header.addWidget(start)
        header.addWidget(stop)
        header.addWidget(shutdown)
        header.addWidget(throttle)

        motion, mgrid = _card()
        sec_m = QLabel('MOTION')
        sec_m.setObjectName('section')
        sec_m.setTextFormat(Qt.PlainText)
        self.cmd_v = _kv(mgrid, 1, 0, 'Commanded vx')
        self.act_v = _kv(mgrid, 1, 1, 'Actual vx')
        self.accel = _kv(mgrid, 2, 0, 'Acceleration')
        self.speed_kmh = _kv(mgrid, 2, 1, 'Speed')
        self.pose = _kv(mgrid, 3, 0, 'Pose x, y')
        self.heading = _kv(mgrid, 3, 1, 'Heading')
        self.s_coord = _kv(mgrid, 4, 0, 'Track s')
        self.lateral = _kv(mgrid, 4, 1, 'Track y')
        self.accel_long_g = _kv(mgrid, 5, 0, 'Long accel')
        self.accel_lat_g = _kv(mgrid, 5, 1, 'Lat accel')
        self.yaw_rate = _kv(mgrid, 6, 0, 'Yaw rate')
        self.lap = _kv(mgrid, 6, 1, 'Lap')
        self.session = _kv(mgrid, 7, 0, 'Session time')
        self.track_status = _kv(mgrid, 7, 1, 'Track status')
        self.steer = _kv(mgrid, 8, 0, 'Steer cmd')
        mgrid.addWidget(sec_m, 0, 0, 1, 4)

        batt, bgrid = _card()
        sec_b = QLabel('ERS / PACK')
        sec_b.setObjectName('section')
        sec_b.setTextFormat(Qt.PlainText)
        self.cell_soc = _kv(bgrid, 1, 0, 'Cell SOC')
        self.batt_temps = _kv(bgrid, 1, 1, 'Pack temp')
        self.batt_soc = _kv(bgrid, 2, 0, 'Battery SOC')
        self.current = _kv(bgrid, 2, 1, 'Current Demand')
        self.max_cell_temp = _kv(bgrid, 3, 0, 'Max cell temp')
        self.voltage = _kv(bgrid, 3, 1, 'Voltage')
        self.pack_power = _kv(bgrid, 4, 0, 'Pack power (V×−I)')
        self.mode = _kv(bgrid, 4, 1, 'Forgez mode')
        self.t_core = _kv(bgrid, 5, 0, 'T_core (target)')
        self.e_lap = _kv(bgrid, 5, 1, 'E_lap')
        self.r_ot = _kv(bgrid, 6, 0, 'R_OT')
        self.energy = _kv(bgrid, 6, 1, 'Charge / lap left')
        self.derate = _kv(bgrid, 7, 0, 'Derate')
        self.cell_temps = _kv(bgrid, 8, 0, 'Cell temps')
        self.unsupported_ers = _kv(bgrid, 8, 1, 'SOH / MGUK / deploy')
        self.unsupported_ers.setToolTip(
            'Unavailable: no authoritative SOH, MGUK power, or lap deployment sensor'
        )
        batt.setToolTip(
            'Available ERS values come from BatteryState and Forgez telemetry. '
            'SOH, MGUK power, and lap deployment have no source.'
        )
        bgrid.addWidget(sec_b, 0, 0, 1, 4)

        tyres, tgrid = _card()
        sec_t = QLabel('TYRES')
        sec_t.setObjectName('section')
        sec_t.setTextFormat(Qt.PlainText)
        self.tyre_temp = _kv(tgrid, 1, 0, 'Tire temps (model estimate)')
        self.tyre_rate = _kv(tgrid, 1, 1, 'Degradation rate')
        self.tyre_lap = _kv(tgrid, 2, 0, 'Lap wear (accumulated %)')
        self.tyre_life = _kv(tgrid, 2, 1, 'Wear')
        self.temp_fl = _kv(tgrid, 3, 0, 'FL temp')
        self.temp_fr = _kv(tgrid, 3, 1, 'FR temp')
        self.temp_rl = _kv(tgrid, 4, 0, 'RL temp')
        self.temp_rr = _kv(tgrid, 4, 1, 'RR temp')
        self.fl = _kv(tgrid, 5, 0, 'FL rpm')
        self.fr = _kv(tgrid, 5, 1, 'FR rpm')
        self.rl = _kv(tgrid, 6, 0, 'RL rpm')
        self.rr = _kv(tgrid, 6, 1, 'RR rpm')
        tyres.setToolTip(
            'Temperatures are the custom tyre model estimate. Surface values are '
            'shown when available; carcass temperature has no source. '
            'Degradation rate is percent per second; lap wear is accumulated percent.'
        )
        tgrid.addWidget(sec_t, 0, 0, 1, 4)

        opponents, ogrid = _card()
        sec_o = QLabel('OPPONENTS')
        sec_o.setObjectName('section')
        sec_o.setTextFormat(Qt.PlainText)
        self.opponent_summary = QTableWidget(0, 7)
        self.opponent_summary.setHorizontalHeaderLabels(
            ('ID', 'Speed', 'Gap', 'Gap time', 'World X', 'World Y', 'Lateral')
        )
        self.opponent_summary.setObjectName('opponentTable')
        self.opponent_summary.setEditTriggers(QTableWidget.NoEditTriggers)
        self.opponent_summary.setSelectionMode(QTableWidget.NoSelection)
        self.opponent_summary.setFocusPolicy(Qt.NoFocus)
        self.opponent_summary.verticalHeader().setVisible(False)
        self.opponent_summary.horizontalHeader().setStretchLastSection(True)
        self.opponent_summary.setMinimumHeight(140)
        ogrid.addWidget(sec_o, 0, 0, 1, 4)
        ogrid.addWidget(self.opponent_summary, 1, 0, 1, 4)

        motion_page = QWidget()
        motion_layout = QVBoxLayout(motion_page)
        motion_layout.setAlignment(Qt.AlignTop)
        motion_layout.addWidget(motion)
        self.history_plot = HistoryPlot(node)
        motion_layout.addWidget(self.history_plot)
        ers_page = QWidget()
        ers_layout = QVBoxLayout(ers_page)
        ers_layout.setAlignment(Qt.AlignTop)
        ers_layout.addWidget(batt)
        tyres_page = QWidget()
        tyres_layout = QVBoxLayout(tyres_page)
        tyres_layout.setAlignment(Qt.AlignTop)
        tyres_layout.addWidget(tyres)
        opponents_page = QWidget()
        opponents_layout = QVBoxLayout(opponents_page)
        opponents_layout.setAlignment(Qt.AlignTop)
        opponents_layout.addWidget(opponents)
        self.tabs = QTabWidget()
        self.tabs.addTab(motion_page, 'Motion')
        self.tabs.addTab(ers_page, 'ERS / Pack')
        self.tabs.addTab(tyres_page, 'Tyres')
        self.tabs.addTab(opponents_page, 'Opponents')

        self.status = QLabel(
            f'Loading {node.track()} in gzserver. Start enables after the car spawns — '
            'do not open Gazebo/RViz until then.'
        )
        self.status.setObjectName('status')
        self.status.setWordWrap(True)

        form = QVBoxLayout()
        form.setSpacing(12)
        form.setAlignment(Qt.AlignTop)
        form.addLayout(header)
        form.addWidget(self.tabs)
        form.addWidget(self.status)
        self.setLayout(form)

        self._drive = SimTimedDrive()
        self._drive_odom_seq = 0
        self._brake_started_wall = None
        self._brake_settle_started_wall = None
        self._braking = False
        self._brake_ticks_left = 0
        self._drive_timer = QTimer(self)
        self._drive_timer.timeout.connect(self._drive_tick)
        self._pending_autostart = False

        self._ready_timer = QTimer(self)
        self._ready_timer.timeout.connect(self._poll_gazebo_ready)
        self._ready_timer.start(500)

        self._ui = QTimer(self)
        self._ui.timeout.connect(self._refresh)
        self._ui.start(100)
        self._close_requested = False
        self._close_poll = QTimer(self)
        self._close_poll.timeout.connect(self._poll_close)
        self._refresh()

    def _on_selection_changed(self):
        if not self.node.managed_stack():
            return
        manager = self.node.stack_manager
        if manager.state == 'running' and not self.node._physics_wanted:
            self.start_btn.setEnabled(True)
            self.status.setText(
                f'Paused — Start will load track={self.track.currentText()} cars={self.cars.value()}'
            )

    def _set_selection_enabled(self, enabled):
        self.track.setEnabled(enabled)
        self.cars.setEnabled(enabled)

    def _poll_gazebo_ready(self):
        if self.node.managed_stack():
            manager = self.node.stack_manager
            state = manager.tick()
            if state == 'error':
                self.start_btn.setEnabled(True)
                self._set_selection_enabled(True)
                self.status.setText(f'Backend error — click Start to retry: {manager.last_error or "unknown error"}')
                return
            if state == 'stopped':
                if self.node._shutdown_pending_clear:
                    self.node.reset_session()
                    self.node._shutdown_pending_clear = False
                self.start_btn.setEnabled(True)
                self.stop_btn.setEnabled(False)
                self._set_selection_enabled(True)
                self.status.setText('Backend stopped — choose track/cars and click Start to retry.')
                return
            if state != 'running' or not self.node.gazebo_ready():
                self.start_btn.setEnabled(False)
                self._set_selection_enabled(False)
                self.status.setText('Loading EUFS backend — waiting for car spawn…')
                return
            config = (manager.track, manager.cars)
            if config[0] and config != self.node._managed_config:
                self.node.configure_telemetry(track=config[0], cars=config[1])
                self.node._managed_config = config
            if self._pending_autostart and config == (self.track.currentText(), self.cars.value()):
                if self.node.start_sim():
                    self._pending_autostart = False
                    self.start_btn.setEnabled(False)
                    self._set_selection_enabled(False)
                    self.status.setText(
                        f'RUNNING track={config[0]} cars={config[1]} — physics stays running until Stop'
                    )
                    return
            paused = not self.node._physics_wanted
            self.start_btn.setEnabled(paused)
            self.stop_btn.setEnabled(not paused)
            self._set_selection_enabled(paused)
            if paused:
                self.status.setText(
                    f'Backend ready: {config[0]} cars={config[1]}. Click Start to run.'
                )
            return
        if not self.node.gazebo_ready():
            return
        self.start_btn.setEnabled(True)
        self.status.setText(
            'Gazebo ready (car spawned). Click Start for gzclient + RViz.'
        )
        self._ready_timer.stop()

    def _on_start(self):
        if self.node.managed_stack():
            manager = self.node.stack_manager
            selected = (self.track.currentText(), self.cars.value())
            if manager.state != 'running':
                self.node.reset_session(track=selected[0], cars=selected[1])
                self._pending_autostart = True
                if not manager.start(*selected):
                    self.status.setText(manager.last_error or 'Backend start failed')
                else:
                    self.status.setText('Starting EUFS backend…')
                return
            if (manager.track, manager.cars) != selected:
                self.node.reset_session(track=selected[0], cars=selected[1])
                manager.start(*selected)
                self._pending_autostart = True
                self.node._physics_wanted = False
                self.start_btn.setEnabled(False)
                self._set_selection_enabled(False)
                self.status.setText(
                    f'Restarting backend for track={selected[0]} cars={selected[1]}…'
                )
                return
            if self.node.start_sim():
                self.start_btn.setEnabled(False)
                self._set_selection_enabled(False)
                self.status.setText(
                    f'RUNNING track={selected[0]} cars={selected[1]} — physics stays running until Stop'
                )
            else:
                self.status.setText('Backend is not ready yet — Start remains available when it spawns')
            return
        if not self.node.gazebo_ready():
            self.status.setText(
                f'Still loading {self.track.currentText()} — wait until Start enables.'
            )
            self.start_btn.setEnabled(False)
            if not self._ready_timer.isActive():
                self._ready_timer.start(500)
            return
        self.status.setText('Attaching gzclient + RViz to the existing gzserver...')
        QApplication.processEvents()
        settle = time.time() + 1.5
        while time.time() < settle:
            QApplication.processEvents()
            time.sleep(0.05)
        if self.node.start_sim():
            self.status.setText(
                f'RUNNING track={self.track.currentText()} cars={self.cars.value()}  '
                'physics stays running until Stop'
            )
        else:
            self.status.setText('gzserver not ready — Start aborted so Gazebo was not killed')

    def _on_stop(self):
        self._drive_timer.stop()
        self._braking = False
        self._brake_ticks_left = 0
        self.node._throttle_active = False
        if self.node.stop_sim():
            self.status.setText('PAUSED — physics stays paused until Start')
        else:
            self.status.setText('Stop skipped: /pause_physics not ready')

    def _on_throttle(self):
        if self.node._throttle_active:
            self._finish_drive('10s throttle cancelled — commands zeroed')
            return
        if not self.node.ensure_running():
            self.status.setText('Drive rejected: Gazebo is not ready')
            return
        if not self.node.command_ready():
            self.status.setText(
                f'Drive rejected: no subscriber on {self.node.ack_topic}'
            )
            return
        if not self.node.request_manual_mission():
            self.status.setText('Drive rejected: mission gate is not ready')
            return
        self.node._throttle_active = True
        self._drive.arm(self.node.sim_time())
        self._drive_odom_seq = self.node._odom_seq
        self._braking = False
        self._brake_ticks_left = 0
        self._drive_timer.start(int(1000 / THROTTLE_HZ))
        self.status.setText(
            f'10s front throttle: {self.node.ack_topic} accel={THROTTLE_ACCEL_MPS2:g} '
            f'steer=0 speed={THROTTLE_SPEED_MPS:g}; waiting for simulation time'
        )
        self._drive_tick()

    def _on_shutdown(self):
        """Delegate lifecycle teardown to the optional launch helper."""
        self._drive_timer.stop()
        self._braking = False
        self._brake_ticks_left = 0
        self._pending_autostart = False
        self.node.shutdown_sim()
        if self.node.managed_stack():
            self.start_btn.setEnabled(False)
            self.stop_btn.setEnabled(False)
            self._set_selection_enabled(False)
            self.status.setText('Shutting down backend, Gazebo, and RViz…')
        else:
            self.status.setText('PAUSED — dashboard remains ready to start again')

    def _poll_close(self):
        manager = self.node.stack_manager
        if manager is None:
            self._close_poll.stop()
            self.close()
            return
        manager.tick()
        if manager.state == 'stopped':
            self._close_poll.stop()
            self.close()

    def _finish_drive(self, message):
        self._drive_timer.stop()
        self.node._throttle_active = False
        self._braking = True
        self._brake_started_wall = time.monotonic()
        self._brake_settle_started_wall = None
        self._drive_timer.start(int(1000 / THROTTLE_HZ))
        self.status.setText(message)

    def _drive_tick(self):
        if self._braking:
            now_wall = time.monotonic()
            if not self.node.odom_fresh():
                self.node.publish_brake_cmd()
                if now_wall - self._brake_started_wall >= BRAKE_TIMEOUT_S:
                    self.node.publish_stop_cmd()
                    self._braking = False
                    self._drive_timer.stop()
                    self.status.setText('Brake failed: odometry became stale; commands zeroed')
                return
            speed = abs(self.node.act_vx or 0.0)
            if speed >= 0.05:
                self.node.publish_brake_cmd()
            else:
                self.node.publish_stop_cmd()
            if speed < 0.05:
                if self._brake_settle_started_wall is None:
                    self._brake_settle_started_wall = now_wall
                elif now_wall - self._brake_settle_started_wall >= BRAKE_SETTLE_S:
                    self.node.publish_stop_cmd()
                    self._braking = False
                    self._drive_timer.stop()
                    self.status.setText('Drive settled — speed below 0.05 m/s, command zeroed')
            else:
                self._brake_settle_started_wall = None
            if now_wall - self._brake_started_wall >= BRAKE_TIMEOUT_S:
                self.node.publish_stop_cmd()
                self._braking = False
                self._drive_timer.stop()
                self.status.setText('Brake timeout — command zeroed')
            return
        if not self.node._throttle_active:
            self._drive_timer.stop()
            return
        now_wall = time.monotonic()
        sim_time = self.node.sim_time()
        if self._drive.baseline_sim is None and sim_time is not None:
            self._drive.arm(sim_time)
        if not self.node.odom_fresh() or self.node._odom_seq <= self._drive_odom_seq:
            self.node.publish_stop_cmd()
            if self._drive.started_wall and now_wall - self._drive.started_wall >= 3.0:
                self._finish_drive('Drive aborted: odometry is stale')
            return
        drive_state = self._drive.state(sim_time, now_wall)
        if drive_state == 'done':
            self._finish_drive('10s throttle done — commands zeroed, physics still RUNNING')
            return
        if drive_state == 'timeout':
            self._finish_drive('Drive safety timeout — simulation clock stalled or physics too slow')
            return
        if drive_state == 'waiting':
            self.node.publish_stop_cmd()
            return
        self.node.publish_forward_cmd()

    def _refresh(self):
        n = self.node
        n.update_derived_telemetry()
        self.history_plot.update()
        frame = n.telemetry_frame()
        kin = frame.ego.kinematics
        self.opponent_summary.clearContents()
        self.opponent_summary.clearSpans()
        self.opponent_summary.setRowCount(len(frame.opponents) or 1)
        if not frame.opponents:
            item = QTableWidgetItem('No opponent source')
            item.setToolTip('No opponent odometry source is configured for the selected car count.')
            self.opponent_summary.setItem(0, 0, item)
            self.opponent_summary.setSpan(0, 0, 1, self.opponent_summary.columnCount())
        else:
            for row, opponent in enumerate(frame.opponents.values()):
                values = (
                    opponent.car_id,
                    _fmt(opponent.kinematics.speed_kmh, ' km/h'),
                    _fmt(opponent.gap_to_ego_m, ' m'),
                    _fmt(opponent.gap_to_ego_s, ' s'),
                    _fmt(opponent.kinematics.x_world, ' m'),
                    _fmt(opponent.kinematics.y_world, ' m'),
                    _fmt(opponent.kinematics.y, ' m'),
                )
                for column, value in enumerate(values):
                    self.opponent_summary.setItem(row, column, QTableWidgetItem(value))
        self.physics.setText(n.physics_state())
        self.cmd_v.setText(_fmt(n.cmd_vx, ' m/s'))
        self.act_v.setText(_fmt(n.act_vx if n.odom_fresh() else None, ' m/s'))
        self.accel.setText(_fmt(None if kin.accel_long_g is None else kin.accel_long_g * 9.80665, ' m/s²'))
        self.speed_kmh.setText(_fmt(kin.speed_kmh, ' km/h'))
        self.heading.setText(_fmt(None if kin.heading_rad is None else math.degrees(kin.heading_rad), '°'))
        if kin.x_world is None or kin.y_world is None:
            self.pose.setText('—')
        else:
            self.pose.setText(f'{kin.x_world:.2f}, {kin.y_world:.2f} m (world)')
        self.s_coord.setText(_fmt(kin.s, ' m'))
        self.lateral.setText(_fmt(kin.y, ' m'))
        self.accel_long_g.setText(_fmt(kin.accel_long_g, ' g'))
        self.accel_lat_g.setText(_fmt(kin.accel_lat_g, ' g'))
        self.yaw_rate.setText(_fmt(kin.yaw_rate_rads, ' rad/s'))
        self.lap.setText(_fmt(frame.ego.lap_number, ''))
        self.session.setText(_fmt(frame.ego.session_time, ' s'))
        self.track_status.setText(frame.ego.track_status or '—')
        self.steer.setText(_fmt(n.steer_cmd, ' rad'))

        if n.cell_soc is None:
            self.cell_soc.setText('N/A (no cell_percentage)')
        else:
            self.cell_soc.setText(n.cell_soc)
        battery_live = n.battery_fresh()
        ers = frame.ego.ers
        self.batt_temps.setText(_fmt(ers.pack_temp_c, ' °C', 1))
        self.batt_soc.setText(_fmt(ers.soc_pct, ' %', 1))
        self.current.setText(_fmt(ers.current_demand_a, ' A', 2))
        self.max_cell_temp.setText(_fmt(ers.max_cell_temp_c, ' °C', 1))
        if n.cell_temps is None:
            self.cell_temps.setText('N/A (no cell_temperature)')
        else:
            self.cell_temps.setText(n.cell_temps if battery_live else '—')
        self.voltage.setText(_fmt(ers.voltage_v, ' V'))
        pack_power = (ers.voltage_v * ers.current_demand_a / 1000.0
                      if ers.voltage_v is not None and ers.current_demand_a is not None else None)
        self.pack_power.setText(_fmt(pack_power, ' kW'))
        self.mode.setText(n.forgez_mode())
        self.t_core.setText(f'{n.forgez_t_core()} °C')
        self.e_lap.setText(f'{n.forgez_e_lap()} Wh')
        self.r_ot.setText(f'{n.forgez_r_ot()} Ω')
        charge = _fmt(n.charge_wh, ' Wh', 1)
        remain = _fmt(n.lap_remain_wh, ' Wh', 1)
        self.energy.setText(f'{charge} / {remain}' if battery_live else '—')
        self.derate.setText((n.derate or '—') if battery_live else '—')

        tyres_live = n.tyres_fresh()
        if tyres_live and n.tyre_temps:
            temps = [value for value in n.tyre_temps[:4] if value is not None]
            self.tyre_temp.setText(
                ' '.join(f'{t:.1f}' for t in temps) + ' °C' if temps else '—'
            )
        else:
            self.tyre_temp.setText('waiting /eufs/tyres/temps')
        self.tyre_rate.setText(_fmt(n.tyre_deg_rate, ' %/s', 4) if tyres_live else '—')
        self.tyre_lap.setText(_fmt(n.tyre_lap_deg, ' %', 3) if tyres_live else '—')
        self.tyre_life.setText(_fmt(frame.ego.tires.wear_pct, ' % wear', 2))
        corner_temp = frame.ego.tires.surface_temp_c
        self.tyre_temp.setText(' '.join(
            _fmt(corner_temp.get(corner), ' °C', 1) for corner in ('FL', 'FR', 'RL', 'RR')
        ))
        self.temp_fl.setText(_fmt(corner_temp.get('FL'), ' °C', 1))
        self.temp_fr.setText(_fmt(corner_temp.get('FR'), ' °C', 1))
        self.temp_rl.setText(_fmt(corner_temp.get('RL'), ' °C', 1))
        self.temp_rr.setText(_fmt(corner_temp.get('RR'), ' °C', 1))

        rpm = n.wheel_rpm
        tyre_rpm = n.tyre_rpm or []

        def _rpm(joint, index):
            value = rpm.get(joint) if tyres_live else None
            if value is None and tyres_live and index < len(tyre_rpm):
                value = tyre_rpm[index]
            return _fmt(value, ' rpm', 0)

        self.fl.setText(_rpm('left_front_wheel_joint', 0))
        self.fr.setText(_rpm('right_front_wheel_joint', 1))
        self.rl.setText(_rpm('left_rear_wheel_joint', 2))
        self.rr.setText(_rpm('right_rear_wheel_joint', 3))

    def closeEvent(self, event):
        """Zero the drive command if the operator closes the dashboard."""
        if self.node.managed_stack() and not self._close_requested:
            self._close_requested = True
            self._drive_timer.stop()
            self._ui.stop()
            self._ready_timer.stop()
            self.node.shutdown_sim()
            event.ignore()
            self._close_poll.start(50)
            return
        if self.node.managed_stack() and self.node.stack_manager.state != 'stopped':
            event.ignore()
            return
        self._drive_timer.stop()
        self.node._throttle_active = False
        self.node.publish_brake_cmd()
        for _ in range(max(1, int(BRAKE_S * THROTTLE_HZ))):
            time.sleep(1.0 / THROTTLE_HZ)
            self.node.publish_brake_cmd()
        self.node.publish_stop_cmd()
        super().closeEvent(event)


def main(args=None):
    if not os.environ.get('DISPLAY'):
        os.environ['DISPLAY'] = ':0'
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    app.setStyle(QStyleFactory.create('Fusion'))
    font = QFont('DejaVu Sans', 10)
    app.setFont(font)
    palette = QPalette()
    palette.setColor(QPalette.Window, QColor('#0b0b0c'))
    palette.setColor(QPalette.WindowText, QColor('#f3f3f3'))
    palette.setColor(QPalette.Base, QColor('#141416'))
    palette.setColor(QPalette.Text, QColor('#ffffff'))
    palette.setColor(QPalette.Button, QColor('#e10600'))
    palette.setColor(QPalette.ButtonText, QColor('#ffffff'))
    app.setPalette(palette)
    node = StartDashboard()
    window = DemoWindow(node)
    window.show()
    window.raise_()
    window.activateWindow()
    node.get_logger().info(
        f'EUFS F1 Demo window shown on DISPLAY={os.environ.get("DISPLAY")}'
    )
    timer = QTimer()

    def _spin():
        deadline = time.monotonic() + 0.005
        callbacks = 0
        try:
            node.tick_stack()
            while callbacks < 32 and time.monotonic() < deadline and rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.0)
                callbacks += 1
        except Exception as exc:  # noqa: BLE001 — keep the Qt loop alive
            node.get_logger().error(f'spin_once: {exc}')

    timer.timeout.connect(_spin)
    timer.start(10)
    code = app.exec_()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
