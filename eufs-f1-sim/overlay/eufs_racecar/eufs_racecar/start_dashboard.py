"""Red/black EUFS F1 telemetry dashboard. Node on load_car.launch.py.

Start/Stop open gzclient+RViz against the existing gzserver and set physics
ONCE. Physics state is the last user action, not a /clock sample.
"""

import math
import os
import subprocess
import sys

from ackermann_msgs.msg import AckermannDriveStamped
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QColor, QFont, QPalette
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
    QVBoxLayout,
    QWidget,
)
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState, JointState
from std_msgs.msg import Float32, Float32MultiArray, Float64, String
from std_srvs.srv import Empty

TRACKS = ('cota', 'small_track')
WHEEL_JOINTS = (
    'left_front_wheel_joint',
    'right_front_wheel_joint',
    'left_rear_wheel_joint',
    'right_rear_wheel_joint',
)
THROTTLE_SPEED_MPS = 8.0
THROTTLE_ACCEL_MPS2 = 8.0
THROTTLE_HZ = 20
THROTTLE_S = 10.0

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


class StartDashboard(Node):
    def __init__(self):
        super().__init__('start_dashboard')
        self.declare_parameter('track', 'cota')
        self.declare_parameter('cars', '1')
        self.declare_parameter('namespace', 'eufs')
        self.declare_parameter('rviz_config', '')
        self.declare_parameter('forgez_mode', 'Nominal')
        self.declare_parameter('forgez_T_core', '40.0')
        self.declare_parameter('forgez_E_lap', '220.0')
        self.declare_parameter('forgez_R_OT', '0.040')
        self.pause_cli = self.create_client(Empty, '/pause_physics')
        self.unpause_cli = self.create_client(Empty, '/unpause_physics')
        namespace = str(self.get_parameter('namespace').value).strip('/')
        ns = f'/{namespace}' if namespace else ''
        self.cmd_topic = f'{ns}/cmd_vel' if ns else '/cmd_vel'
        self.ack_topic = f'{ns}/cmd' if ns else '/cmd'
        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.ack_pub = self.create_publisher(AckermannDriveStamped, self.ack_topic, 10)
        self._gui_procs = []
        # Launch starts gzserver paused. Only Start / 10s throttle / Stop change this.
        self._physics_wanted = False

        self.cmd_vx = None
        self.act_vx = None
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
        self.cell_soc = None
        self.cell_temps = None
        self.tyre_temps = None
        self.tyre_deg_rate = None
        self.tyre_lap_deg = None
        self.tyre_life = None
        self.tyre_rpm = None
        self._throttle_active = False

        self.create_subscription(Twist, self.cmd_topic, self._on_cmd, 10)
        self.create_subscription(Odometry, f'{ns}/odom', self._on_odom, 10)
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
            f'drive topics: Twist {self.cmd_topic} (gate target speed) and '
            f'AckermannDriveStamped {self.ack_topic} (accel+steer)'
        )

    def track(self):
        value = str(self.get_parameter('track').value).strip()
        return value if value in TRACKS else 'cota'

    def cars(self):
        try:
            return max(1, int(self.get_parameter('cars').value))
        except (TypeError, ValueError):
            return 1

    def forgez_mode(self):
        return str(self.get_parameter('forgez_mode').value)

    def forgez_t_core(self):
        return str(self.get_parameter('forgez_T_core').value)

    def forgez_e_lap(self):
        return str(self.get_parameter('forgez_E_lap').value)

    def forgez_r_ot(self):
        return str(self.get_parameter('forgez_R_OT').value)

    def physics_state(self):
        return 'RUNNING' if self._physics_wanted else 'PAUSED'

    def _on_cmd(self, msg):
        self.cmd_vx = msg.linear.x

    def _on_odom(self, msg):
        self.act_vx = msg.twist.twist.linear.x
        self.pose_x = msg.pose.pose.position.x
        self.pose_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny, cosy)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self._last_v is not None and stamp > self._last_v_t + 1e-4:
            self.accel_x = (self.act_vx - self._last_v) / (stamp - self._last_v_t)
        self._last_v = self.act_vx
        self._last_v_t = stamp

    def _on_joints(self, msg):
        velocities = list(msg.velocity) if msg.velocity else []
        for index, name in enumerate(msg.name):
            if name not in self.wheel_rpm or index >= len(velocities):
                continue
            self.wheel_rpm[name] = abs(velocities[index]) * 60.0 / (2.0 * math.pi)

    def _on_battery(self, msg):
        self.voltage = msg.voltage
        self.current = msg.current
        self.soc = msg.percentage
        self.pack_temp = msg.temperature
        cell_pct = list(getattr(msg, 'cell_percentage', None) or [])
        self.cell_soc = (
            ', '.join(f'{p * 100.0:.1f}%' for p in cell_pct) if cell_pct else None
        )
        cell_temps = list(getattr(msg, 'cell_temperature', None) or [])
        self.cell_temps = (
            ', '.join(f'{t:.1f}°C' for t in cell_temps) if cell_temps else None
        )

    def _on_charge_wh(self, msg):
        self.charge_wh = msg.data

    def _on_deploy(self, msg):
        self.deploy_w = msg.data

    def _on_signed(self, msg):
        self.signed_w = msg.data

    def _on_lap_remain(self, msg):
        self.lap_remain_wh = msg.data

    def _on_derate(self, msg):
        self.derate = msg.data

    def _on_tyre_temps(self, msg):
        self.tyre_temps = list(msg.data)

    def _on_tyre_rpm(self, msg):
        self.tyre_rpm = list(msg.data)

    def _on_tyre_rate(self, msg):
        self.tyre_deg_rate = msg.data

    def _on_tyre_lap(self, msg):
        self.tyre_lap_deg = msg.data

    def _on_tyre_life(self, msg):
        self.tyre_life = msg.data

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

    def _start_guis(self):
        env = self._gui_env()
        if not _pgrep('gzclient'):
            proc = subprocess.Popen(
                ['gzclient', '--gui-client-plugin=libgazebo_ros_eol_gui.so'],
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

    def _call_empty(self, client):
        if not client.wait_for_service(timeout_sec=0.5):
            self.get_logger().warn(f'{client.srv_name} is not ready')
            return False
        client.call_async(Empty.Request())
        return True

    def publish_forward_cmd(self):
        twist = Twist()
        twist.linear.x = THROTTLE_SPEED_MPS
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)
        ack = AckermannDriveStamped()
        ack.header.stamp = self.get_clock().now().to_msg()
        ack.drive.steering_angle = 0.0
        ack.drive.acceleration = THROTTLE_ACCEL_MPS2
        ack.drive.speed = THROTTLE_SPEED_MPS
        self.ack_pub.publish(ack)

    def publish_stop_cmd(self):
        self.cmd_pub.publish(Twist())
        ack = AckermannDriveStamped()
        ack.header.stamp = self.get_clock().now().to_msg()
        ack.drive.steering_angle = 0.0
        ack.drive.acceleration = 0.0
        ack.drive.speed = 0.0
        self.ack_pub.publish(ack)

    def start_sim(self):
        self._start_guis()
        self._physics_wanted = True
        return self._call_empty(self.unpause_cli)

    def stop_sim(self):
        self._throttle_active = False
        self.publish_stop_cmd()
        self._physics_wanted = False
        return self._call_empty(self.pause_cli)

    def ensure_running(self):
        if self._physics_wanted:
            return True
        self._physics_wanted = True
        return self._call_empty(self.unpause_cli)


def _card():
    frame = QFrame()
    frame.setObjectName('card')
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
        self.cars.setMaximum(1)
        self.cars.setValue(min(1, node.cars()))

        start = QPushButton('Start')
        stop = QPushButton('Stop')
        stop.setObjectName('stop')
        throttle = QPushButton('10s front throttle')
        throttle.setObjectName('throttle')
        start.clicked.connect(self._on_start)
        stop.clicked.connect(self._on_stop)
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
        header.addWidget(throttle)

        motion, mgrid = _card()
        sec_m = QLabel('MOTION')
        sec_m.setObjectName('section')
        sec_m.setTextFormat(Qt.PlainText)
        self.cmd_v = _kv(mgrid, 1, 0, 'Commanded vx')
        self.act_v = _kv(mgrid, 1, 1, 'Actual vx')
        self.accel = _kv(mgrid, 2, 0, 'Acceleration')
        self.yaw = _kv(mgrid, 2, 1, 'Yaw')
        self.pose = _kv(mgrid, 3, 0, 'Pose x, y')
        self.steer = _kv(mgrid, 3, 1, 'Steer cmd')
        mgrid.addWidget(sec_m, 0, 0, 1, 4)

        batt, bgrid = _card()
        sec_b = QLabel('BATTERY')
        sec_b.setObjectName('section')
        sec_b.setTextFormat(Qt.PlainText)
        self.cell_soc = _kv(bgrid, 1, 0, 'Cell SOC')
        self.batt_temps = _kv(bgrid, 1, 1, 'Battery Temps')
        self.batt_soc = _kv(bgrid, 2, 0, 'Battery SOC')
        self.current = _kv(bgrid, 2, 1, 'Current Demand')
        self.cell_temps = _kv(bgrid, 3, 0, 'Cell Temps')
        self.voltage = _kv(bgrid, 3, 1, 'Voltage')
        self.mode = _kv(bgrid, 4, 0, 'Forgez mode')
        self.t_core = _kv(bgrid, 4, 1, 'T_core (target)')
        self.e_lap = _kv(bgrid, 5, 0, 'E_lap')
        self.r_ot = _kv(bgrid, 5, 1, 'R_OT')
        self.energy = _kv(bgrid, 6, 0, 'Charge / lap left')
        self.derate = _kv(bgrid, 6, 1, 'Derate')
        bgrid.addWidget(sec_b, 0, 0, 1, 4)

        tyres, tgrid = _card()
        sec_t = QLabel('TYRES')
        sec_t.setObjectName('section')
        sec_t.setTextFormat(Qt.PlainText)
        self.tyre_temp = _kv(tgrid, 1, 0, 'Tire temps')
        self.tyre_rate = _kv(tgrid, 1, 1, 'Degradation rate')
        self.tyre_lap = _kv(tgrid, 2, 0, 'Lap-time tire deg')
        self.tyre_life = _kv(tgrid, 2, 1, 'Tire life')
        self.fl = _kv(tgrid, 3, 0, 'FL rpm')
        self.fr = _kv(tgrid, 3, 1, 'FR rpm')
        self.rl = _kv(tgrid, 4, 0, 'RL rpm')
        self.rr = _kv(tgrid, 4, 1, 'RR rpm')
        tgrid.addWidget(sec_t, 0, 0, 1, 4)

        self.status = QLabel(
            'Click Start to open Gazebo and RViz. 10s front throttle commands '
            f'{node.ack_topic} (accel+steer) and {node.cmd_topic} (target speed).'
        )
        self.status.setObjectName('status')
        self.status.setWordWrap(True)

        form = QVBoxLayout()
        form.setSpacing(12)
        form.addLayout(header)
        form.addWidget(motion)
        form.addWidget(batt)
        form.addWidget(tyres)
        form.addWidget(self.status)
        self.setLayout(form)

        self._drive_ticks_left = 0
        self._drive_timer = QTimer(self)
        self._drive_timer.timeout.connect(self._drive_tick)

        self._ui = QTimer(self)
        self._ui.timeout.connect(self._refresh)
        self._ui.start(100)
        self._refresh()

    def _on_start(self):
        if self.node.start_sim():
            self.status.setText(
                f'RUNNING track={self.track.currentText()} cars={self.cars.value()}  '
                'physics stays running until Stop'
            )
        else:
            self.status.setText('Opened GUIs; /unpause_physics not ready')

    def _on_stop(self):
        self._drive_timer.stop()
        self._drive_ticks_left = 0
        if self.node.stop_sim():
            self.status.setText('PAUSED — physics stays paused until Start')
        else:
            self.status.setText('Stop skipped: /pause_physics not ready')

    def _on_throttle(self):
        self.node.ensure_running()
        self.node._throttle_active = True
        self._drive_ticks_left = int(THROTTLE_S * THROTTLE_HZ)
        self._drive_timer.start(int(1000 / THROTTLE_HZ))
        self.status.setText(
            f'10s front throttle: {self.node.ack_topic} accel={THROTTLE_ACCEL_MPS2:g} '
            f'steer=0 speed={THROTTLE_SPEED_MPS:g} and {self.node.cmd_topic} linear.x='
            f'{THROTTLE_SPEED_MPS:g}'
        )
        self._drive_tick()

    def _drive_tick(self):
        if self._drive_ticks_left <= 0:
            self._drive_timer.stop()
            self.node._throttle_active = False
            self.node.publish_stop_cmd()
            self.status.setText('10s throttle done — commands zeroed, physics still RUNNING')
            return
        self.node.publish_forward_cmd()
        self._drive_ticks_left -= 1

    def _refresh(self):
        n = self.node
        self.physics.setText(n.physics_state())
        self.cmd_v.setText(_fmt(n.cmd_vx, ' m/s'))
        self.act_v.setText(_fmt(n.act_vx, ' m/s'))
        self.accel.setText(_fmt(n.accel_x, ' m/s²'))
        self.yaw.setText(_fmt(None if n.yaw is None else math.degrees(n.yaw), '°'))
        if n.pose_x is None:
            self.pose.setText('—')
        else:
            self.pose.setText(f'{n.pose_x:.2f}, {n.pose_y:.2f} m')
        self.steer.setText('0.00 rad' if n._throttle_active else '0 (hold)')

        if n.cell_soc is None:
            self.cell_soc.setText('N/A (no cell_percentage)')
        else:
            self.cell_soc.setText(n.cell_soc)
        self.batt_temps.setText(_fmt(n.pack_temp, ' °C', 1) if n.pack_temp is not None else '—')
        if n.soc is None:
            self.batt_soc.setText('—')
        else:
            self.batt_soc.setText(f'{n.soc * 100.0:.1f} %')
        if n.current is None:
            self.current.setText('—')
        else:
            extra = f'  deploy {_fmt(n.deploy_w, " W", 0)}' if n.deploy_w is not None else ''
            self.current.setText(f'{n.current:.2f} A{extra}')
        if n.cell_temps is None:
            self.cell_temps.setText('N/A (no cell_temperature)')
        else:
            self.cell_temps.setText(n.cell_temps)
        self.voltage.setText(_fmt(n.voltage, ' V'))
        self.mode.setText(n.forgez_mode())
        self.t_core.setText(f'{n.forgez_t_core()} °C')
        self.e_lap.setText(f'{n.forgez_e_lap()} Wh')
        self.r_ot.setText(f'{n.forgez_r_ot()} Ω')
        charge = _fmt(n.charge_wh, ' Wh', 1)
        remain = _fmt(n.lap_remain_wh, ' Wh', 1)
        self.energy.setText(f'{charge} / {remain}')
        self.derate.setText(n.derate or '—')

        if n.tyre_temps:
            self.tyre_temp.setText(
                ' '.join(f'{t:.1f}' for t in n.tyre_temps[:4]) + ' °C'
            )
        else:
            self.tyre_temp.setText('waiting /eufs/tyres/temps')
        self.tyre_rate.setText(_fmt(n.tyre_deg_rate, ' %/s', 4))
        self.tyre_lap.setText(_fmt(n.tyre_lap_deg, ' %', 3))
        if n.tyre_life is None:
            self.tyre_life.setText('waiting /eufs/tyres/life')
        else:
            self.tyre_life.setText(f'{n.tyre_life:.2f} %')

        rpm = n.wheel_rpm
        tyre_rpm = n.tyre_rpm or []

        def _rpm(joint, index):
            value = rpm.get(joint)
            if value is None and index < len(tyre_rpm):
                value = tyre_rpm[index]
            return _fmt(value, ' rpm', 0)

        self.fl.setText(_rpm('left_front_wheel_joint', 0))
        self.fr.setText(_rpm('right_front_wheel_joint', 1))
        self.rl.setText(_rpm('left_rear_wheel_joint', 2))
        self.rr.setText(_rpm('right_rear_wheel_joint', 3))


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
        try:
            rclpy.spin_once(node, timeout_sec=0.0)
        except Exception as exc:  # noqa: BLE001 — keep the Qt loop alive
            node.get_logger().error(f'spin_once: {exc}')

    timer.timeout.connect(_spin)
    timer.start(20)
    code = app.exec_()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
