"""Red/black EUFS F1 telemetry dashboard. Node on load_car.launch.py.

Start/Stop still open gzclient+RViz and pause the existing gzserver.
Display-only otherwise — no drive controls on this window.
"""

import math
import os
import subprocess
import sys
import time

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import (
    QApplication,
    QComboBox,
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import BatteryState, JointState
from std_msgs.msg import Float64, String
from std_srvs.srv import Empty

TRACKS = ('cota', 'small_track')
WHEEL_JOINTS = (
    'left_front_wheel_joint',
    'right_front_wheel_joint',
    'left_rear_wheel_joint',
    'right_rear_wheel_joint',
)
THEME = """
QWidget { background-color: #0b0b0c; color: #f3f3f3; font-size: 13px; }
QLabel#title { color: #e10600; font-size: 22px; font-weight: 700; }
QLabel#section { color: #e10600; font-size: 13px; font-weight: 700; letter-spacing: 1px; }
QLabel#key { color: #9a9a9a; }
QLabel#val { color: #ffffff; font-weight: 600; font-family: "DejaVu Sans Mono", monospace; }
QLabel#na { color: #777777; font-style: italic; }
QLabel#status { color: #d0d0d0; }
QPushButton {
  background: #e10600; color: #fff; border: 0; padding: 8px 18px; font-weight: 700;
}
QPushButton:hover { background: #ff2a1f; }
QPushButton#stop { background: #2a2a2a; }
QPushButton#stop:hover { background: #444; }
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
        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self._gui_procs = []

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
        self.sim_paused = True
        self._last_v = None
        self._last_v_t = None
        self._last_clock = None
        self._last_clock_wall = 0.0
        self.cell_soc = None
        self.cell_temps = None

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
        clock_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Clock, '/clock', self._on_clock, clock_qos)

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
        for name, vel in zip(msg.name, msg.velocity):
            if name in self.wheel_rpm:
                self.wheel_rpm[name] = abs(vel) * 60.0 / (2.0 * math.pi)

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

    def _on_clock(self, msg):
        now = msg.clock.sec + msg.clock.nanosec * 1e-9
        self.sim_paused = self._last_clock is not None and abs(now - self._last_clock) < 1e-9
        self._last_clock = now
        self._last_clock_wall = time.monotonic()

        self._last_clock_ns = None

    def physics_state(self):
        now = self.get_clock().now().nanoseconds
        if self._last_clock_ns is None:
            state = 'WAITING'
        elif now == self._last_clock_ns:
            state = 'PAUSED'
        else:
            state = 'RUNNING'
        self._last_clock_ns = now
        return state

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

    def start_sim(self):
        self._start_guis()
        return self._call_empty(self.unpause_cli)

    def stop_sim(self):
        self.cmd_pub.publish(Twist())
        return self._call_empty(self.pause_cli)

    def _call_empty(self, client):
        if not client.wait_for_service(timeout_sec=0.5):
            self.get_logger().warn(f'{client.srv_name} is not ready')
            return False
        client.call_async(Empty.Request())
        return True


def _card():
    frame = QFrame()
    frame.setObjectName('card')
    layout = QGridLayout(frame)
    layout.setContentsMargins(12, 10, 12, 10)
    layout.setHorizontalSpacing(16)
    layout.setVerticalSpacing(6)
    return frame, layout


def _kv(layout, row, col, key):
    k = QLabel(key)
    k.setObjectName('key')
    v = QLabel('—')
    v.setObjectName('val')
    layout.addWidget(k, row, col * 2)
    layout.addWidget(v, row, col * 2 + 1)
    return v


class DemoWindow(QWidget):
    def __init__(self, node: StartDashboard):
        super().__init__()
        self.node = node
        self.setWindowTitle('EUFS F1 Demo')
        self.setStyleSheet(THEME)
        self.setMinimumSize(920, 680)

        title = QLabel('EUFS F1')
        title.setObjectName('title')
        self.physics = QLabel('PAUSED')
        self.physics.setObjectName('val')

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
        start.clicked.connect(self._on_start)
        stop.clicked.connect(self._on_stop)

        header = QHBoxLayout()
        header.addWidget(title)
        header.addStretch(1)
        header.addWidget(QLabel('Track'))
        header.addWidget(self.track)
        header.addWidget(QLabel('Cars'))
        header.addWidget(self.cars)
        header.addWidget(start)
        header.addWidget(stop)

        motion, mgrid = _card()
        sec_m = QLabel('MOTION')
        sec_m.setObjectName('section')
        self.cmd_v = _kv(mgrid, 1, 0, 'Commanded vx')
        self.act_v = _kv(mgrid, 1, 1, 'Actual vx')
        self.accel = _kv(mgrid, 2, 0, 'Acceleration')
        self.yaw = _kv(mgrid, 2, 1, 'Yaw')
        self.pose = _kv(mgrid, 3, 0, 'Pose x, y')
        self.sim = _kv(mgrid, 3, 1, 'Physics')
        mgrid.addWidget(sec_m, 0, 0, 1, 4)

        batt, bgrid = _card()
        sec_b = QLabel('BATTERY')
        sec_b.setObjectName('section')
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
        sec_t = QLabel('TYRES / WHEELS')
        sec_t.setObjectName('section')
        self.fl = _kv(tgrid, 1, 0, 'FL rpm')
        self.fr = _kv(tgrid, 1, 1, 'FR rpm')
        self.rl = _kv(tgrid, 2, 0, 'RL rpm')
        self.rr = _kv(tgrid, 2, 1, 'RR rpm')
        note = QLabel('No tyre temperature or pressure topics in this sim — wheel speed from /eufs/joint_states only.')
        note.setObjectName('na')
        note.setWordWrap(True)
        tgrid.addWidget(sec_t, 0, 0, 1, 4)
        tgrid.addWidget(note, 3, 0, 1, 4)

        self.status = QLabel('Click Start to open Gazebo and RViz. This panel is telemetry only.')
        self.status.setObjectName('status')
        self.status.setWordWrap(True)

        form = QVBoxLayout()
        form.addLayout(header)
        form.addWidget(self.physics)
        form.addWidget(motion)
        form.addWidget(batt)
        form.addWidget(tyres)
        form.addWidget(self.status)
        self.setLayout(form)

        self._ui = QTimer(self)
        self._ui.timeout.connect(self._refresh)
        self._ui.start(100)
        self._refresh()

    def _on_start(self):
        if self.node.start_sim():
            self.status.setText(
                f'Running track={self.track.currentText()} cars={self.cars.value()}'
            )
        else:
            self.status.setText('Opened GUIs; /unpause_physics not ready')

    def _on_stop(self):
        if self.node.stop_sim():
            self.status.setText('Paused')
        else:
            self.status.setText('Stop skipped: /pause_physics not ready')

    def _refresh(self):
        n = self.node
        state = n.physics_state()
        self.physics.setText(state)
        self.cmd_v.setText(_fmt(n.cmd_vx, ' m/s'))
        self.act_v.setText(_fmt(n.act_vx, ' m/s'))
        self.accel.setText(_fmt(n.accel_x, ' m/s²'))
        self.yaw.setText(_fmt(None if n.yaw is None else math.degrees(n.yaw), '°'))
        if n.pose_x is None:
            self.pose.setText('—')
        else:
            self.pose.setText(f'{n.pose_x:.2f}, {n.pose_y:.2f} m')
        self.sim.setText(state)

        if n.cell_soc is None:
            self.cell_soc.setText('N/A (no cell_percentage on /eufs/forgez/battery_state)')
            self.cell_soc.setObjectName('na')
        else:
            self.cell_soc.setText(n.cell_soc)
            self.cell_soc.setObjectName('val')
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
            self.cell_temps.setText('N/A (no cell_temperature on /eufs/forgez/battery_state)')
            self.cell_temps.setObjectName('na')
        else:
            self.cell_temps.setText(n.cell_temps)
            self.cell_temps.setObjectName('val')
        self.voltage.setText(_fmt(n.voltage, ' V'))
        self.mode.setText(n.forgez_mode())
        self.t_core.setText(f'{n.forgez_t_core()} °C')
        self.e_lap.setText(f'{n.forgez_e_lap()} Wh')
        self.r_ot.setText(f'{n.forgez_r_ot()} Ω')
        charge = _fmt(n.charge_wh, ' Wh', 1)
        remain = _fmt(n.lap_remain_wh, ' Wh', 1)
        self.energy.setText(f'{charge} / {remain}')
        self.derate.setText(n.derate or '—')

        rpm = n.wheel_rpm
        self.fl.setText(_fmt(rpm['left_front_wheel_joint'], ' rpm', 0))
        self.fr.setText(_fmt(rpm['right_front_wheel_joint'], ' rpm', 0))
        self.rl.setText(_fmt(rpm['left_rear_wheel_joint'], ' rpm', 0))
        self.rr.setText(_fmt(rpm['right_rear_wheel_joint'], ' rpm', 0))


def main(args=None):
    if not os.environ.get('DISPLAY'):
        os.environ['DISPLAY'] = ':0'
    rclpy.init(args=args)
    app = QApplication(sys.argv)
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
