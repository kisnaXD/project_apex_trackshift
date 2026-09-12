"""Stock PyQt5 start window launched as a Node from load_car.launch.py.

Start/Stop pause and unpause the Gazebo instance that launch already started.
This node never starts Gazebo or another launch file.
"""

import os
import sys

from geometry_msgs.msg import Twist
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import (
    QApplication,
    QComboBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)
import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty

TRACKS = ('cota', 'small_track')


class StartDashboard(Node):
    def __init__(self):
        super().__init__('start_dashboard')
        self.declare_parameter('track', 'cota')
        # Humble launch YAML stringifies integers; declare a string and coerce.
        self.declare_parameter('cars', '1')
        self.declare_parameter('namespace', 'eufs')
        self.pause_cli = self.create_client(Empty, '/pause_physics')
        self.unpause_cli = self.create_client(Empty, '/unpause_physics')
        namespace = str(self.get_parameter('namespace').value).strip('/')
        cmd_topic = f'/{namespace}/cmd_vel' if namespace else '/cmd_vel'
        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)

    def track(self):
        value = str(self.get_parameter('track').value).strip()
        return value if value in TRACKS else 'cota'

    def cars(self):
        try:
            return max(1, int(self.get_parameter('cars').value))
        except (TypeError, ValueError):
            return 1

    def start_sim(self):
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


class StartWindow(QWidget):
    def __init__(self, node: StartDashboard):
        super().__init__()
        self.node = node
        self.setWindowTitle('EUFS F1 Start')

        self.track = QComboBox()
        self.track.addItems(TRACKS)
        self.track.setCurrentText(node.track())

        self.cars = QSpinBox()
        self.cars.setMinimum(1)
        self.cars.setMaximum(1)
        self.cars.setValue(min(1, node.cars()))

        self.status = QLabel('Gazebo is already running from load_car.launch.py')

        start = QPushButton('Start')
        stop = QPushButton('Stop')
        start.clicked.connect(self._on_start)
        stop.clicked.connect(self._on_stop)

        form = QVBoxLayout()
        track_row = QHBoxLayout()
        track_row.addWidget(QLabel('Track'))
        track_row.addWidget(self.track)
        cars_row = QHBoxLayout()
        cars_row.addWidget(QLabel('Cars'))
        cars_row.addWidget(self.cars)
        buttons = QHBoxLayout()
        buttons.addWidget(start)
        buttons.addWidget(stop)
        form.addLayout(track_row)
        form.addLayout(cars_row)
        form.addLayout(buttons)
        form.addWidget(self.status)
        self.setLayout(form)

    def _on_start(self):
        if self.node.start_sim():
            self.status.setText(
                f'Running track={self.track.currentText()} cars={self.cars.value()}'
            )
        else:
            self.status.setText('Start skipped: /unpause_physics not ready')

    def _on_stop(self):
        if self.node.stop_sim():
            self.status.setText('Paused')
        else:
            self.status.setText('Stop skipped: /pause_physics not ready')


def main(args=None):
    if not os.environ.get('DISPLAY'):
        os.environ['DISPLAY'] = ':0'
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    node = StartDashboard()
    window = StartWindow(node)
    window.show()
    window.raise_()
    window.activateWindow()
    node.get_logger().info(
        f'EUFS F1 Start window shown on DISPLAY={os.environ.get("DISPLAY")}'
    )
    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0.0))
    timer.start(50)
    code = app.exec_()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
