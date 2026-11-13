#!/usr/bin/env python3

import sys
import time
import threading
import numpy as np
import rclpy

from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.signals import SignalHandlerOptions

from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from PyQt5 import QtWidgets, QtCore
import pyqtgraph as pg

from planar_arm_control.planar_arm import PlanarArm


LINK_LENGTHS = [3.0, 2.0, 1.5]


# ============================================================
# PROFESSIONAL DARK THEME
# ============================================================

DASHBOARD_STYLE = """
QMainWindow, QWidget {
    background-color: #101820;
    color: #E6EDF3;
    font-family: 'DejaVu Sans';
    font-size: 12px;
}

QLabel {
    color: #E6EDF3;
}

QFrame#Panel {
    background-color: #192630;
    border: 1px solid #304452;
    border-radius: 9px;
}

QLabel#Title {
    font-size: 21px;
    font-weight: bold;
    color: #FFFFFF;
}

QLabel#Subtitle {
    color: #91A5B5;
    font-size: 11px;
}

QLabel#SectionTitle {
    font-size: 14px;
    font-weight: bold;
    color: #62C8F5;
}

QLabel#MetricValue {
    font-size: 21px;
    font-weight: bold;
    color: #62C8F5;
}

QLabel#MetricName {
    color: #91A5B5;
    font-size: 11px;
}

QLineEdit {
    background-color: #101820;
    border: 1px solid #405666;
    border-radius: 5px;
    padding: 9px;
    color: white;
}

QPushButton {
    background-color: #263D4C;
    border: 1px solid #405666;
    border-radius: 5px;
    padding: 10px;
    color: white;
    font-weight: bold;
}

QPushButton:hover {
    background-color: #34566B;
}

QPushButton:disabled {
    background-color: #27323A;
    color: #71808B;
}

QPushButton#Primary {
    background-color: #087EAA;
    border: none;
}

QPushButton#Primary:hover {
    background-color: #079ACD;
}

QPushButton#Success {
    background-color: #147D64;
    border: none;
}

QPushButton#Success:hover {
    background-color: #199B7B;
}

QPushButton#Danger {
    background-color: #8E4141;
    border: none;
}

QPushButton#Danger:hover {
    background-color: #AD5050;
}

QTextEdit {
    background-color: #101820;
    border: 1px solid #304452;
    border-radius: 5px;
    color: #B8D3E2;
    padding: 6px;
}

QStatusBar {
    background-color: #192630;
    color: #91A5B5;
}
"""


# ============================================================
# ROS 2 GUI NODE
# ============================================================

class GuiNode(Node):

    def __init__(self):
        super().__init__("gui_node")

        self.arm = PlanarArm(LINK_LENGTHS)

        self._lock = threading.Lock()
        self.joint_positions = None
        self.status = "Waiting for controller..."

        self.target_pub = self.create_publisher(
            PointStamped,
            "/target_pose",
            10
        )

        self.joint_sub = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_callback,
            10
        )

        self.status_sub = self.create_subscription(
            String,
            "/controller/status",
            self.status_callback,
            10
        )

        self.get_logger().info("Professional GUI node initialized.")

    def joint_callback(self, msg):

        if len(msg.position) != 3:
            return

        q = np.asarray(msg.position, dtype=float)

        if not np.all(np.isfinite(q)):
            return

        with self._lock:
            self.joint_positions = q.copy()

    def status_callback(self, msg):

        with self._lock:
            self.status = msg.data

    def get_snapshot(self):

        with self._lock:
            q = (
                None if self.joint_positions is None
                else self.joint_positions.copy()
            )
            status = self.status

        return q, status

    def set_local_status(self, message):

        with self._lock:
            self.status = message

    def send_target(self, x, y):

        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"

        msg.point.x = float(x)
        msg.point.y = float(y)
        msg.point.z = 0.0

        self.set_local_status(
            f"Command sent: X={x:.2f}, Y={y:.2f}"
        )

        self.target_pub.publish(msg)

        self.get_logger().info(
            f"Target published: x={x:.2f}, y={y:.2f}"
        )


# ============================================================
# METRIC CARD
# ============================================================

class MetricCard(QtWidgets.QFrame):

    def __init__(self, title, initial_value="--"):

        super().__init__()

        self.setObjectName("Panel")
        layout = QtWidgets.QVBoxLayout(self)

        self.title_label = QtWidgets.QLabel(title)
        self.title_label.setObjectName("MetricName")

        self.value_label = QtWidgets.QLabel(initial_value)
        self.value_label.setObjectName("MetricValue")

        layout.addWidget(self.title_label)
        layout.addWidget(self.value_label)


# ============================================================
# MAIN GUI WINDOW
# ============================================================

class ArmWindow(QtWidgets.QMainWindow):

    def __init__(self, node, executor, ros_thread):

        super().__init__()

        self.node = node
        self.executor = executor
        self.ros_thread = ros_thread

        self.setWindowTitle("Kineshia Robotics | Arm Control Station")
        self.resize(1450, 900)
        self.setMinimumSize(1100, 720)

        self.time_data = []
        self.angle_data = [[], [], []]

        self.start_time = time.monotonic()
        self.last_plot_time = 0.0

        self.sequence_state = "idle"
        self.sequence_motion_started = False

        # Simulated pick-and-place object.
        self.object_state = "waiting"
        self.object_position = np.array([4.0, 2.0])

        self.build_ui()

        self.plot_timer = QtCore.QTimer(self)
        self.plot_timer.timeout.connect(self.update_display)
        self.plot_timer.start(50)

        self.sequence_timer = QtCore.QTimer(self)
        self.sequence_timer.timeout.connect(self.check_sequence)
        self.sequence_timer.start(50)

        self.statusBar().showMessage("Kineshia Robotics | ROS 2 Jazzy")

    # --------------------------------------------------------
    # UI CONSTRUCTION
    # --------------------------------------------------------

    def build_ui(self):

        central = QtWidgets.QWidget()
        self.setCentralWidget(central)

        root = QtWidgets.QVBoxLayout(central)
        root.setContentsMargins(15, 12, 15, 12)
        root.setSpacing(12)

        # HEADER

        header = QtWidgets.QHBoxLayout()

        title_layout = QtWidgets.QVBoxLayout()

        title = QtWidgets.QLabel("KINESHIA ROBOTICS")
        title.setObjectName("Title")

        subtitle = QtWidgets.QLabel(
            "3-DOF PLANAR ROBOT ARM | OPERATOR CONTROL STATION"
        )
        subtitle.setObjectName("Subtitle")

        title_layout.addWidget(title)
        title_layout.addWidget(subtitle)

        header.addLayout(title_layout)
        header.addStretch()

        self.connection_label = QtWidgets.QLabel(
            "● WAITING FOR ROS"
        )
        self.connection_label.setStyleSheet(
            "color: #E6B84A; font-weight: bold; padding: 10px;"
        )

        header.addWidget(self.connection_label)
        root.addLayout(header)

        # METRIC CARDS

        metrics = QtWidgets.QHBoxLayout()

        self.metric_x = MetricCard("END EFFECTOR X", "0.000")
        self.metric_y = MetricCard("END EFFECTOR Y", "0.000")

        self.metric_j1 = MetricCard("JOINT 1", "0.00°")
        self.metric_j2 = MetricCard("JOINT 2", "0.00°")
        self.metric_j3 = MetricCard("JOINT 3", "0.00°")

        for card in [
            self.metric_x,
            self.metric_y,
            self.metric_j1,
            self.metric_j2,
            self.metric_j3
        ]:
            metrics.addWidget(card)

        root.addLayout(metrics)

        # MAIN CONTENT

        content = QtWidgets.QHBoxLayout()
        content.setSpacing(12)

        root.addLayout(content, 1)

        # LEFT SIDE

        left = QtWidgets.QVBoxLayout()
        content.addLayout(left, 7)

        # ROBOT VISUALIZATION

        self.arm_plot = pg.PlotWidget(
            title="LIVE 3-DOF ROBOT ARM MODEL"
        )

        self.arm_plot.setBackground("#101820")
        self.arm_plot.setLabel(
            "bottom", "X Position", units="m"
        )
        self.arm_plot.setLabel(
            "left", "Y Position", units="m"
        )

        self.arm_plot.showGrid(
            x=True, y=True, alpha=0.2
        )

        self.arm_plot.setAspectLocked(True)
        self.arm_plot.setXRange(-7, 7)
        self.arm_plot.setYRange(0, 7.5)

        self.arm_plot.getAxis("bottom").setTextPen("#A9BAC8")
        self.arm_plot.getAxis("left").setTextPen("#A9BAC8")

        # Workspace boundary: maximum reach = 3 + 2 + 1.5 = 6.5 m.
        workspace_x = np.linspace(-6.5, 6.5, 180)
        workspace_y = np.sqrt(np.maximum(0.0, 6.5 ** 2 - workspace_x ** 2))
        self.workspace_curve = self.arm_plot.plot(
            workspace_x,
            workspace_y,
            pen=pg.mkPen("#304452", width=1, style=QtCore.Qt.DashLine)
        )

        # Ground reference and fixed robot base.
        self.ground_line = self.arm_plot.plot(
            [-7.0, 7.0], [0.0, 0.0],
            pen=pg.mkPen("#607786", width=2)
        )

        self.base_foot = self.arm_plot.plot(
            [-0.75, -0.55, 0.55, 0.75],
            [0.04, 0.22, 0.22, 0.04],
            pen=pg.mkPen("#A9BAC8", width=5)
        )

        self.base_pedestal = self.arm_plot.plot(
            [0.0, 0.0], [0.18, 0.42],
            pen=pg.mkPen("#607786", width=15)
        )

        self.base_outer = self.arm_plot.plot(
            [0.0], [0.0],
            pen=None,
            symbol="o",
            symbolSize=30,
            symbolBrush="#263D4C",
            symbolPen=pg.mkPen("#62C8F5", width=3)
        )

        self.base_inner = self.arm_plot.plot(
            [0.0], [0.0],
            pen=None,
            symbol="o",
            symbolSize=11,
            symbolBrush="#62C8F5",
            symbolPen=pg.mkPen("#FFFFFF", width=1)
        )

        # Dark shadow beneath the robot links adds visual depth.
        self.arm_shadow = self.arm_plot.plot(
            [], [],
            pen=pg.mkPen("#050A0E", width=22)
        )

        # Three individually styled links.
        link_styles = [
            ("#21B8F2", 17),
            ("#39C98A", 14),
            ("#F5B942", 11)
        ]

        self.link_curves = []

        for color, width in link_styles:
            curve = self.arm_plot.plot(
                [],
                [],
                pen=pg.mkPen(color, width=width)
            )
            self.link_curves.append(curve)

        # Joint housings and inner joint caps.
        self.joint_outer = self.arm_plot.plot(
            [], [],
            pen=None,
            symbol="o",
            symbolSize=25,
            symbolBrush="#192630",
            symbolPen=pg.mkPen("#D8E5ED", width=3)
        )

        self.joint_inner = self.arm_plot.plot(
            [], [],
            pen=None,
            symbol="o",
            symbolSize=10,
            symbolBrush="#62C8F5",
            symbolPen=pg.mkPen("#FFFFFF", width=1)
        )

        # End-effector housing and two simple simulated gripper fingers.
        self.tool_mount = self.arm_plot.plot(
            [], [],
            pen=pg.mkPen("#E6EDF3", width=7)
        )

        self.gripper_curve = self.arm_plot.plot(
            [], [],
            pen=pg.mkPen("#F5B942", width=5)
        )

        self.end_effector_marker = self.arm_plot.plot(
            [], [],
            pen=None,
            symbol="o",
            symbolSize=13,
            symbolBrush="#F5B942",
            symbolPen=pg.mkPen("#FFFFFF", width=2)
        )

        self.target_marker = self.arm_plot.plot(
            [],
            [],
            pen=None,
            symbol="x",
            symbolSize=18,
            symbolPen=pg.mkPen("#F5B942", width=3)
        )

        # Simulated cube: waiting, attached, or released.
        self.object_marker = pg.ScatterPlotItem(
            size=19,
            symbol="s",
            pen=pg.mkPen("#FFFFFF", width=2),
            brush=pg.mkBrush("#E65B5B")
        )
        self.arm_plot.addItem(self.object_marker)

        self.object_label = pg.TextItem(
            text="PICK OBJECT",
            color="#F5B942",
            anchor=(0.5, 1.2)
        )
        self.arm_plot.addItem(self.object_label)

        self.object_marker.setData(
            x=[4.0], y=[2.0]
        )
        self.object_label.setPos(4.0, 2.0)

        left.addWidget(self.arm_plot, 3)

        # JOINT GRAPH

        self.joint_plot = pg.PlotWidget(
            title="REAL-TIME JOINT ANGLE MONITOR"
        )

        self.joint_plot.setBackground("#101820")
        self.joint_plot.showGrid(
            x=True, y=True, alpha=0.2
        )

        self.joint_plot.setLabel(
            "bottom", "Time", units="s"
        )
        self.joint_plot.setLabel(
            "left", "Angle", units="deg"
        )

        self.joint_plot.addLegend()

        for axis in ["bottom", "left"]:
            self.joint_plot.getAxis(axis).setTextPen("#A9BAC8")

        self.joint_curves = []

        colors = ["#21B8F2", "#39C98A", "#F5B942"]

        for name, color in zip(
            ["Joint 1", "Joint 2", "Joint 3"],
            colors
        ):

            curve = self.joint_plot.plot(
                [],
                [],
                pen=pg.mkPen(color, width=2),
                name=name
            )

            self.joint_curves.append(curve)

        left.addWidget(self.joint_plot, 2)

        # RIGHT SIDE: OPERATOR PANEL

        right_panel = QtWidgets.QFrame()
        right_panel.setObjectName("Panel")
        right_panel.setMinimumWidth(285)
        right_panel.setMaximumWidth(360)

        right = QtWidgets.QVBoxLayout(right_panel)
        right.setContentsMargins(15, 15, 15, 15)
        right.setSpacing(10)

        content.addWidget(right_panel, 3)

        control_title = QtWidgets.QLabel("OPERATOR CONTROLS")
        control_title.setObjectName("SectionTitle")
        right.addWidget(control_title)

        # TARGET INPUT

        right.addWidget(QtWidgets.QLabel("Target X coordinate"))

        self.x_input = QtWidgets.QLineEdit("4.0")
        right.addWidget(self.x_input)

        right.addWidget(QtWidgets.QLabel("Target Y coordinate"))

        self.y_input = QtWidgets.QLineEdit("2.0")
        right.addWidget(self.y_input)

        self.send_button = QtWidgets.QPushButton(
            "SEND TARGET"
        )
        self.send_button.setObjectName("Primary")
        self.send_button.clicked.connect(
            self.send_custom_target
        )
        right.addWidget(self.send_button)

        self.add_separator(right)

        # PICK AND PLACE

        pick_title = QtWidgets.QLabel("PICK AND PLACE")
        pick_title.setObjectName("SectionTitle")
        right.addWidget(pick_title)

        self.pick_button = QtWidgets.QPushButton(
            "MOVE TO PICK  (4, 2)"
        )
        self.pick_button.clicked.connect(
            lambda: self.send_target(4.0, 2.0)
        )
        right.addWidget(self.pick_button)

        self.place_button = QtWidgets.QPushButton(
            "MOVE TO PLACE  (-3, 3)"
        )
        self.place_button.clicked.connect(
            lambda: self.send_target(-3.0, 3.0)
        )
        right.addWidget(self.place_button)

        self.sequence_button = QtWidgets.QPushButton(
            "RUN FULL PICK & PLACE"
        )
        self.sequence_button.setObjectName("Success")
        self.sequence_button.setMinimumHeight(45)
        self.sequence_button.clicked.connect(
            self.start_sequence
        )
        right.addWidget(self.sequence_button)

        self.add_separator(right)

        # TEST TARGET

        test_title = QtWidgets.QLabel("VALIDATION")
        test_title.setObjectName("SectionTitle")
        right.addWidget(test_title)

        self.edge_button = QtWidgets.QPushButton(
            "TEST EDGE TARGET  (7, 3)"
        )
        self.edge_button.setObjectName("Danger")
        self.edge_button.clicked.connect(
            lambda: self.send_target(7.0, 3.0)
        )
        right.addWidget(self.edge_button)

        self.add_separator(right)

        # STATUS

        status_title = QtWidgets.QLabel("CONTROLLER STATUS")
        status_title.setObjectName("SectionTitle")
        right.addWidget(status_title)

        self.status_label = QtWidgets.QLabel(
            "Waiting for controller feedback..."
        )
        self.status_label.setWordWrap(True)
        self.status_label.setMinimumHeight(100)
        self.status_label.setAlignment(
            QtCore.Qt.AlignTop | QtCore.Qt.AlignLeft
        )
        self.status_label.setStyleSheet(
            "background-color: #101820;"
            "border: 1px solid #304452;"
            "border-radius: 5px;"
            "padding: 10px;"
            "color: #39C98A;"
        )

        right.addWidget(self.status_label)

        note = QtWidgets.QLabel(
            "NOTE: Pick and place actions are simulated. "
            "No physical gripper is connected."
        )
        note.setWordWrap(True)
        note.setStyleSheet(
            "color: #91A5B5; font-size: 10px;"
        )
        right.addWidget(note)

        right.addStretch()

    def add_separator(self, layout):

        line = QtWidgets.QFrame()
        line.setFrameShape(QtWidgets.QFrame.HLine)
        line.setStyleSheet(
            "color: #304452; background-color: #304452;"
        )
        layout.addWidget(line)

    # --------------------------------------------------------
    # TARGET CONTROL
    # --------------------------------------------------------

    def set_controls_enabled(self, enabled):

        for widget in [
            self.send_button,
            self.pick_button,
            self.place_button,
            self.edge_button,
            self.x_input,
            self.y_input,
            self.sequence_button
        ]:
            widget.setEnabled(enabled)

    def send_target(self, x, y):

        if self.sequence_state != "idle":
            return

        self.node.send_target(x, y)
        self.target_marker.setData([x], [y])

    def send_custom_target(self):

        try:
            x = float(self.x_input.text())
            y = float(self.y_input.text())

            if not np.isfinite(x) or not np.isfinite(y):
                raise ValueError

            self.send_target(x, y)

        except ValueError:

            QtWidgets.QMessageBox.warning(
                self,
                "Invalid Target",
                "Please enter valid numerical X and Y coordinates."
            )

    # --------------------------------------------------------
    # AUTOMATIC PICK AND PLACE
    # --------------------------------------------------------

    def start_sequence(self):

        if self.sequence_state != "idle":
            return

        self.sequence_state = "to_pick"
        self.sequence_motion_started = False

        self.object_state = "waiting"
        self.object_position = np.array([4.0, 2.0])
        self.object_marker.setData(
            x=[4.0], y=[2.0],
            brush=pg.mkBrush("#E65B5B")
        )
        self.object_label.setText("PICK OBJECT")
        self.object_label.setPos(4.0, 2.0)

        self.set_controls_enabled(False)

        self.node.set_local_status(
            "Sequence started: moving to pick position."
        )

        self.node.send_target(4.0, 2.0)
        self.target_marker.setData([4.0], [2.0])

    def check_sequence(self):

        if self.sequence_state == "idle":
            return

        _, status = self.node.get_snapshot()

        if status.startswith("Moving to"):
            self.sequence_motion_started = True

        if (
            self.sequence_motion_started
            and status.startswith("Target reached successfully")
        ):

            self.sequence_motion_started = False

            if self.sequence_state == "to_pick":

                self.sequence_state = "pick_pause"
                self.object_state = "attached"

                self.node.set_local_status(
                    "PICK: Object attached (simulated)."
                )

                QtCore.QTimer.singleShot(
                    1000,
                    self.move_to_place
                )

            elif self.sequence_state == "to_place":

                self.sequence_state = "place_pause"
                self.object_state = "released"

                # Keep the object at the final end-effector position.
                q, _ = self.node.get_snapshot()
                if q is not None:
                    points = self.node.arm.forward_kinematics(q)
                    self.object_position = np.array(
                        points[-1], dtype=float
                    )

                self.node.set_local_status(
                    "PLACE: Object released (simulated)."
                )

                QtCore.QTimer.singleShot(
                    1000,
                    self.finish_sequence
                )

    def move_to_place(self):

        if self.sequence_state != "pick_pause":
            return

        self.sequence_state = "to_place"

        self.node.send_target(-3.0, 3.0)
        self.target_marker.setData([-3.0], [3.0])

        self.node.set_local_status(
            "Moving object to place position."
        )

    def finish_sequence(self):

        if self.sequence_state != "place_pause":
            return

        self.sequence_state = "idle"

        self.node.set_local_status(
            "Pick-and-place sequence completed successfully."
        )

        self.set_controls_enabled(True)

    # --------------------------------------------------------
    # LIVE VISUALIZATION AND TELEMETRY
    # --------------------------------------------------------

    def update_display(self):

        q, status = self.node.get_snapshot()

        self.status_label.setText(status)

        if q is None:
            return

        self.connection_label.setText(
            "● ROS 2 CONNECTED"
        )
        self.connection_label.setStyleSheet(
            "color: #39C98A; font-weight: bold; padding: 10px;"
        )

        # Forward kinematics: base, joint 1, joint 2, end effector.
        points = self.node.arm.forward_kinematics(q)

        x_values = [point[0] for point in points]
        y_values = [point[1] for point in points]

        # A subtle dark line behind the links gives a raised 3D-like effect.
        self.arm_shadow.setData(x_values, y_values)

        # Render each arm segment with a different industrial accent color.
        for index, curve in enumerate(self.link_curves):
            curve.setData(
                [points[index][0], points[index + 1][0]],
                [points[index][1], points[index + 1][1]]
            )

        # Render joint housings at the base and each revolute joint.
        joint_points = points[:-1]
        joint_x = [point[0] for point in joint_points]
        joint_y = [point[1] for point in joint_points]

        self.joint_outer.setData(x=joint_x, y=joint_y)
        self.joint_inner.setData(x=joint_x, y=joint_y)

        # End-effector position.
        end_x, end_y = points[-1]

        # While attached, the cube follows the live end-effector.
        if self.object_state == "attached":
            self.object_position = np.array(
                [end_x, end_y], dtype=float
            )

        object_colors = {
            "waiting": "#E65B5B",
            "attached": "#39C98A",
            "released": "#62C8F5"
        }

        object_labels = {
            "waiting": "PICK OBJECT",
            "attached": "GRIPPED",
            "released": "PLACED"
        }

        object_x, object_y = self.object_position

        self.object_marker.setData(
            x=[object_x],
            y=[object_y],
            brush=pg.mkBrush(
                object_colors[self.object_state]
            )
        )

        self.object_label.setText(
            object_labels[self.object_state]
        )
        self.object_label.setPos(
            object_x, object_y
        )

        self.metric_x.value_label.setText(
            f"{end_x:.3f} m"
        )

        self.metric_y.value_label.setText(
            f"{end_y:.3f} m"
        )

        self.end_effector_marker.setData([end_x], [end_y])

        # Orient the simulated gripper along the final link.
        previous_x, previous_y = points[-2]
        direction = np.array(
            [end_x - previous_x, end_y - previous_y],
            dtype=float
        )

        direction_norm = np.linalg.norm(direction)

        if direction_norm > 1e-9:
            direction = direction / direction_norm
            perpendicular = np.array(
                [-direction[1], direction[0]],
                dtype=float
            )

            mount_start = np.array([end_x, end_y]) - 0.22 * direction
            finger_root = np.array([end_x, end_y]) + 0.10 * direction
            finger_tip = np.array([end_x, end_y]) + 0.48 * direction

            self.tool_mount.setData(
                [mount_start[0], end_x],
                [mount_start[1], end_y]
            )

            # Narrow finger spacing while carrying the object.
            if self.object_state == "attached":
                root_gap = 0.045
                tip_gap = 0.075
            else:
                root_gap = 0.11
                tip_gap = 0.19

            finger_1_root = finger_root + root_gap * perpendicular
            finger_1_tip = finger_tip + tip_gap * perpendicular
            finger_2_root = finger_root - root_gap * perpendicular
            finger_2_tip = finger_tip - tip_gap * perpendicular

            self.gripper_curve.setData(
                [
                    finger_1_root[0], finger_1_tip[0], np.nan,
                    finger_2_root[0], finger_2_tip[0]
                ],
                [
                    finger_1_root[1], finger_1_tip[1], np.nan,
                    finger_2_root[1], finger_2_tip[1]
                ]
            )

        # Joint angles.
        degrees = np.degrees(q)

        cards = [
            self.metric_j1,
            self.metric_j2,
            self.metric_j3
        ]

        for index, value in enumerate(degrees):
            cards[index].value_label.setText(
                f"{value:.2f}°"
            )

        # Joint graph.
        elapsed = time.monotonic() - self.start_time

        if elapsed - self.last_plot_time >= 0.05:

            self.time_data.append(elapsed)

            for index in range(3):
                self.angle_data[index].append(
                    degrees[index]
                )

            self.last_plot_time = elapsed

            self.time_data = self.time_data[-300:]

            for index in range(3):
                self.angle_data[index] = (
                    self.angle_data[index][-300:]
                )

            for index in range(3):
                self.joint_curves[index].setData(
                    self.time_data,
                    self.angle_data[index]
                )

            if self.time_data:
                self.joint_plot.setXRange(
                    max(0.0, self.time_data[-1] - 15.0),
                    max(15.0, self.time_data[-1]),
                    padding=0
                )

    # --------------------------------------------------------
    # CLEAN SHUTDOWN
    # --------------------------------------------------------

    def shutdown_ros(self):

        if hasattr(self, "plot_timer") and self.plot_timer.isActive():
            self.plot_timer.stop()

        if hasattr(self, "sequence_timer") and self.sequence_timer.isActive():
            self.sequence_timer.stop()

        if self.executor is not None:
            self.executor.shutdown(timeout_sec=2.0)
            self.executor = None

        if self.ros_thread is not None:
            self.ros_thread.join(timeout=2.0)
            self.ros_thread = None

    def closeEvent(self, event):

        self.shutdown_ros()
        event.accept()


# ============================================================
# MAIN
# ============================================================

def main(args=None):

    app = QtWidgets.QApplication(sys.argv)
    app.setStyle("Fusion")
    app.setStyleSheet(DASHBOARD_STYLE)

    rclpy.init(
        args=args,
        signal_handler_options=SignalHandlerOptions.NO
    )

    node = GuiNode()

    executor = SingleThreadedExecutor()
    executor.add_node(node)

    ros_thread = threading.Thread(
        target=executor.spin,
        daemon=True
    )

    ros_thread.start()

    window = ArmWindow(
        node,
        executor,
        ros_thread
    )

    window.show()

    try:
        exit_code = app.exec_()

    finally:
        window.shutdown_ros()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

    sys.exit(exit_code)


if __name__ == "__main__":
    main()
