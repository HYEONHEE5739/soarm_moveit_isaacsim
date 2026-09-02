import os
import sys
import math
import threading

import rclpy
from rclpy.node import Node

from urdf_parser_py.urdf import URDF
from ament_index_python.packages import get_package_share_directory

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QSlider, QPushButton, QGroupBox
)
from PyQt5.QtCore import Qt, QObject, pyqtSignal


# =========================
# URDF 로드 + JOINT_CONFIG 생성
# =========================
description_share = get_package_share_directory('so_arm_description')
urdf_path = os.path.join(description_share, 'urdf', 'so101_calibrated.urdf')
with open(urdf_path, 'rb') as f:
    robot = URDF.from_xml_string(f.read())

JOINT_NAMES = [
    'shoulder_pan',
    'shoulder_lift',
    'elbow_flex',
    'wrist_flex',
    'wrist_roll',
    'gripper'
]

ARM_JOINT_NAMES = JOINT_NAMES[:-1]
GRIPPER_JOINT_NAME = 'gripper'

joint_map = {joint.name: joint for joint in robot.joints}

JOINT_CONFIG = []

for joint_name in JOINT_NAMES:
    joint = joint_map.get(joint_name)

    if joint is None:
        raise ValueError(f"Joint '{joint_name}' not found in URDF")

    if joint.limit is None:
        raise ValueError(f"Joint '{joint_name}' has no limit in URDF")

    min_deg = math.floor(math.degrees(joint.limit.lower))
    max_deg = math.ceil(math.degrees(joint.limit.upper))

    JOINT_CONFIG.append({
        'name': joint.name,
        'min': min_deg,
        'max': max_deg,
    })


# =========================
# Qt <-> ROS signal bridge
# =========================
class RosSignalBridge(QObject):
    joint_states_received = pyqtSignal(dict)
    status_updated = pyqtSignal(str)


# =========================
# ROS Node
# =========================
class SoArmGuiNode(Node):
    def __init__(self, bridge: RosSignalBridge):
    """
    Description:

    Parameters
    ----------
    :param self: INSERT DESCRIPTION
    :type self: type
    :param bridge: INSERT DESCRIPTION
    :type bridge: RosSignalBridge

    .
    """
        super().__init__('so_arm_gui')

        self.bridge = bridge
        self.latest_joint_positions_deg = {}

        self.get_logger().info('SO-ARM GUI node started')

        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.arm_traj_pub = self.create_publisher(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            10
        )

        self.gripper_traj_pub = self.create_publisher(
            JointTrajectory,
            '/gripper_controller/joint_trajectory',
            10
        )

    def joint_state_callback(self, msg: JointState):
    """
    Description:

    Parameters
    ----------
    :param self: INSERT DESCRIPTION
    :type self: type
    :param msg: INSERT DESCRIPTION
    :type msg: JointState

    .
    """
        joint_dict_deg = {}

        for name, position_rad in zip(msg.name, msg.position):
            if name in JOINT_NAMES:
                joint_dict_deg[name] = round(math.degrees(position_rad))

        self.latest_joint_positions_deg = joint_dict_deg
        self.bridge.joint_states_received.emit(joint_dict_deg)

    def publish_joint_targets(self, target_deg_dict: dict, duration_sec: int = 2):
    """
    Description:

    Parameters
    ----------
    :param self: INSERT DESCRIPTION
    :type self: type
    :param target_deg_dict: INSERT DESCRIPTION
    :type target_deg_dict: dict
    :param duration_sec: INSERT DESCRIPTION
    :type duration_sec: int

    .
    """
        # arm 5축 trajectory
        arm_msg = JointTrajectory()
        arm_msg.joint_names = ARM_JOINT_NAMES.copy()

        arm_point = JointTrajectoryPoint()
        arm_point.positions = [
            math.radians(target_deg_dict[joint_name])
            for joint_name in ARM_JOINT_NAMES
        ]
        arm_point.time_from_start = Duration(sec=duration_sec, nanosec=0)
        arm_msg.points = [arm_point]

        # gripper trajectory
        gripper_msg = JointTrajectory()
        gripper_msg.joint_names = [GRIPPER_JOINT_NAME]

        gripper_point = JointTrajectoryPoint()
        gripper_point.positions = [
            math.radians(target_deg_dict[GRIPPER_JOINT_NAME])
        ]
        gripper_point.time_from_start = Duration(sec=duration_sec, nanosec=0)
        gripper_msg.points = [gripper_point]

        self.arm_traj_pub.publish(arm_msg)
        self.gripper_traj_pub.publish(gripper_msg)

        self.bridge.status_updated.emit("목표 전송 완료")
        self.get_logger().info(f'Published target(deg): {target_deg_dict}')


# =========================
# ROS Executor Thread
# =========================
class RosExecutorThread(threading.Thread):
    def __init__(self, ros_node: SoArmGuiNode):
    """
    Description:

    Parameters
    ----------
    :param self: INSERT DESCRIPTION
    :type self: type
    :param ros_node: INSERT DESCRIPTION
    :type ros_node: SoArmGuiNode

    .
    """
        super().__init__(daemon=True)
        self.ros_node = ros_node
        self.executor = rclpy.executors.SingleThreadedExecutor()
        self.executor.add_node(self.ros_node)

    def run(self):
    """
    Description:

    Parameters
    ----------
    :param self: INSERT DESCRIPTION
    :type self: type

    .
    """
        self.executor.spin()

    def stop(self):
    """
    Description:

    Parameters
    ----------
    :param self: INSERT DESCRIPTION
    :type self: type

    .
    """
        self.executor.shutdown()


# =========================
# Joint Slider Widget
# =========================
class JointSlider(QWidget):
    def __init__(self, joint_name: str, min_val: int = -180, max_val: int = 180, init_val: int = 0):
    """
    Description:

    Parameters
    ----------
    :param self: INSERT DESCRIPTION
    :type self: type
    :param joint_name: INSERT DESCRIPTION
    :type joint_name: str
    :param min_val: INSERT DESCRIPTION
    :type min_val: int
    :param max_val: INSERT DESCRIPTION
    :type max_val: int
    :param init_val: INSERT DESCRIPTION
    :type init_val: int

    .
    """
        super().__init__()

        self.joint_name = joint_name

        layout = QHBoxLayout()

        self.name_label = QLabel(joint_name)
        self.name_label.setFixedWidth(110)

        self.slider = QSlider(Qt.Horizontal)
        self.slider.setMinimum(min_val)
        self.slider.setMaximum(max_val)
        self.slider.setValue(init_val)

        self.target_label = QLabel(f"target: {init_val}")
        self.target_label.setFixedWidth(90)

        self.current_label = QLabel("current: --")
        self.current_label.setFixedWidth(90)

        self.slider.valueChanged.connect(self.on_value_changed)

        layout.addWidget(self.name_label)
        layout.addWidget(self.slider)
        layout.addWidget(self.target_label)
        layout.addWidget(self.current_label)

        self.setLayout(layout)

    def on_value_changed(self, value: int):
    """
    Description:

    Parameters
    ----------
    :param self: INSERT DESCRIPTION
    :type self: type
    :param value: INSERT DESCRIPTION
    :type value: int

    .
    """
        self.target_label.setText(f"target: {value}")

    def get_value(self) -> int:
    """
    Description:

    Parameters
    ----------
    :param self: INSERT DESCRIPTION
    :type self: type

    Returns
    -------
    :return: INSERT RETURN DESCRIPTION
    :rtype: int

    .
    """
        return self.slider.value()

    def set_target_value(self, value: int):
    """
    Description:

    Parameters
    ----------
    :param self: INSERT DESCRIPTION
    :type self: type
    :param value: INSERT DESCRIPTION
    :type value: int

    .
    """
        self.slider.setValue(value)

    def set_current_value(self, value: int):
    """
    Description:

    Parameters
    ----------
    :param self: INSERT DESCRIPTION
    :type self: type
    :param value: INSERT DESCRIPTION
    :type value: int

    .
    """
        self.current_label.setText(f"current: {value}")


# =========================
# Main Window
# =========================
class MainWindow(QWidget):
    def __init__(self, ros_node: SoArmGuiNode, bridge: RosSignalBridge):
    """
    Description:

    Parameters
    ----------
    :param self: INSERT DESCRIPTION
    :type self: type
    :param ros_node: INSERT DESCRIPTION
    :type ros_node: SoArmGuiNode
    :param bridge: INSERT DESCRIPTION
    :type bridge: RosSignalBridge

    .
    """
        super().__init__()

        self.ros_node = ros_node
        self.bridge = bridge
        self.slider_widgets = {}
        self.sliders_initialized = False

        self.setWindowTitle("SO-ARM GUI")
        self.resize(900, 450)

        main_layout = QVBoxLayout()

        title = QLabel("SO-ARM Joint Control")
        title.setStyleSheet("font-size: 20px; font-weight: bold;")
        main_layout.addWidget(title)

        self.status_label = QLabel("Status: Ready")
        main_layout.addWidget(self.status_label)

        joint_group = QGroupBox("Joint Sliders")
        joint_layout = QVBoxLayout()

        for joint_config in JOINT_CONFIG:
            slider_widget = JointSlider(
                joint_name=joint_config['name'],
                min_val=joint_config['min'],
                max_val=joint_config['max'],
                init_val=0
            )
            self.slider_widgets[joint_config['name']] = slider_widget
            joint_layout.addWidget(slider_widget)

        joint_group.setLayout(joint_layout)
        main_layout.addWidget(joint_group)

        button_layout = QHBoxLayout()

        self.print_btn = QPushButton("현재 값 출력")
        self.print_btn.clicked.connect(self.print_joint_values)

        self.sync_btn = QPushButton("현재값으로 동기화")
        self.sync_btn.clicked.connect(self.sync_sliders_to_current)

        self.send_btn = QPushButton("목표 전송")
        self.send_btn.clicked.connect(self.send_joint_values)

        button_layout.addWidget(self.print_btn)
        button_layout.addWidget(self.sync_btn)
        button_layout.addWidget(self.send_btn)

        main_layout.addLayout(button_layout)

        self.setLayout(main_layout)

        self.bridge.joint_states_received.connect(self.update_joint_states)
        self.bridge.status_updated.connect(self.update_status)

        self.load_qss()

    def load_qss(self):
    """
    Description:

    Parameters
    ----------
    :param self: INSERT DESCRIPTION
    :type self: type

    .
    """
        current_dir = os.path.dirname(os.path.abspath(__file__))
        qss_path = os.path.join(current_dir, "styles", "main.qss")

        if os.path.exists(qss_path):
            with open(qss_path, "r", encoding="utf-8") as f:
                self.setStyleSheet(f.read())

    def update_status(self, text: str):
    """
    Description:

    Parameters
    ----------
    :param self: INSERT DESCRIPTION
    :type self: type
    :param text: INSERT DESCRIPTION
    :type text: str

    .
    """
        self.status_label.setText(f"Status: {text}")

    def update_joint_states(self, joint_dict_deg: dict):
    """
    Description:

    Parameters
    ----------
    :param self: INSERT DESCRIPTION
    :type self: type
    :param joint_dict_deg: INSERT DESCRIPTION
    :type joint_dict_deg: dict

    .
    """
        self.update_status("joint_states 수신 중")

        for joint_name, current_deg in joint_dict_deg.items():
            if joint_name in self.slider_widgets:
                self.slider_widgets[joint_name].set_current_value(current_deg)

        # 첫 수신 시 target도 현재값으로 한번 맞춤
        if not self.sliders_initialized:
            for joint_name, current_deg in joint_dict_deg.items():
                if joint_name in self.slider_widgets:
                    self.slider_widgets[joint_name].set_target_value(current_deg)
            self.sliders_initialized = True
            self.update_status("초기 동기화 완료")

    def sync_sliders_to_current(self):
    """
    Description:

    Parameters
    ----------
    :param self: INSERT DESCRIPTION
    :type self: type

    .
    """
        if not self.ros_node.latest_joint_positions_deg:
            self.update_status("아직 joint_states를 못 받음")
            return

        for joint_name, current_deg in self.ros_node.latest_joint_positions_deg.items():
            if joint_name in self.slider_widgets:
                self.slider_widgets[joint_name].set_target_value(current_deg)

        self.update_status("현재값으로 동기화 완료")

    def print_joint_values(self):
    """
    Description:

    Parameters
    ----------
    :param self: INSERT DESCRIPTION
    :type self: type

    .
    """
        values = {name: widget.get_value() for name, widget in self.slider_widgets.items()}
        print(values)
        self.ros_node.get_logger().info(f'Target joint values(deg): {values}')
        self.update_status("현재 target 값 출력")

    def send_joint_values(self):
    """
    Description:

    Parameters
    ----------
    :param self: INSERT DESCRIPTION
    :type self: type

    .
    """
        target_deg_dict = {
            name: widget.get_value()
            for name, widget in self.slider_widgets.items()
        }
        self.ros_node.publish_joint_targets(target_deg_dict, duration_sec=2)

    def closeEvent(self, event):
    """
    Description:

    Parameters
    ----------
    :param self: INSERT DESCRIPTION
    :type self: type
    :param event: INSERT DESCRIPTION
    :type event: type

    .
    """
        self.update_status("종료 중...")
        super().closeEvent(event)


# =========================
# main
# =========================
def main(args=None):
    """
    Description:

    Parameters
    ----------
    :param args: INSERT DESCRIPTION
    :type args: type

    .
    """
    rclpy.init(args=args)

    bridge = RosSignalBridge()
    ros_node = SoArmGuiNode(bridge)
    ros_thread = RosExecutorThread(ros_node)
    ros_thread.start()

    app = QApplication(sys.argv)
    window = MainWindow(ros_node, bridge)
    window.show()

    exit_code = app.exec_()

    ros_thread.stop()
    ros_thread.join(timeout=1.0)

    ros_node.destroy_node()
    rclpy.shutdown()

    sys.exit(exit_code)


if __name__ == '__main__':
    main()