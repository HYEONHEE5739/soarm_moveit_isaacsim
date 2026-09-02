import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import (
    generate_static_virtual_joint_tfs_launch,
    generate_rsp_launch,
    generate_move_group_launch,
    generate_moveit_rviz_launch,
    generate_spawn_controllers_launch,
)


def add_launch_description(target_ld, source_ld):
    """
    Description:

    Parameters
    ----------
    :param target_ld: INSERT DESCRIPTION
    :type target_ld: type
    :param source_ld: INSERT DESCRIPTION
    :type source_ld: type

    .
    """
    for action in source_ld.entities:
        target_ld.add_action(action)


def generate_launch_description():
    """
    Description:

    .
    """
    description_pkg = get_package_share_directory("so_arm_description")
    moveit_config_pkg = get_package_share_directory("so_arm_moveit_config")

    sim_xacro = os.path.join(
        description_pkg,
        "urdf",
        "so101_sim.urdf.xacro",
    )

    srdf_file = os.path.join(
        moveit_config_pkg,
        "config",
        "so101_calibrated.srdf",
    )

    ros2_controllers_file = os.path.join(
        moveit_config_pkg,
        "config",
        "ros2_controllers.yaml",
    )

    moveit_config = (
        MoveItConfigsBuilder(
            "so101_calibrated",
            package_name="so_arm_moveit_config",
        )
        .robot_description(file_path=sim_xacro)
        .robot_description_semantic(file_path=srdf_file)
        .to_moveit_configs()
    )

    ld = LaunchDescription()

    # Fixed/base TF
    add_launch_description(
        ld,
        generate_static_virtual_joint_tfs_launch(moveit_config),
    )

    # Robot State Publisher
    add_launch_description(
        ld,
        generate_rsp_launch(moveit_config),
    )

    # Move Group (with topic_based backend)
    add_launch_description(
        ld,
        generate_move_group_launch(moveit_config),
    )

    # RViz
    add_launch_description(
        ld,
        generate_moveit_rviz_launch(moveit_config),
    )

    # ros2_control_node
    # so101_sim.urdf.xacro includes so101_sim.ros2_control.xacro
    # which uses topic_based_ros2_control/TopicBasedSystem plugin
    ld.add_action(
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                moveit_config.robot_description,
                ros2_controllers_file,
            ],
            output="screen",
            emulate_tty=True,
        )
    )

    # Controller Spawner
    add_launch_description(
        ld,
        generate_spawn_controllers_launch(moveit_config),
    )

    return ld