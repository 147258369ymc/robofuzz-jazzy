#!/usr/bin/env python3
# Copyright 2026 RoboFuzz Jazzy adaptation.
#
# Minimal TurtleBot4 simulation spawn for fuzzing. This mirrors the upstream
# turtlebot4_spawn.launch.py surface that RoboFuzz needs, but intentionally
# does not spawn the standard dock. Starting docked can trigger Create3
# autonomous docking/undocking behavior, which makes /cmd_vel no longer the
# sole motion source and pollutes command/odom oracles.

from ament_index_python.packages import get_package_share_directory

from irobot_create_common_bringup.namespace import GetNamespacedName
from irobot_create_common_bringup.offset import OffsetParser

import sys

from launch import LaunchDescription, LaunchService
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace


ARGUMENTS = [
    DeclareLaunchArgument("rviz", default_value="false",
                          choices=["true", "false"],
                          description="Start rviz."),
    DeclareLaunchArgument("use_sim_time", default_value="true",
                          choices=["true", "false"],
                          description="Use simulation time."),
    DeclareLaunchArgument("model", default_value="standard",
                          choices=["standard", "lite"],
                          description="TurtleBot4 model."),
    DeclareLaunchArgument("namespace", default_value="",
                          description="Robot namespace."),
    DeclareLaunchArgument("world", default_value="simple",
                          description="Gazebo world name."),
]

for pose_element in ["x", "y", "z", "yaw"]:
    ARGUMENTS.append(
        DeclareLaunchArgument(
            pose_element,
            default_value="0.0",
            description=f"{pose_element} component of the robot pose.",
        )
    )


def generate_launch_description():
    pkg_turtlebot4_gz_bringup = get_package_share_directory(
        "turtlebot4_gz_bringup")
    pkg_turtlebot4_description = get_package_share_directory(
        "turtlebot4_description")
    pkg_irobot_create_common_bringup = get_package_share_directory(
        "irobot_create_common_bringup")
    pkg_irobot_create_gz_bringup = get_package_share_directory(
        "irobot_create_gz_bringup")

    turtlebot4_ros_gz_bridge_launch = PathJoinSubstitution(
        [pkg_turtlebot4_gz_bringup, "launch", "ros_gz_bridge.launch.py"])
    turtlebot4_node_launch = PathJoinSubstitution(
        [pkg_turtlebot4_gz_bringup, "launch", "turtlebot4_nodes.launch.py"])
    create3_nodes_launch = PathJoinSubstitution(
        [pkg_irobot_create_common_bringup, "launch", "create3_nodes.launch.py"])
    create3_gz_nodes_launch = PathJoinSubstitution(
        [pkg_irobot_create_gz_bringup, "launch", "create3_gz_nodes.launch.py"])
    robot_description_launch = PathJoinSubstitution(
        [pkg_turtlebot4_description, "launch", "robot_description.launch.py"])

    param_file_cmd = DeclareLaunchArgument(
        "param_file",
        default_value=PathJoinSubstitution(
            [pkg_turtlebot4_gz_bringup, "config", "turtlebot4_node.yaml"]),
        description="TurtleBot4 node parameter file.",
    )

    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")
    yaw = LaunchConfiguration("yaw")
    world = LaunchConfiguration("world")
    turtlebot4_node_yaml_file = LaunchConfiguration("param_file")

    robot_name = GetNamespacedName(namespace, "turtlebot4")
    # Keep a dummy dock name so bridge/toolbox nodes retain their normal
    # interface shape, but no model with this name is spawned.
    dock_name = GetNamespacedName(namespace, "robofuzz_no_dock")
    z_robot = OffsetParser(z, -0.0025)

    spawn_robot_group_action = GroupAction([
        PushRosNamespace(namespace),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([robot_description_launch]),
            launch_arguments=[
                ("model", LaunchConfiguration("model")),
                ("use_sim_time", use_sim_time),
            ],
        ),

        Node(
            package="ros_gz_sim",
            executable="create",
            arguments=[
                "-name", robot_name,
                "-x", x,
                "-y", y,
                "-z", z_robot,
                "-Y", yaw,
                "-topic", "robot_description",
            ],
            output="screen",
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([turtlebot4_ros_gz_bridge_launch]),
            launch_arguments=[
                ("model", LaunchConfiguration("model")),
                ("robot_name", robot_name),
                ("dock_name", dock_name),
                ("namespace", namespace),
                ("world", world),
            ],
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([turtlebot4_node_launch]),
            launch_arguments=[
                ("model", LaunchConfiguration("model")),
                ("param_file", turtlebot4_node_yaml_file),
            ],
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([create3_nodes_launch]),
            launch_arguments=[("namespace", namespace)],
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([create3_gz_nodes_launch]),
            launch_arguments=[
                ("robot_name", robot_name),
                ("dock_name", dock_name),
            ],
        ),

        Node(
            name="rplidar_stf",
            package="tf2_ros",
            executable="static_transform_publisher",
            output="screen",
            arguments=[
                "0", "0", "0", "0", "0", "0.0",
                "rplidar_link", [robot_name, "/rplidar_link/rplidar"],
            ],
            remappings=[
                ("/tf", "tf"),
                ("/tf_static", "tf_static"),
            ],
        ),

        Node(
            name="camera_stf",
            package="tf2_ros",
            executable="static_transform_publisher",
            output="screen",
            arguments=[
                "0", "0", "0",
                "1.5707", "-1.5707", "0",
                "oakd_rgb_camera_optical_frame",
                [robot_name, "/oakd_rgb_camera_frame/rgbd_camera"],
            ],
            remappings=[
                ("/tf", "tf"),
                ("/tf_static", "tf_static"),
            ],
        ),
    ])

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(param_file_cmd)
    ld.add_action(spawn_robot_group_action)
    return ld


def main():
    launch_service = LaunchService(argv=sys.argv[1:])
    launch_service.include_launch_description(generate_launch_description())
    return launch_service.run()


if __name__ == "__main__":
    sys.exit(main())
