#!/usr/bin/env python3
"""
handeye_calibration.launch.py

Full launch for automated eye-in-hand calibration of the D405 on the FR5.

Starts:
  1) RealSense D405 camera (via D405_sam6d.launch.py)
  2) FR5 real robot + MoveIt (via fairino5_v6_moveit2_config demo.launch.py)
  3) handeye_auto_calib node (after 12 s, once MoveIt is ready)

Resolution notes:
  Higher colour resolution → finer ChArUco corner localisation → better calibration accuracy.
  The calib node reads K from camera_info, which the RealSense driver publishes correctly for
  whatever resolution is active — no code changes needed in the calib node.

  D405 colour sensor:  max 1280×720  (default for this launch)
  D405 depth sensor:   max 1280×720   (depth not used during calibration, but must be a
                                        valid profile; can be lower than colour)

  Override example:
    ros2 launch fairino_gazebo handeye_calibration.launch.py \\
        color_profile:=1280x720x30 depth_profile:=1280x720x30

Workflow before running:
  1. Place the ChArUco board on the table (fixed, board must not move during calibration)
  2. Record ~15 calibration poses with the robot at diverse angles/distances:
       ros2 run fairino_gazebo save_robot_pose.py \\
           --output /home/simon/ros2_ws/calib_poses.yaml \\
           --name calib_1
     Repeat for calib_2 … calib_N
  3. ros2 launch fairino_gazebo handeye_calibration.launch.py
  4. After completion, paste the printed xacro:property values into
       src/fr5_workcell_description/urdf/fr5_D405_camera_with_table.urdf.xacro
  5. Rebuild: colcon build --packages-select fr5_workcell_description
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:

    declared_arguments = [
        DeclareLaunchArgument(
            "robot_ip",
            default_value="192.168.58.22",
            description="IP address of the FR5 controller."),
        DeclareLaunchArgument(
            "calibration_poses_file",
            default_value="/home/ubuntu/fairino_ws/calib_poses.yaml",
            description="YAML file with pre-recorded calibration poses (saved_poses format)."),
        DeclareLaunchArgument(
            "params_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("fairino_gazebo"), "config", "handeye_calib_params.yaml"
            ]),
            description="Node parameter file (ChArUco dimensions, frame IDs, etc.)."),
        DeclareLaunchArgument(
            "output_file",
            default_value="/home/ubuntu/fairino_ws/handeye_result.yaml",
            description="Path where the calibration result YAML is written."),
        DeclareLaunchArgument(
            "color_profile",
            default_value="1280×720x30",
            description=(
                "RealSense colour stream profile (WIDTHxHEIGHTxFPS). "
                "Higher resolution improves ChArUco corner accuracy. "
                "D405 colour max: 1280×720. "
                "The calib node reads K from camera_info, so no other changes are needed."
            )),
        DeclareLaunchArgument(
            "depth_profile",
            default_value="1280x720x30",
            description=(
                "RealSense depth stream profile (WIDTHxHEIGHTxFPS). "
                "Depth is not used during calibration; this sets the driver to a valid profile. "
                "D405 depth sensor max: 1280x720."
            )),
    ]

    # ── 1. RealSense D405 ─────────────────────────────────────────────────────
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("realsense2_camera"),
                "launch",
                "rs_launch.py"
            ])
        ]),
        launch_arguments={
            "rviz":          "false",
            "color_profile": LaunchConfiguration("color_profile"),
            "depth_profile": LaunchConfiguration("depth_profile"),
        }.items(),
    )

    # ── 2. FR5 real robot + MoveIt ────────────────────────────────────────────
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("fairino5_v6_moveit2_config"),
                "launch",
                "demo.launch.py"
            ])
        ])
    )

    # ── 3. handeye_auto_calib node ────────────────────────────────────────────
    # Delayed so that MoveIt / move_group is fully up before the node starts
    # sending motion goals.
    calib_node = TimerAction(
        period=12.0,
        actions=[Node(
            package="fairino_gazebo",
            executable="handeye_auto_calib.py",
            name="handeye_auto_calib",
            output="screen",
            parameters=[
                LaunchConfiguration("params_file"),
                {
                    "calibration_poses_file": LaunchConfiguration("calibration_poses_file"),
                    "output_file":            LaunchConfiguration("output_file"),
                },
            ],
        )],
    )

    return LaunchDescription(
        declared_arguments + [
            camera_launch,
            moveit_launch,
            calib_node,
        ]
    )
