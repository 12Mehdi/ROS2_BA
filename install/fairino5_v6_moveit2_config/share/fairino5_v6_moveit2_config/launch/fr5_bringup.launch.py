"""Gemeinsamer Einstieg fuer FR5-Simulation und realen Roboter."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim = LaunchConfiguration("use_sim")

    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("fairino_gazebo"),
                    "launch",
                    "fr5_gazebo_moveit_base.launch.py",
                ]
            )
        ),
        condition=IfCondition(use_sim),
        launch_arguments={
            "gz_world": LaunchConfiguration("gz_world"),
            "moveit_delay": LaunchConfiguration("moveit_delay"),
        }.items(),
    )

    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("fairino5_v6_moveit2_config"),
                    "launch",
                    "fairino_hardware.launch.py",
                ]
            )
        ),
        condition=UnlessCondition(use_sim),
        launch_arguments={
            "vision_mode": LaunchConfiguration("vision_mode"),
            "reference_map": LaunchConfiguration("reference_map"),
            "merge_distance": LaunchConfiguration("merge_distance"),
            "downsample_map": LaunchConfiguration("downsample_map"),
            "voxel_size": LaunchConfiguration("voxel_size"),
            "table_z_pos": LaunchConfiguration("table_z_pos"),
            "moveit_delay": LaunchConfiguration("moveit_delay"),
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim",
                default_value="true",
                choices=["true", "false"],
                description=(
                    "true: Gazebo + simuliertes ros2_control; "
                    "false: realer FR5 + FairinoHardwareInterface"
                ),
            ),
            DeclareLaunchArgument(
                "gz_world",
                default_value="world/table.sdf",
                description="Gazebo-World; wird nur bei use_sim:=true verwendet.",
            ),
            DeclareLaunchArgument(
                "moveit_delay",
                default_value="5.0",
                description="Verzoegerung vor dem Start von MoveIt in Sekunden.",
            ),
            DeclareLaunchArgument(
                "vision_mode",
                default_value="mapping",
                choices=["mapping", "detecting"],
                description="Vision-Modus; wird nur bei use_sim:=false verwendet.",
            ),
            DeclareLaunchArgument("reference_map", default_value=""),
            DeclareLaunchArgument("merge_distance", default_value=""),
            DeclareLaunchArgument("downsample_map", default_value="true"),
            DeclareLaunchArgument("voxel_size", default_value="0.005"),
            DeclareLaunchArgument("table_z_pos", default_value="-0.3"),
            simulation_launch,
            hardware_launch,
        ]
    )
