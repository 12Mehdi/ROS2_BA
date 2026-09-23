from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, TimerAction,)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value=""
    )

    urdf_package_arg = DeclareLaunchArgument(
        "urdf_package",
        default_value="fairino_gazebo"
    )

    robot_urdf_arg = DeclareLaunchArgument(
        "robot_urdf",
        default_value="fairino5_v6_robot.urdf.xacro"
    )

    prefix_arg = DeclareLaunchArgument(
        "prefix",
        default_value=""
    )

    model_name_arg = DeclareLaunchArgument(
        "model_name",
        default_value="fairino_fr5_robot"
    )

    initial_positions_file_arg = DeclareLaunchArgument(
        "initial_positions_file",
        default_value=PathJoinSubstitution([
            FindPackageShare("fairino_gazebo"),
            "config",
            "initial_positions.yaml"
        ])
    )

    controllers_file_arg = DeclareLaunchArgument(
        "controllers_file",
        default_value=PathJoinSubstitution([
            FindPackageShare("fairino_gazebo"),
            "config",
            "ros2_controllers.yaml"
        ])
    )

    world_package_arg = DeclareLaunchArgument(
        "world_package",
        default_value="fairino_gazebo"
    )

    gz_world_arg = DeclareLaunchArgument(
        "gz_world",
        default_value="world/table.sdf"
    )

    activate_joint_controller_arg = DeclareLaunchArgument(
        "activate_joint_controller",
        default_value="true"
    )

    initial_joint_controller_arg = DeclareLaunchArgument(
        "initial_joint_controller",
        default_value="joint_trajectory_controller"
    )

    moveit_delay_arg = DeclareLaunchArgument(
        "moveit_delay",
        default_value="5.0"
    )

    namespace = LaunchConfiguration("namespace")
    urdf_package = LaunchConfiguration("urdf_package")
    robot_urdf = LaunchConfiguration("robot_urdf")
    prefix = LaunchConfiguration("prefix")
    model_name = LaunchConfiguration("model_name")

    initial_positions_file = LaunchConfiguration(
        "initial_positions_file"
    )

    controllers_file = LaunchConfiguration(
        "controllers_file"
    )

    world_package = LaunchConfiguration("world_package")
    gz_world = LaunchConfiguration("gz_world")

    activate_joint_controller = LaunchConfiguration(
        "activate_joint_controller"
    )

    initial_joint_controller = LaunchConfiguration(
        "initial_joint_controller"
    )

    moveit_delay = LaunchConfiguration("moveit_delay")

    # ============================================================
    # GAZEBO
    # ============================================================

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("fairino_gazebo"),
                "launch",
                "fr5_gazebo.launch.py"
            ])
        ),
        launch_arguments={
            "namespace": namespace,
            "urdf_package": urdf_package,
            "robot_urdf": robot_urdf,
            "prefix": prefix,
            "model_name": model_name,
            "initial_positions_file": initial_positions_file,
            "controllers_file": controllers_file,
            "world_package": world_package,
            "gz_world": gz_world,
            "activate_joint_controller":
                activate_joint_controller,
            "initial_joint_controller":
                initial_joint_controller,
        }.items()
    )

    # ============================================================
    # MOVEIT
    # ============================================================

    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("fairino5_v6_moveit2_config"),
                "launch",
                "fairino_moveit.launch.py"
            ])
        ),
        launch_arguments={
            "namespace": namespace,
            "urdf_package": urdf_package,
            "robot_urdf": robot_urdf,
            "prefix": prefix,
            "initial_positions_file": initial_positions_file,
            "controllers_file": controllers_file,
        }.items()
    )

    delayed_moveit_launch = TimerAction(
        period=moveit_delay,
        actions=[
            moveit_launch
        ]
    )

    return LaunchDescription([
        namespace_arg,
        urdf_package_arg,
        robot_urdf_arg,
        prefix_arg,
        model_name_arg,
        initial_positions_file_arg,
        controllers_file_arg,
        world_package_arg,
        gz_world_arg,
        activate_joint_controller_arg,
        initial_joint_controller_arg,
        moveit_delay_arg,

        gazebo_launch,

        delayed_moveit_launch,
    ])