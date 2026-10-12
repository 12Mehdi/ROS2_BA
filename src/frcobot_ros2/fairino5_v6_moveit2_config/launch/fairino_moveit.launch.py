from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml

# Zusätzlich: Überprüfen Sie den controller_manager vor dem Start von MoveIt


def require_path(path: str, what: str):
    """Überprüfen, ob der erforderliche Dateipfad existiert"""
    if not os.path.exists(path):
        raise FileNotFoundError(f"[fairino_moveit] {what} wurde nicht gefunden: {path}")


def launch_setup(context, *args, **kwargs):
    # LaunchConfiguration-Variablen abrufen
    ns = LaunchConfiguration("namespace")
    urdf_package = LaunchConfiguration("urdf_package")
    robot_urdf = LaunchConfiguration("robot_urdf")
    mode = LaunchConfiguration("mode")
    prefix = LaunchConfiguration("prefix")
    initial_positions_file = LaunchConfiguration("initial_positions_file")
    controllers_file = LaunchConfiguration("controllers_file")
    moveit_config_package = LaunchConfiguration("moveit_config_package")
    rviz_config_file = LaunchConfiguration("rviz_config_file")

    # Werte aus dem Kontext extrahieren
    ns_val = ns.perform(context)
    urdf_pkg_val = urdf_package.perform(context)
    robot_urdf_val = robot_urdf.perform(context)
    mode_val = mode.perform(context)
    prefix_val = prefix.perform(context)
    init_pos_path = initial_positions_file.perform(context)
    controllers_path = controllers_file.perform(context)
    moveit_cfg_pkg_val = moveit_config_package.perform(context)
    rviz_cfg_path = rviz_config_file.perform(context)

    # TF-Präfix basierend auf Namespace festlegen
    tf_prefix = "" if ns_val == "" else f"{ns_val}_"

    # URDF-Pfad ermitteln und überprüfen
    urdf_pkg_share = get_package_share_directory(urdf_pkg_val)
    urdf_path = os.path.join(urdf_pkg_share, "urdf", robot_urdf_val)
    require_path(urdf_path, "URDF/Xacro")

    # MoveIt-Konfigurationspfade ermitteln und überprüfen
    moveit_cfg_share = get_package_share_directory(moveit_cfg_pkg_val)
    srdf_path = os.path.join(moveit_cfg_share, "config", "fairino5_v6_robot_gazebo.srdf")
    kinematics_path = os.path.join(moveit_cfg_share, "config", "kinematics_gazebo.yaml")
    joint_limits_path = os.path.join(moveit_cfg_share, "config", "joint_limits.yaml")
    moveit_controllers_path = os.path.join(moveit_cfg_share, "config", "moveit_controllers.yaml")
    pilz_cartesian_limits_path = os.path.join(moveit_cfg_share, "config", "pilz_cartesian_limits.yaml")

    require_path(srdf_path, "SRDF")
    require_path(kinematics_path, "kinematics.yaml")
    require_path(joint_limits_path, "joint_limits.yaml")
    require_path(moveit_controllers_path, "moveit_controllers.yaml")

    # MoveIt-Konfiguration mit MoveItConfigsBuilder erstellen
    from moveit_configs_utils import MoveItConfigsBuilder
    moveit_config = (
        MoveItConfigsBuilder(robot_name="fairino5_v6", package_name=moveit_cfg_pkg_val)
        .robot_description(
            file_path=urdf_path,
            mappings={
                "mode": mode_val,
                "initial_positions_file": init_pos_path,
                "simulation_controllers": controllers_path,
                "prefix": prefix_val,
                "tf_prefix": tf_prefix,  # Achtung: Stellen Sie sicher, dass SRDF/Kinematics konsistent sind, wenn tf_prefix/prefix verwendet wird
            },
        )
        .robot_description_semantic(file_path=srdf_path)
        .robot_description_kinematics(file_path=kinematics_path)  # Für move_group
        .trajectory_execution(file_path=moveit_controllers_path)
        .planning_scene_monitor(publish_robot_description=True, publish_robot_description_semantic=True)
        .joint_limits(file_path=joint_limits_path)
        .planning_pipelines(
            pipelines=["ompl", "chomp", "stomp", "pilz_industrial_motion_planner", "ompl_raw"],
            default_planning_pipeline="ompl",
        )
        .to_moveit_configs()
    )

    # Pilz-Kartesische-Limits manuell laden (optional)
    cartesian_limits_param = None
    if os.path.exists(pilz_cartesian_limits_path):
        with open(pilz_cartesian_limits_path, "r") as f:
            loaded_yaml = yaml.safe_load(f) or {}
        cartesian_limits_param = loaded_yaml.get("cartesian_limits", loaded_yaml)

    # Kinematics.yaml manuell als Parameter robot_description_kinematics laden
    with open(kinematics_path, "r") as f:
        kin_yaml = yaml.safe_load(f) or {}
    if "robot_description_kinematics" not in kin_yaml:
        kin_yaml = {"robot_description_kinematics": kin_yaml}

    # Robot State Publisher Node erstellen
    '''robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=ns_val if ns_val else None,
        output="screen",
        parameters=[moveit_config.robot_description, {"use_sim_time": True}],
    )'''

    # Move Group Parameter vorbereiten (ini yang untuk gerakin lewat cpp atau external)
    move_group_params = [
        moveit_config.to_dict(),
        {"use_sim_time": True},
        {"publish_planning_scene_hz": 30.0},
        {"allow_trajectory_execution": True},
        {"publish_planning_scene": True},
        {"publish_state_updates": True},
        {"publish_transforms_updates": True},
        kin_yaml
    ]
    if cartesian_limits_param is not None:
        move_group_params.append({"cartesian_limits": cartesian_limits_param})

    # Move Group Node erstellen
    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        namespace=ns_val if ns_val else None,
        output="screen",
        parameters=move_group_params,
    )

    # WICHTIG: robot_description + semantic + kinematics auch an RViz übergeben (ini yang untuk gerakin lewat rviz + moveit)
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_cfg_path, "--ros-args", "--log-level", "error"],
        parameters=[
            {"use_sim_time": True},
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            kin_yaml,  # Damit das MotionPlanning-Plugin Zugriff auf die IK-Konfiguration hat
        ],
    )

    # Verzögerte Node-Starts zur Sicherstellung der richtigen Reihenfolge
    #delayed_rsp = TimerAction(period=1.0, actions=[robot_state_publisher])
    delayed_move_group = TimerAction(period=3.0, actions=[move_group])
    delayed_rviz = TimerAction(period=4.0, actions=[rviz])  # Etwas längere Pause für Parameter-Publishing

    # Node zum Korrigieren der Transformation, falls die Frame-ID von Gazebo vom Robot State Publisher nicht erkannt wird
    '''static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_base_link_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'wrist3_link', 'depth_camera'] 
        # Ersetze 'link_6' durch den Namen des letzten Links in in der URDF
        # Ersetze 'depth_camera' durch die Frame-ID, die im RViz als Fehler angezeigt wird
    )'''

    #return [delayed_rsp, delayed_move_group, delayed_rviz, static_tf_node]
    return [delayed_move_group, delayed_rviz]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "namespace",
            default_value="",
            description="Namespace des Roboters (optional)."
        ),

        DeclareLaunchArgument(
            "urdf_package",
            default_value="fairino_gazebo"
        ),

        DeclareLaunchArgument(
            "robot_urdf",
            default_value="fairino5_v6_robot.urdf.xacro"
        ),

        DeclareLaunchArgument(
            "mode",
            default_value="gazebo"
        ),

        DeclareLaunchArgument(
            "prefix",
            default_value=""
        ),

        DeclareLaunchArgument(
            "initial_positions_file",
            default_value=os.path.join(
                get_package_share_directory("fairino_gazebo"),
                "config",
                "initial_positions.yaml"
            )
        ),

        DeclareLaunchArgument(
            "controllers_file",
            default_value=os.path.join(
                get_package_share_directory("fairino_gazebo"),
                "config",
                "ros2_controllers.yaml"
            )
        ),

        DeclareLaunchArgument(
            "moveit_config_package",
            default_value="fairino5_v6_moveit2_config"
        ),

        DeclareLaunchArgument(
            "rviz_config_file",
            default_value=os.path.join(
                get_package_share_directory(
                    "fairino5_v6_moveit2_config"
                ),
                "config",
                "moveit.rviz"
            )
        ),

        OpaqueFunction(function=launch_setup),
    ])