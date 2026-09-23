from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction, TimerAction, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def launch_setup(context, *args, **kwargs):
    ns = LaunchConfiguration("namespace")
    urdf_package = LaunchConfiguration("urdf_package")
    robot_urdf = LaunchConfiguration("robot_urdf")
    mode = LaunchConfiguration("mode")
    prefix = LaunchConfiguration("prefix")
    initial_positions_file = LaunchConfiguration("initial_positions_file")
    controllers_file = LaunchConfiguration("controllers_file")
    world_package = LaunchConfiguration("world_package")
    world_relpath = LaunchConfiguration("gz_world")

    # Controller Start/Stopp wie im zweiten Launchfile
    activate_joint_controller = LaunchConfiguration("activate_joint_controller")
    initial_joint_controller = LaunchConfiguration("initial_joint_controller")

    # Gripper controller start/stop + name
    activate_gripper_controller = LaunchConfiguration("activate_gripper_controller")
    gripper_controller_name = LaunchConfiguration("gripper_controller_name")

    # Entitätsname in Gazebo; wird auch als Namespace für controller_manager verwendet, wenn ns leer ist
    model_name = LaunchConfiguration("model_name")

    # Xacro: simulation_controllers hinzufügen, damit die Methode mit dem zweiten Launchfile übereinstimmt
    xacro_cmd = [
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        PathJoinSubstitution([FindPackageShare(urdf_package.perform(context)), "urdf", robot_urdf]), " ",
        "mode:=", mode, " ",
        "initial_positions_file:=", initial_positions_file, " ",
        "simulation_controllers:=", controllers_file,
    ]
    if prefix.perform(context) == "":
        xacro_cmd += [" ", 'prefix:=""']
    else:
        xacro_cmd += [" ", "prefix:=", prefix]
    robot_description_content = Command(xacro_cmd, on_stderr="capture")
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    rsp = Node(
        namespace=ns, package="robot_state_publisher", executable="robot_state_publisher",
        output="both", parameters=[robot_description, {"use_sim_time": True, "publish_frequency": 100.0}],
    )

    # Gazebo: Deine World lädt die Plugins bereits, daher ist -s in gz_args nicht erforderlich
    world_pkg_dir = get_package_share_directory(world_package.perform(context))
    world_path = os.path.join(world_pkg_dir, world_relpath.perform(context))
    gazebo_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])),
        launch_arguments={"gz_args": [world_path, " -r -v4"]}.items(),
    )

    # Entität erzeugen (Spawn)
    pose_args = ["-x", "0.0", "-y", "0.0", "-z", "0.0", "-R", "0.0", "-P", "0.0", "-Y", "0.0"]
    spawn = Node(
        package="ros_gz_sim", executable="create",
        arguments=["-topic", "robot_description", "-name", model_name, "-allow_renaming"] + pose_args,
        output="screen",
    )

    clock_bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge",
        parameters=[{"use_sim_time": True}],
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"], output="screen",
    )

    # Bridge Gazebo -> ROS 2: Clock + Tiefenkamera-Themen (entsprechend dem Referenz-Launchfile)
    gazebo_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{"use_sim_time": True}],
        arguments=[
            "/depth_camera/image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/depth_camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/depth_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
            "/depth_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
        ],
        output="screen",
    )

    # Controller-Spawner folgen dem Muster des zweiten Launchfiles:
    # - verwende das Argument "--controller-manager" "/controller_manager"
    # - übergib kein --param-file an den Spawner (Inhalte kommen über xacro simulation_controllers)
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    initial_joint_controller_spawner_started = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[initial_joint_controller, "--controller-manager", "/controller_manager"],
        condition=IfCondition(activate_joint_controller),
        output="screen",
    )

    initial_joint_controller_spawner_stopped = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[initial_joint_controller, "--controller-manager", "/controller_manager", "--stopped"],
        condition=UnlessCondition(activate_joint_controller),
        output="screen",
    )

    # Gripper controller spawner (start/stop wie oben)
    gripper_controller_spawner_started = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[gripper_controller_name, "--controller-manager", "/controller_manager"],
        condition=IfCondition(activate_gripper_controller),
        output="screen",
    )

    gripper_controller_spawner_stopped = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[gripper_controller_name, "--controller-manager", "/controller_manager", "--stopped"],
        condition=UnlessCondition(activate_gripper_controller),
        output="screen",
    )

    # Verzögerung, damit controller_manager bereit ist (optional; aus Launchfile 1 beibehalten)
    delayed_jsb = TimerAction(period=3.0, actions=[joint_state_broadcaster_spawner])
    delayed_initial_started = TimerAction(period=5.0, actions=[initial_joint_controller_spawner_started])
    delayed_initial_stopped = TimerAction(period=5.0, actions=[initial_joint_controller_spawner_stopped])

    # Gripper nach dem initialen Joint-Controller spawnen
    delayed_gripper_started = TimerAction(period=7.0, actions=[gripper_controller_spawner_started])
    delayed_gripper_stopped = TimerAction(period=7.0, actions=[gripper_controller_spawner_stopped])

    # Node zum Korrigieren der Transformation, falls die Frame-ID von Gazebo vom Robot State Publisher nicht erkannt wird
    '''static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_base_link_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'wrist3_link', 'depth_camera'] 
        # arguments=['0', '0', '0', '0', '0', '0', 'wrist3_link', 'camera_link']
         
        # Ersetze 'wrist3_link' durch den Namen des letzten Links in in der URDF
        # Ersetze 'depth_camera' durch die Frame-ID, die im RViz als Fehler angezeigt wird
    )'''

    return [
        rsp, gazebo_ld, spawn, gazebo_bridge, clock_bridge,
        delayed_jsb, delayed_initial_started, delayed_initial_stopped,
        #delayed_gripper_started, delayed_gripper_stopped, static_tf_node
        delayed_gripper_started, delayed_gripper_stopped
    ]

def generate_launch_description():

    # ============================================================
    # Gazebo resource paths
    # ============================================================

    pkg_fairino_gazebo = get_package_share_directory(
        'fairino_gazebo'
    )

    pkg_fairino_description = get_package_share_directory(
        'fairino_description'
    )

    pkg_schnuck_description = get_package_share_directory(
        'schnuck_fr_description'
    )

    models_path = os.path.join(
        pkg_fairino_gazebo,
        'models'
    )

    # Gazebo resolves:
    #
    # model://fairino_description/...
    #
    # relative to:
    #
    # .../share
    #
    # NOT .../share/fairino_description
    gz_resource_path = ":".join([
        models_path,
        os.path.dirname(pkg_fairino_description),
        os.path.dirname(pkg_schnuck_description),
        "/opt/ros/jazzy/share",
    ])

    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=gz_resource_path
    )

    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value=""),
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
            "model_name",
            default_value="fairino_fr5_robot"
        ),
        DeclareLaunchArgument(
            "prefix",
            default_value=""
        ),

        DeclareLaunchArgument(
            "initial_positions_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("fairino_gazebo"),
                "config",
                "initial_positions.yaml"
            ])
        ),

        DeclareLaunchArgument(
            "controllers_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("fairino_gazebo"),
                "config",
                "ros2_controllers.yaml"
            ])
        ),

        DeclareLaunchArgument(
            "world_package",
            default_value="fairino_gazebo"
        ),

        DeclareLaunchArgument(
            "gz_world",
            default_value="world/table.sdf"
        ),

        DeclareLaunchArgument(
            "activate_joint_controller",
            default_value="true"
        ),

        DeclareLaunchArgument(
            "initial_joint_controller",
            default_value="joint_trajectory_controller"
        ),

        DeclareLaunchArgument(
            "activate_gripper_controller",
            default_value="true"
        ),

        DeclareLaunchArgument(
            "gripper_controller_name",
            default_value="gripper_position_controller"
        ),

        set_gz_resource_path,

        OpaqueFunction(function=launch_setup),
    ])