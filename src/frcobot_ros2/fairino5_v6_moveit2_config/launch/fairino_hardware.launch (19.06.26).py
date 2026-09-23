import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare

def launch_nodes(context, *args, **kwargs):
    vision_mode = LaunchConfiguration('vision_mode').perform(context)
    reference_map = LaunchConfiguration('reference_map').perform(context)
    merge_distance_arg = LaunchConfiguration('merge_distance').perform(context)

    # arg baru
    downsample_map_arg = LaunchConfiguration('downsample_map').perform(context)
    voxel_size_arg = LaunchConfiguration('voxel_size').perform(context)
    table_z_pos_arg = LaunchConfiguration('table_z_pos').perform(context)

    # Parsing downsample_map (string -> bool)
    downsample_map = str(downsample_map_arg).lower() in ["true", "1", "yes", "on"]

    # Parsing voxel_size (string -> float, dengan default 0.005)
    # try and except supaya launch tidak crash kalau nilai voxel_size_arg tidak valid saat dikonversi ke float
    try:
        voxel_size = float(voxel_size_arg) if voxel_size_arg not in ["", "None"] else 0.005
    except Exception:
        voxel_size = 0.005

    # Parsing table_z_pos (string -> float, default -0.3)
    try:
        table_z_pos = float(table_z_pos_arg) if table_z_pos_arg not in ["", "None"] else -0.3
    except Exception:
        table_z_pos = -0.3

    # Resolusi path PCD berbasis package agar tidak pakai path absolut
    try:
        depth_share_dir = get_package_share_directory('depth_camera_vision')
        pcd_dir = os.path.join(depth_share_dir, 'PCD')
    except Exception as e:
        # Fallback jika pencarian package gagal (tetap minimal perubahan)
        print(f"[LaunchFile] Warning: gagal menemukan package 'depth_camera_vision': {e}. Menggunakan fallback path.")
        #pcd_dir = "/home/ubuntu/fairino_ws/src/depth_camera_vision/PCD"
        pcd_dir = "/ros2_ws/fairino_ws/src/depth_camera_vision/PCD"


    # Nilai default/reference_map: gunakan file di folder PCD dalam package
    if not reference_map or reference_map in ["", "None"]:
        reference_map_path = os.path.join(pcd_dir, "reference_map.pcd")
    elif not os.path.isabs(reference_map):
        reference_map_path = os.path.join(pcd_dir, reference_map)
    else:
        # Jika user memberi path absolut, langsung pakai
        reference_map_path = reference_map

    # Pilihan default merge_distance
    is_merge_distance_set = bool(merge_distance_arg and merge_distance_arg not in ["", "None"])
    try:
        merge_distance_value = float(merge_distance_arg) if is_merge_distance_set else None
    except Exception:
        merge_distance_value = None

    nodes = []

    # Node map_builder_node (mapping vision_mode, default merge_distance 0.0005)
    mapping_merge_distance = merge_distance_value if is_merge_distance_set and vision_mode == "mapping" else 0.0005
    map_builder_node_mapping = Node(
        package="depth_camera_vision",
        executable="map_builder_node",
        name="world_map_node",
        output="screen",
        parameters=[
            {'camera_topic': '/camera/camera/depth/color/points'},
            {'camera_frame': 'camera_depth_optical_frame'},
            {'reference_frame': 'base_link'},
            {"avoid_double_points": True},
            {"merge_distance": mapping_merge_distance},
            {"downsample_map": downsample_map},
            {"voxel_size": voxel_size},
            {"table_z_pos": table_z_pos},
        ]
    )

    # Node map_builder_node (detecting vision_mode, default merge_distance 0.00005)
    detecting_merge_distance = merge_distance_value if is_merge_distance_set and vision_mode == "detecting" else 0.00005
    map_builder_node_detecting = Node(
        package="depth_camera_vision",
        executable="map_builder_node",
        name="world_map_node_detecting",
        output="screen",
        parameters=[
            {'camera_topic': '/camera/camera/depth/color/points'},
            {'camera_frame': 'camera_depth_optical_frame'},
            {'reference_frame': 'base_link'},
            {"output_topic": "/environment_current/points"},
            {"avoid_double_points": True},
            {"merge_distance": detecting_merge_distance},
            {"downsample_map": downsample_map},
            {"voxel_size": voxel_size},
            {"table_z_pos": table_z_pos},
        ]
    )

    # Node static_environment_node: baca reference_map saja, merge_distance bukan untuk node ini
    static_environment_node = Node(
        package="depth_camera_vision",
        executable="static_environment_node",
        name="static_environment_node",
        output="screen",
        parameters=[
            {"pcd_file": reference_map_path},
            {"source_frame": "base_link"},
            {"target_frame": "base_link"},
            {"output_topic": "/environment_reference/points"}
        ]
    )

    # ===== Tambahan: Node map_diff_node jika vision_mode detecting =====
    map_diff_node = Node(
        package="depth_camera_vision",
        executable="map_diff_node",           # Nama executable seperti dipanggil lewat ros2 run
        name="map_diff_node",
        output="screen"
    )

    if vision_mode == "mapping":
        nodes.append(map_builder_node_mapping)
    elif vision_mode == "detecting":
        nodes.append(static_environment_node)
        nodes.append(map_builder_node_detecting)
        nodes.append(map_diff_node)  # <--------- PENTING: Tambahkan node map_diff_node di vision_mode detecting!
    else:
        print(f"[LaunchFile] Warning: Unknown vision_mode '{vision_mode}', no custom nodes launched.")

    return nodes


def generate_launch_description():
    # Argumen dari launchfile pertama (dipertahankan)
    namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Namespace untuk robot"
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

    # Argumen baru untuk kontrol downsampling map
    downsample_map_arg = DeclareLaunchArgument(
        "downsample_map", default_value="true",
        description="Aktifkan downsampling map di map_builder_node (true/false)."
    )
    voxel_size_arg = DeclareLaunchArgument(
        "voxel_size", default_value="0.005",
        description="Ukuran voxel (meter) untuk downsampling map_builder_node."
    )

    table_z_pos_arg = DeclareLaunchArgument(
        "table_z_pos", default_value="-0.3",
        description="Posisi meja di koordinat Z"
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

    moveit_config_package_arg = DeclareLaunchArgument(
        "moveit_config_package",
        default_value="fairino5_v6_moveit2_config"
    )

    rviz_config_file_arg = DeclareLaunchArgument(
        "rviz_config_file",
        default_value=PathJoinSubstitution([
            FindPackageShare("fairino5_v6_moveit2_config"),
            "config",
            "moveit.rviz"
        ])
    )

    controller_manager_ns_arg = DeclareLaunchArgument(
        "controller_manager_ns",
        default_value="/controller_manager"
    )

    require_controller_manager_arg = DeclareLaunchArgument(
        "require_controller_manager",
        default_value="true"
    )

    controller_manager_timeout_arg = DeclareLaunchArgument(
        "controller_manager_timeout",
        default_value="10.0"
    )

    # Delay untuk memastikan MoveIt start setelah Gazebo
    moveit_delay_arg = DeclareLaunchArgument(
        "moveit_delay",
        default_value="1.0",
        description="Delay (detik) sebelum start MoveIt setelah Gazebo"
    )

    # Argumen tambahan dari launchfile kedua (mode + vision)
    mode_arg = DeclareLaunchArgument(
        "vision_mode",
        default_value="mapping",
        description="Pilih 'mapping' atau 'detecting'."
    )

    reference_map_arg = DeclareLaunchArgument(
        "reference_map",
        default_value="",
        description="Nama file .pcd untuk environment reference (default: reference_map.pcd di dalam package depth_camera_vision/PCD)"
    )

    merge_distance_arg = DeclareLaunchArgument(
        "merge_distance",
        default_value="",
        description="merge_distance untuk map_builder_node. (default mapping: 0.0005, default detecting: 0.00005)"
    )

    # LaunchConfiguration untuk argumen yang digunakan di include
    namespace = LaunchConfiguration("namespace")
    urdf_package = LaunchConfiguration("urdf_package")
    robot_urdf = LaunchConfiguration("robot_urdf")
    prefix = LaunchConfiguration("prefix")
    model_name = LaunchConfiguration("model_name")
    initial_positions_file = LaunchConfiguration("initial_positions_file")
    controllers_file = LaunchConfiguration("controllers_file")
    world_package = LaunchConfiguration("world_package")
    gz_world = LaunchConfiguration("gz_world")
    activate_joint_controller = LaunchConfiguration("activate_joint_controller")
    initial_joint_controller = LaunchConfiguration("initial_joint_controller")
    moveit_config_package = LaunchConfiguration("moveit_config_package")
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    controller_manager_ns = LaunchConfiguration("controller_manager_ns")
    require_controller_manager = LaunchConfiguration("require_controller_manager")
    controller_manager_timeout = LaunchConfiguration("controller_manager_timeout")
    moveit_delay = LaunchConfiguration("moveit_delay")

    # # Gazebo-Launch
    # gazebo_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         PathJoinSubstitution([
    #             FindPackageShare("fairino_gazebo"),
    #             "launch",
    #             "fr5_gazebo.launch.py"
    #         ])
    #     ]),
    #     launch_arguments={
    #         "namespace": namespace,
    #         "urdf_package": urdf_package,
    #         "robot_urdf": robot_urdf,
    #         "prefix": prefix,
    #         "model_name": model_name,
    #         "initial_positions_file": initial_positions_file,
    #         "controllers_file": controllers_file,
    #         "world_package": world_package,
    #         "gz_world": gz_world,
    #         "activate_joint_controller": activate_joint_controller,
    #         "initial_joint_controller": initial_joint_controller,
    #     }.items()
    # )

    # # MoveIt-Launch
    # moveit_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         PathJoinSubstitution([
    #             FindPackageShare("fairino5_v6_moveit2_config"),
    #             "launch",
    #             "fairino_moveit.launch.py"
    #         ])
    #     ]),
    #     launch_arguments={
    #         "namespace": namespace,
    #         "urdf_package": urdf_package,
    #         "robot_urdf": robot_urdf,
    #         "prefix": prefix,
    #         "initial_positions_file": initial_positions_file,
    #         "controllers_file": controllers_file,
    #         "moveit_config_package": moveit_config_package,
    #         "rviz_config_file": rviz_config_file,
    #         "controller_manager_ns": controller_manager_ns,
    #         "require_controller_manager": require_controller_manager,
    #         "controller_manager_timeout": controller_manager_timeout,
    #         # Opsional: aktifkan sim time untuk konsistensi dengan Gazebo
    #         "use_sim_time": "true",
    #     }.items()
    # )

    # Node zum Korrigieren der Transformation, falls die Frame-ID von Gazebo vom Robot State Publisher nicht erkannt wird
    # Buat nanti 
    # static_tf_node = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='camera_base_link_tf',
    #     arguments=['0.013', '-0.065', '-0.11', '-1.571', '0', '-1.571', 'tool0', 'camera_link'] 
    #     # ist schon kalibriert
    # )

    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_base_link_tf',
        arguments=['0.013', '-0.065', '-0.13', '-1.5716', '0', '-3.1416', 'tool0', 'camera_link'] 
        # ist schon kalibriert
    )

    # MoveIt-Launch
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("fairino5_v6_moveit2_config"),
                "launch",
                "demo.launch.py"
            ])
        ])
    )

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("realsense2_camera"),
                "launch",
                "rs_launch.py"
            ])
        ]),
        launch_arguments={
            "align_depth.enable": "true",
            "pointcloud.enable": "true",
            "filters": "decimation,spatial,temporal,hole_filling",
            "clip_distance": "0.4",
        }.items()
    )

    decimation_param = SetParameter(
        name="/camera/camera/decimation_filter.enable",
        value=True
    )
    spatial_param = SetParameter(
        name="/camera/camera/spatial_filter.enable",
        value=True
    )
    temporal_param = SetParameter(
        name="/camera/camera/temporal_filter.enable",
        value=True
    )
    hole_filling_param = SetParameter(
        name="/camera/camera/hole_filling_filter.enable",
        value=True
    )

    delayed_params = TimerAction(
        period=5.0,  # detik
        actions=[decimation_param, spatial_param, temporal_param, hole_filling_param]
    )

    # TimerAction untuk delay MoveIt
    delayed_moveit_launch = TimerAction(
        period=moveit_delay,
        actions=[moveit_launch]
    )
    # OpaqueFunction untuk meluncurkan node vision dinamis (mapping/detecting)
    dynamic_nodes = OpaqueFunction(function=launch_nodes)

    return LaunchDescription([
        # Deklarasi semua argumen
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
        moveit_config_package_arg,
        rviz_config_file_arg,
        controller_manager_ns_arg,
        require_controller_manager_arg,
        controller_manager_timeout_arg,
        moveit_delay_arg,
        # Argumen tambahan (mode vision)
        mode_arg,
        reference_map_arg,
        merge_distance_arg,
        table_z_pos_arg, 
        # Argumen untuk downsampling map (INI YANG KURANG)
        downsample_map_arg,
        voxel_size_arg,
        
        realsense_launch,
        delayed_params,
        # Jalankan static TF juga
        static_tf_node,

        # Gazebo terlebih dahulu
        # gazebo_launch,
        # Lalu MoveIt setelah delay
        delayed_moveit_launch,

        # Node vision dinamis (mapping/detecting)
        dynamic_nodes,
    ])


#   ros2 launch fairino_gazebo fr5_gazebo_moveit.launch.py    gz_world:=world/table_with_block.sdf 

#   ros2 launch fairino_gazebo fr5_gazebo_moveit.launch.py vision_mode:=mapping
#   ros2 launch fairino_gazebo fr5_gazebo_moveit.launch.py vision_mode:=mapping  gz_world:=world/table_with_block.sdf 
#   ros2 launch fairino_gazebo fr5_gazebo_moveit.launch.py vision_mode:=mapping  gz_world:=world/table_with_block.sdf     merge_distance:=0.001

#   ros2 launch fairino_gazebo fr5_gazebo_moveit.launch.py vision_mode:=detecting    gz_world:=world/table_with_block.sdf 
#   ros2 launch fairino_gazebo f5r_gazebo_moveit.launch.py vision_mode:=detecting    gz_world:=world/table_with_block.sdf     reference_map:=reference_map.pcd
#   ros2 launch fairino_gazebo fr5_gazebo_moveit.launch.py vision_mode:=detecting    gz_world:=world/table_with_block.sdf     reference_map:=reference_map.pcd     merge_distance:=0.00007

# Ga boleh mode karena value parameter mode itu bakal turun ke launchfile gazebo dan moveit juga