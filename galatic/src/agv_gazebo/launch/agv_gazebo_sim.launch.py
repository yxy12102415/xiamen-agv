import os
import shutil
import tempfile

from ament_index_python.packages import PackageNotFoundError
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import EnvironmentVariable
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def gazebo_ros_packages():
    try:
        get_package_share_directory("ros_gz_sim")
        return {
            "sim_package": "ros_gz_sim",
            "sim_launch": "gz_sim.launch.py",
            "sim_args": "gz_args",
            "bridge_package": "ros_gz_bridge",
            "interfaces_package": "ros_gz_interfaces",
            "msgs_package": "gz.msgs",
            "supports_service_bridge": True,
        }
    except PackageNotFoundError:
        return {
            "sim_package": "ros_ign_gazebo",
            "sim_launch": "ign_gazebo.launch.py",
            "sim_args": "ign_args",
            "bridge_package": "ros_ign_bridge",
            "interfaces_package": "ros_ign_interfaces",
            "msgs_package": "ignition.msgs",
            "supports_service_bridge": False,
        }


def ignition_compat_paths(world_path, models_path):
    replacements = {
        '<sdf version="1.9">': '<sdf version="1.8">',
        'filename="gz-sim-physics-system" name="gz::sim::systems::Physics"':
            'filename="libignition-gazebo-physics-system.so" '
            'name="ignition::gazebo::systems::Physics"',
        'filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"':
            'filename="libignition-gazebo-user-commands-system.so" '
            'name="ignition::gazebo::systems::UserCommands"',
        'filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"':
            'filename="libignition-gazebo-scene-broadcaster-system.so" '
            'name="ignition::gazebo::systems::SceneBroadcaster"',
        'filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"':
            'filename="libignition-gazebo-sensors-system.so" '
            'name="ignition::gazebo::systems::Sensors"',
    }
    with open(world_path, "r", encoding="utf-8") as src:
        world = src.read()
    for old, new in replacements.items():
        world = world.replace(old, new)

    patched_path = os.path.join(tempfile.gettempdir(), "agv_xiamen_empty_ignition.sdf")
    with open(patched_path, "w", encoding="utf-8") as dst:
        dst.write(world)

    patched_models_path = os.path.join(tempfile.gettempdir(), "agv_galactic_models")
    if os.path.exists(patched_models_path):
        shutil.rmtree(patched_models_path)
    shutil.copytree(models_path, patched_models_path)
    for root, _, files in os.walk(patched_models_path):
        if "model.sdf" not in files:
            continue
        model_sdf_path = os.path.join(root, "model.sdf")
        with open(model_sdf_path, "r", encoding="utf-8") as src:
            model = src.read().replace('<sdf version="1.9">', '<sdf version="1.8">')
        with open(model_sdf_path, "w", encoding="utf-8") as dst:
            dst.write(model)

    return patched_path, os.pathsep.join([patched_models_path, models_path])


def generate_launch_description():
    map_path = LaunchConfiguration("map_path")
    projector_info_path = LaunchConfiguration("projector_info_path")
    pkg_share = get_package_share_directory("agv_gazebo")
    models_path = os.path.join(pkg_share, "models")
    world_file = (
        "xiamen_empty_galactic_lite.sdf"
        if os.environ.get("AGV_GALACTIC_LITE_WORLD", "0") == "1"
        else "xiamen_empty.sdf"
    )
    world_path = os.path.join(pkg_share, "worlds", world_file)
    initial_x = LaunchConfiguration("initial_x")
    initial_y = LaunchConfiguration("initial_y")
    initial_yaw = LaunchConfiguration("initial_yaw")
    steering_mode = LaunchConfiguration("steering_mode")
    imu_x = LaunchConfiguration("imu_x")
    imu_y = LaunchConfiguration("imu_y")
    imu_z = LaunchConfiguration("imu_z")
    imu_yaw = LaunchConfiguration("imu_yaw")
    imu_pitch = LaunchConfiguration("imu_pitch")
    imu_roll = LaunchConfiguration("imu_roll")
    imu_frame_id = LaunchConfiguration("imu_frame_id")
    gnss_x = LaunchConfiguration("gnss_x")
    gnss_y = LaunchConfiguration("gnss_y")
    gnss_z = LaunchConfiguration("gnss_z")
    gnss_yaw = LaunchConfiguration("gnss_yaw")
    gnss_pitch = LaunchConfiguration("gnss_pitch")
    gnss_roll = LaunchConfiguration("gnss_roll")
    gnss_frame_id = LaunchConfiguration("gnss_frame_id")
    secondary_gnss_x = LaunchConfiguration("secondary_gnss_x")
    secondary_gnss_y = LaunchConfiguration("secondary_gnss_y")
    secondary_gnss_z = LaunchConfiguration("secondary_gnss_z")
    secondary_gnss_yaw = LaunchConfiguration("secondary_gnss_yaw")
    secondary_gnss_pitch = LaunchConfiguration("secondary_gnss_pitch")
    secondary_gnss_roll = LaunchConfiguration("secondary_gnss_roll")
    secondary_gnss_frame_id = LaunchConfiguration("secondary_gnss_frame_id")
    imu_topic = LaunchConfiguration("imu_topic")
    gnss_topic = LaunchConfiguration("gnss_topic")
    secondary_gnss_topic = LaunchConfiguration("secondary_gnss_topic")
    reference_latitude = LaunchConfiguration("reference_latitude")
    reference_longitude = LaunchConfiguration("reference_longitude")
    reference_altitude = LaunchConfiguration("reference_altitude")
    rviz_config = LaunchConfiguration("rviz_config")
    use_rviz = LaunchConfiguration("use_rviz")
    use_gazebo_gui = LaunchConfiguration("use_gazebo_gui")
    use_standalone_map = LaunchConfiguration("use_standalone_map")
    use_autoware_map = LaunchConfiguration("use_autoware_map")
    enable_lidar_topics = LaunchConfiguration("enable_lidar_topics")
    enable_lidar_markers = LaunchConfiguration("enable_lidar_markers")
    enable_sensor_tf = LaunchConfiguration("enable_sensor_tf")
    enable_dynamic_obstacles = LaunchConfiguration("enable_dynamic_obstacles")
    enable_static_scene_markers = LaunchConfiguration("enable_static_scene_markers")
    enable_centerline_trajectory = LaunchConfiguration("enable_centerline_trajectory")
    enable_mpc = LaunchConfiguration("enable_mpc")
    enable_vehicle_interface = LaunchConfiguration("enable_vehicle_interface")
    vehicle_interface_shadow_mode = LaunchConfiguration("vehicle_interface_shadow_mode")
    vehicle_interface_autonomous_enable = LaunchConfiguration("vehicle_interface_autonomous_enable")
    vehicle_interface_mock_feedback = LaunchConfiguration("vehicle_interface_mock_feedback")
    target_speed = LaunchConfiguration("target_speed")
    route_lane_index = LaunchConfiguration("route_lane_index")
    normal_lane_change_guard_distance = LaunchConfiguration("normal_lane_change_guard_distance")
    normal_lane_change_guard_epsilon = LaunchConfiguration("normal_lane_change_guard_epsilon")
    enable_error_log = LaunchConfiguration("enable_error_log")
    error_log_path = LaunchConfiguration("error_log_path")
    rviz_library_path = os.pathsep.join(
        path for path in os.environ.get("LD_LIBRARY_PATH", "").split(os.pathsep)
        if path and not path.startswith("/snap/")
    )
    default_rviz_config = PathJoinSubstitution(
        [FindPackageShare("agv_gazebo"), "config", "stage4_lidar.rviz"]
    )
    default_map_path = PathJoinSubstitution(
        [FindPackageShare("agv_maps"), "map", "xiamen-prod.osm"]
    )
    default_projector_info_path = PathJoinSubstitution(
        [FindPackageShare("agv_maps"), "map", "map_projector_info.yaml"]
    )
    gazebo_packages = gazebo_ros_packages()
    if not gazebo_packages["supports_service_bridge"]:
        world_path, models_path = ignition_compat_paths(world_path, models_path)

    gz_launch = PathJoinSubstitution(
        [
            FindPackageShare(gazebo_packages["sim_package"]),
            "launch",
            gazebo_packages["sim_launch"],
        ]
    )
    set_pose_service_type = f"{gazebo_packages['interfaces_package']}/srv/SetEntityPose"
    gazebo_msgs = gazebo_packages["msgs_package"]

    system_bridges = [
        f"/clock@rosgraph_msgs/msg/Clock[{gazebo_msgs}.Clock",
    ]
    lidar_bridges = [
        f"/sensing/lidar/fl_lidar_1/points@sensor_msgs/msg/PointCloud2[{gazebo_msgs}.PointCloudPacked",
        f"/sensing/lidar/fr_lidar_1/points@sensor_msgs/msg/PointCloud2[{gazebo_msgs}.PointCloudPacked",
        f"/sensing/lidar/rl_lidar_1/points@sensor_msgs/msg/PointCloud2[{gazebo_msgs}.PointCloudPacked",
        f"/sensing/lidar/rr_lidar_1/points@sensor_msgs/msg/PointCloud2[{gazebo_msgs}.PointCloudPacked",
        f"/sensing/lidar/fl_lidar_2/points@sensor_msgs/msg/PointCloud2[{gazebo_msgs}.PointCloudPacked",
        f"/sensing/lidar/rr_lidar_2/points@sensor_msgs/msg/PointCloud2[{gazebo_msgs}.PointCloudPacked",
    ]
    camera_bridges = [
        f"/sensing/camera/front/image@sensor_msgs/msg/Image[{gazebo_msgs}.Image",
        f"/sensing/camera/front/camera_info@sensor_msgs/msg/CameraInfo[{gazebo_msgs}.CameraInfo",
        f"/sensing/camera/rear/image@sensor_msgs/msg/Image[{gazebo_msgs}.Image",
        f"/sensing/camera/rear/camera_info@sensor_msgs/msg/CameraInfo[{gazebo_msgs}.CameraInfo",
    ]
    if gazebo_packages["supports_service_bridge"]:
        system_bridges.append(
            f"/world/xiamen_empty/set_pose@{set_pose_service_type}"
            f"@{gazebo_msgs}.Pose@{gazebo_msgs}.Boolean"
        )
    lidar_calibration_defaults = [
        ("fl_lidar_1", "7.346", "1.560", "1.25", "0.785398", "0.0", "0.0"),
        ("fr_lidar_1", "7.346", "-1.560", "1.25", "-0.785398", "0.0", "0.0"),
        ("rl_lidar_1", "-7.346", "-1.560", "1.25", "-2.356194", "0.0", "0.0"),
        ("rr_lidar_1", "-7.346", "1.560", "1.25", "2.356194", "0.0", "0.0"),
        ("fl_lidar_2", "4.55", "1.52", "2.45", "0.0", "0.0", "0.0"),
        ("rr_lidar_2", "-4.55", "-1.52", "2.45", "3.141593", "0.0", "0.0"),
    ]
    lidar_static_transforms = [
        (
            LaunchConfiguration(f"{frame_id}_x"),
            LaunchConfiguration(f"{frame_id}_y"),
            LaunchConfiguration(f"{frame_id}_z"),
            LaunchConfiguration(f"{frame_id}_yaw"),
            LaunchConfiguration(f"{frame_id}_pitch"),
            LaunchConfiguration(f"{frame_id}_roll"),
            frame_id,
        )
        for frame_id, *_ in lidar_calibration_defaults
    ]
    lidar_frame_republishers = [
        ("/sim/lidar/fl_lidar_1_raw", "/sensing/lidar/fl_lidar_1/pointcloud", "fl_lidar_1"),
        ("/sim/lidar/fr_lidar_1_raw", "/sensing/lidar/fr_lidar_1/pointcloud", "fr_lidar_1"),
        ("/sim/lidar/rl_lidar_1_raw", "/sensing/lidar/rl_lidar_1/pointcloud", "rl_lidar_1"),
        ("/sim/lidar/rr_lidar_1_raw", "/sensing/lidar/rr_lidar_1/pointcloud", "rr_lidar_1"),
        ("/sim/lidar/fl_lidar_2_raw", "/sensing/lidar/fl_lidar_2/pointcloud", "fl_lidar_2"),
        ("/sim/lidar/rr_lidar_2_raw", "/sensing/lidar/rr_lidar_2/pointcloud", "rr_lidar_2"),
    ]
    ignition_pointcloud_bridges = []
    if not gazebo_packages["supports_service_bridge"]:
        ignition_pointcloud_bridges = [
            ("/sensing/lidar/fl_lidar_1/points", "/sim/lidar/fl_lidar_1_raw", "fl_lidar_1"),
            ("/sensing/lidar/fr_lidar_1/points", "/sim/lidar/fr_lidar_1_raw", "fr_lidar_1"),
            ("/sensing/lidar/rl_lidar_1/points", "/sim/lidar/rl_lidar_1_raw", "rl_lidar_1"),
            ("/sensing/lidar/rr_lidar_1/points", "/sim/lidar/rr_lidar_1_raw", "rr_lidar_1"),
            ("/sensing/lidar/fl_lidar_2/points", "/sim/lidar/fl_lidar_2_raw", "fl_lidar_2"),
            ("/sensing/lidar/rr_lidar_2/points", "/sim/lidar/rr_lidar_2_raw", "rr_lidar_2"),
        ]
    sensor_static_transforms = [
        (imu_x, imu_y, imu_z, imu_yaw, imu_pitch, imu_roll, imu_frame_id, "imu_tf"),
        (gnss_x, gnss_y, gnss_z, gnss_yaw, gnss_pitch, gnss_roll, gnss_frame_id, "gnss_tf"),
        (
            secondary_gnss_x,
            secondary_gnss_y,
            secondary_gnss_z,
            secondary_gnss_yaw,
            secondary_gnss_pitch,
            secondary_gnss_roll,
            secondary_gnss_frame_id,
            "secondary_gnss_tf",
        ),
        ("7.15", "0.0", "1.25", "0.0", "0.0", "0.0", "front_camera", "front_camera_tf"),
        (
            "-7.15",
            "0.0",
            "1.25",
            "3.141593",
            "0.0",
            "0.0",
            "rear_camera",
            "rear_camera_tf",
        ),
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument("initial_x", default_value="17.9991"),
            DeclareLaunchArgument("initial_y", default_value="28.0013"),
            DeclareLaunchArgument("initial_yaw", default_value="-1.5708"),
            DeclareLaunchArgument("steering_mode", default_value="axle12_steer"),
            DeclareLaunchArgument("imu_x", default_value="0.0"),
            DeclareLaunchArgument("imu_y", default_value="0.0"),
            DeclareLaunchArgument("imu_z", default_value="0.60"),
            DeclareLaunchArgument("imu_yaw", default_value="0.0"),
            DeclareLaunchArgument("imu_pitch", default_value="0.0"),
            DeclareLaunchArgument("imu_roll", default_value="0.0"),
            DeclareLaunchArgument("imu_frame_id", default_value="imu_link"),
            DeclareLaunchArgument("gnss_x", default_value="5.30"),
            DeclareLaunchArgument("gnss_y", default_value="0.0"),
            DeclareLaunchArgument("gnss_z", default_value="2.45"),
            DeclareLaunchArgument("gnss_yaw", default_value="0.0"),
            DeclareLaunchArgument("gnss_pitch", default_value="0.0"),
            DeclareLaunchArgument("gnss_roll", default_value="0.0"),
            DeclareLaunchArgument("gnss_frame_id", default_value="fr_m_gnss"),
            DeclareLaunchArgument("secondary_gnss_x", default_value="-5.30"),
            DeclareLaunchArgument("secondary_gnss_y", default_value="0.0"),
            DeclareLaunchArgument("secondary_gnss_z", default_value="2.45"),
            DeclareLaunchArgument("secondary_gnss_yaw", default_value="0.0"),
            DeclareLaunchArgument("secondary_gnss_pitch", default_value="0.0"),
            DeclareLaunchArgument("secondary_gnss_roll", default_value="0.0"),
            DeclareLaunchArgument("secondary_gnss_frame_id", default_value="rl_s_gnss"),
            *[
                DeclareLaunchArgument(f"{frame_id}_{axis}", default_value=default_value)
                for frame_id, x, y, z, yaw, pitch, roll in lidar_calibration_defaults
                for axis, default_value in (
                    ("x", x),
                    ("y", y),
                    ("z", z),
                    ("yaw", yaw),
                    ("pitch", pitch),
                    ("roll", roll),
                )
            ],
            DeclareLaunchArgument("imu_topic", default_value="/chnav/imu/data"),
            DeclareLaunchArgument("gnss_topic", default_value="/sensing/gnss/fr_m/fix"),
            DeclareLaunchArgument("secondary_gnss_topic", default_value="/sensing/gnss/rl_s/fix"),
            DeclareLaunchArgument("map_path", default_value=default_map_path),
            DeclareLaunchArgument(
                "projector_info_path", default_value=default_projector_info_path
            ),
            DeclareLaunchArgument("reference_latitude", default_value="0.0050"),
            DeclareLaunchArgument("reference_longitude", default_value="0.0075"),
            DeclareLaunchArgument("reference_altitude", default_value="0.0"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument(
                "use_gazebo_gui",
                default_value="true" if os.environ.get("DISPLAY") else "false",
            ),
            DeclareLaunchArgument("use_standalone_map", default_value="true"),
            DeclareLaunchArgument("use_autoware_map", default_value="false"),
            DeclareLaunchArgument("enable_lidar_topics", default_value="true"),
            DeclareLaunchArgument("enable_lidar_markers", default_value="false"),
            DeclareLaunchArgument("enable_sensor_tf", default_value="true"),
            DeclareLaunchArgument("enable_dynamic_obstacles", default_value="true"),
            DeclareLaunchArgument("enable_static_scene_markers", default_value="false"),
            DeclareLaunchArgument("enable_centerline_trajectory", default_value="true"),
            DeclareLaunchArgument("enable_mpc", default_value="true"),
            DeclareLaunchArgument("enable_vehicle_interface", default_value="true"),
            DeclareLaunchArgument("vehicle_interface_shadow_mode", default_value="true"),
            DeclareLaunchArgument("vehicle_interface_autonomous_enable", default_value="false"),
            DeclareLaunchArgument("vehicle_interface_mock_feedback", default_value="false"),
            DeclareLaunchArgument("target_speed", default_value="4.0"),
            DeclareLaunchArgument("route_lane_index", default_value="2"),
            DeclareLaunchArgument("normal_lane_change_guard_distance", default_value="12.0"),
            DeclareLaunchArgument("normal_lane_change_guard_epsilon", default_value="0.5"),
            DeclareLaunchArgument("enable_error_log", default_value="true"),
            DeclareLaunchArgument("error_log_path", default_value="/tmp/agv_mpc_error.csv"),
            DeclareLaunchArgument("rviz_config", default_value=default_rviz_config),
            SetEnvironmentVariable(
                name="GZ_SIM_RESOURCE_PATH",
                value=[
                    models_path,
                    os.pathsep,
                    EnvironmentVariable("GZ_SIM_RESOURCE_PATH", default_value=""),
                ],
            ),
            SetEnvironmentVariable(
                name="IGN_GAZEBO_RESOURCE_PATH",
                value=[
                    models_path,
                    os.pathsep,
                    EnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", default_value=""),
                ],
            ),
            SetEnvironmentVariable(name="IGN_PARTITION", value="agv_sim"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gz_launch),
                launch_arguments={gazebo_packages["sim_args"]: ["-r -v 2 ", world_path]}.items(),
                condition=IfCondition(use_gazebo_gui),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gz_launch),
                launch_arguments={
                    gazebo_packages["sim_args"]: ["-r -s -v 2 ", world_path]
                }.items(),
                condition=UnlessCondition(use_gazebo_gui),
            ),
            Node(
                package=gazebo_packages["bridge_package"],
                executable="parameter_bridge",
                name="agv_gazebo_bridge",
                output="screen",
                arguments=system_bridges,
            ),
            *([
                Node(
                    package=gazebo_packages["bridge_package"],
                    executable="parameter_bridge",
                    name="agv_lidar_bridge",
                    output="screen",
                    arguments=lidar_bridges,
                    remappings=[
                        ("/sensing/lidar/fl_lidar_1/points", "/sim/lidar/fl_lidar_1_raw"),
                        ("/sensing/lidar/fr_lidar_1/points", "/sim/lidar/fr_lidar_1_raw"),
                        ("/sensing/lidar/rl_lidar_1/points", "/sim/lidar/rl_lidar_1_raw"),
                        ("/sensing/lidar/rr_lidar_1/points", "/sim/lidar/rr_lidar_1_raw"),
                        ("/sensing/lidar/fl_lidar_2/points", "/sim/lidar/fl_lidar_2_raw"),
                        ("/sensing/lidar/rr_lidar_2/points", "/sim/lidar/rr_lidar_2_raw"),
                    ],
                    condition=IfCondition(enable_lidar_topics),
                )
            ] if gazebo_packages["supports_service_bridge"] else []),
            Node(
                package=gazebo_packages["bridge_package"],
                executable="parameter_bridge",
                name="agv_camera_bridge",
                output="screen",
                arguments=camera_bridges,
                condition=IfCondition(enable_lidar_topics),
            ),
            *[
                Node(
                    package="agv_gazebo",
                    executable="ign_pointcloud_bridge_node",
                    name=f"{frame_id}_ign_pointcloud_bridge",
                    output="screen",
                    parameters=[
                        {
                            "ign_topic": ign_topic,
                            "ros_topic": ros_topic,
                            "fallback_frame_id": frame_id,
                        }
                    ],
                    condition=IfCondition(enable_lidar_topics),
                )
                for ign_topic, ros_topic, frame_id in ignition_pointcloud_bridges
            ],
            *[
                Node(
                    package="agv_gazebo",
                    executable="pointcloud_frame_republisher_node",
                    name=f"{frame_id}_frame_republisher",
                    output="screen",
                    parameters=[
                        {
                            "input_topic": input_topic,
                            "output_topic": output_topic,
                            "frame_id": frame_id,
                            "z_offset": 0.0,
                        }
                    ],
                    condition=IfCondition(enable_lidar_topics),
                )
                for input_topic, output_topic, frame_id in lidar_frame_republishers
            ],
            Node(
                package="autoware_map_projection_loader",
                executable="autoware_map_projection_loader_node",
                name="map_projection_loader",
                output="screen",
                parameters=[
                    {
                        "map_projector_info_path": projector_info_path,
                        "lanelet2_map_path": map_path,
                    }
                ],
                condition=IfCondition(use_autoware_map),
            ),
            Node(
                package="autoware_map_loader",
                executable="autoware_lanelet2_map_loader",
                name="lanelet2_map_loader",
                output="screen",
                parameters=[
                    {
                        "allow_unsupported_version": True,
                        "center_line_resolution": 5.0,
                        "use_waypoints": True,
                        "lanelet2_map_path": map_path,
                    }
                ],
                condition=IfCondition(use_autoware_map),
            ),
            Node(
                package="agv_map_visualizer",
                executable="lanelet2_marker_publisher",
                name="lanelet2_marker_publisher",
                output="screen",
                condition=IfCondition(use_autoware_map),
            ),
            Node(
                package="agv_map_visualizer",
                executable="osm_marker_publisher.py",
                name="osm_marker_publisher",
                output="screen",
                parameters=[
                    {
                        "map_path": map_path,
                        "output_topic": "/map/vector_map_marker",
                        "frame_id": "map",
                    }
                ],
                condition=IfCondition(use_standalone_map),
            ),
            Node(
                package="agv_vehicle_model",
                executable="agv_two_axis_simulator_node",
                name="agv_two_axis_simulator",
                output="screen",
                parameters=[
                    {
                        "frame_id": "map",
                        "child_frame_id": "base_link",
                        "steering_mode": steering_mode,
                        "asymmetric_rear_steer_ratio": -0.5,
                        "dt": 0.025,
                        "vx_lim": 5.0,
                        "steer_lim": 0.47822,
                        "steer_lim_front": 0.47822,
                        "steer_lim_rear": 0.39444,
                        # From vehicle parameter sheet.
                        "vehicle_mass_kg": 23000.0,
                        # Estimated, not measured on the real vehicle.
                        "yaw_inertia_kgm2": 432363.7513,
                        # Estimated tire stiffness; the parameter sheet marks this as no data.
                        "front_cornering_stiffness_n_rad": 400000.0,
                        "rear_cornering_stiffness_n_rad": 400000.0,
                        "axle_count": 4,
                        "wheel_count": 8,
                        # Derived from uniform load distribution: 23000 kg / 4 axles / 8 wheels.
                        "axle_load_kg": 5750.0,
                        "wheel_load_kg": 2875.0,
                        "front_normal_load_n": 112815.0,
                        "rear_normal_load_n": 112815.0,
                        # Estimated from current Gazebo vehicle geometry; not yet matched to
                        # the sheet values 1/4 axle=4.65 m and 2/3 axle=3.0 m.
                        "axle1_x": 4.479714,
                        "axle2_x": 2.596727,
                        "axle3_x": -2.570058,
                        "axle4_x": -4.479289,
                        # Per-axle split of the estimated front/rear tire stiffness above.
                        "axle1_cornering_stiffness_n_rad": 200000.0,
                        "axle2_cornering_stiffness_n_rad": 200000.0,
                        "axle3_cornering_stiffness_n_rad": 200000.0,
                        "axle4_cornering_stiffness_n_rad": 200000.0,
                        "axle1_normal_load_n": 56407.5,
                        "axle2_normal_load_n": 56407.5,
                        "axle3_normal_load_n": 56407.5,
                        "axle4_normal_load_n": 56407.5,
                        "tire_friction_coefficient": 0.8,
                        "vx_rate_lim": 0.35,
                        "max_drive_accel": 0.35,
                        "max_brake_decel": 4.0,
                        "steer_rate_lim": 0.261799,
                        "wheelbase_f": 4.479714,
                        "wheelbase_r": 4.479289,
                        "wheel_tread": 2.381384,
                        "wheel_radius": 0.66,
                        "enable_ackermann_wheel_angles": True,
                        # Back-calculated Ackermann equivalent wheelbases so the visual
                        # inner/outer wheel angles match the sheet:
                        # front 27.4 deg outer -> 33.0 deg inner,
                        # rear 22.6 deg outer -> 30.0 deg inner.
                        "ackermann_front_wheelbase": 6.116611371515338,
                        "ackermann_rear_wheelbase": 3.5527407651152627,
                        # Estimated longitudinal dynamics and aero/rolling parameters.
                        "center_of_gravity_height_m": 0.8,
                        "rolling_resistance_coefficient": 0.012,
                        "aerodynamic_drag_coefficient": 1.10,
                        "frontal_area_m2": 7.936695,
                        "air_density_kg_m3": 1.225,
                        # Estimated wheel rotational inertia; the parameter sheet has no data.
                        "wheel_inertia_kgm2": 67.0824,
                        "wheel_tf_front_x": 4.479714,
                        "wheel_tf_mid_front_x": 2.596727,
                        "wheel_tf_mid_rear_x": -2.570058,
                        "wheel_tf_rear_x": -4.479289,
                        "wheel_tf_left_y": 1.190692,
                        "wheel_tf_right_y": -1.190692,
                        "wheel_tf_z": 0.66,
                        "vx_delay": 0.05,
                        "vx_time_constant": 0.2,
                        "drive_delay": 0.05,
                        "brake_delay": 0.05,
                        "drive_time_constant": 0.18,
                        "brake_time_constant": 0.18,
                        "all_wheel_drive": True,
                        "steer_delay": 0.02,
                        "steer_time_constant": 0.05,
                        "steer_dead_band": 0.005236,
                        "steer_bias": 0.0,
                        "steering_mode_transition_time": 0.45,
                        "initial_x": initial_x,
                        "initial_y": initial_y,
                        "initial_yaw": initial_yaw,
                        "map_path": map_path,
                        "use_ground_height": True,
                        "ground_height_offset": 0.0,
                        "publish_tf": True,
                    }
                ],
            ),
            Node(
                package="agv_mpc_controller",
                executable="centerline_trajectory_publisher",
                name="centerline_trajectory_publisher",
                output="screen",
                parameters=[
                    {
                        "map_path": map_path,
                        "output_topic": "/planning/scenario_planning/trajectory",
                        "marker_topic": "/visualization/centerline_trajectory_marker",
                        "frame_id": "map",
                        "target_speed": target_speed,
                        "min_speed": 0.5,
                        "max_lateral_accel": 0.35,
                        "curvature_window": 4,
                        "speed_smoothing_passes": 3,
                        "short_straight_speed_limit": 1.0,
                        "turn_speed_limit": 1.0,
                        "turn_curvature_threshold": 0.015,
                        "turn_entry_hold_distance": 8.0,
                        "turn_exit_hold_distance": 12.0,
                        "max_profile_accel": 0.35,
                        "max_profile_decel": 0.35,
                        "path_smoothing_passes": 3,
                        "point_spacing": 0.5,
                        "route_mode": "port_loop",
                        "route_lane_index": route_lane_index,
                        "route_turn_radius": 15.0,
                        "lane_change_length": 16.0,
                        "inner_to_outer_inner_s": 84.0,
                        "inner_to_outer_outer_s": 84.0,
                        "outer_to_inner_outer_s": 218.0,
                        "outer_to_inner_inner_s": 212.0,
                        "initial_x": initial_x,
                        "initial_y": initial_y,
                        "initial_yaw": initial_yaw,
                    }
                ],
                condition=IfCondition(enable_centerline_trajectory),
            ),
            Node(
                package="agv_mpc_controller",
                executable="sampling_mpc_controller",
                name="sampling_mpc_controller",
                output="screen",
                parameters=[
                    {
                        "trajectory_topic": "/planning/scenario_planning/trajectory",
                        "odom_topic": "/localization/kinematic_state",
                        "steering_topic": "/vehicle/status/steering_status",
                        "front_steer_topic": "/agv/status/front_steer",
                        "rear_steer_topic": "/agv/status/rear_steer",
                        "control_topic": "/control/command/control_cmd_raw",
                        "two_axis_steer_topic": "/agv/control/two_axis_steer_cmd_raw",
                        "gear_topic": "/control/command/gear_cmd_raw",
                        "control_rate": 30.0,
                        "prediction_dt": 0.2,
                        "horizon_steps": 12,
                        "target_speed": target_speed,
                        "vx_lim": 5.0,
                        "vx_rate_lim": 0.35,
                        "steer_lim": 0.47822,
                        "steer_lim_front": 0.47822,
                        "steer_lim_rear": 0.39444,
                        "steer_rate_lim": 0.261799,
                        "command_steer_rate_lim": 0.261799,
                        "prediction_steer_rate_lim": 0.261799,
                        "prediction_vx_time_constant": 0.2,
                        "prediction_steer_time_constant": 0.05,
                        "prediction_drive_delay": 0.05,
                        "prediction_brake_delay": 0.05,
                        "prediction_drive_time_constant": 0.18,
                        "prediction_brake_time_constant": 0.18,
                        "prediction_steer_delay": 0.02,
                        "wheelbase_f": 4.479714,
                        "wheelbase_r": 4.479289,
                        "axle2_x": 2.596727,
                        "axle3_x": -2.570058,
                        "axle4_x": -4.479289,
                        "steering_mode": steering_mode,
                        "two_axis_beta_limit": 0.22,
                        "two_axis_lateral_gain": 0.12,
                        "curvature_preview_distance": 5.0,
                        "feedforward_steer_limit": 0.42,
                        "enable_loop_stop": True,
                        "loop_stop_duration": 3.0,
                        "loop_stop_speed_threshold": 0.08,
                        "enable_error_log": enable_error_log,
                        "error_log_path": error_log_path,
                        "closed_trajectory": True,
                    }
                ],
                condition=IfCondition(enable_mpc),
            ),
            Node(
                package="agv_safety_supervisor",
                executable="safety_supervisor_node",
                name="agv_safety_supervisor",
                output="screen",
                parameters=[
                    {
                        "raw_control_topic": "/control/command/control_cmd_raw",
                        "raw_two_axis_steer_topic": "/agv/control/two_axis_steer_cmd_raw",
                        "raw_gear_topic": "/control/command/gear_cmd_raw",
                        "safe_control_topic": "/control/command/control_cmd",
                        "safe_two_axis_steer_topic": "/agv/control/two_axis_steer_cmd",
                        "safe_gear_topic": "/control/command/gear_cmd",
                        "odom_topic": "/localization/kinematic_state",
                        "velocity_status_topic": "/vehicle/status/velocity_status",
                        "control_mode_topic": "/vehicle/status/control_mode",
                        "front_steer_topic": "/agv/status/front_steer",
                        "rear_steer_topic": "/agv/status/rear_steer",
                        "command_timeout": 0.2,
                        "odom_timeout": 0.3,
                        "feedback_timeout": 0.5,
                        "require_autonomous_mode": True,
                        "max_velocity": 5.0,
                        "max_drive_accel": 0.35,
                        "max_brake_decel": 4.0,
                        "front_steer_limit": 0.47822,
                        "rear_steer_limit": 0.39444,
                        "stop_acceleration": -1.0,
                        "stop_gear": "drive",
                    }
                ],
                condition=IfCondition(enable_mpc),
            ),
            Node(
                package="agv_vehicle_interface",
                executable="reference_vehicle_interface_node",
                name="reference_vehicle_interface",
                output="screen",
                parameters=[
                    {
                        "control_topic": "/control/command/control_cmd",
                        "two_axis_steer_topic": "/agv/control/two_axis_steer_cmd",
                        "gear_topic": "/control/command/gear_cmd",
                        "tx_frame_topic": "/agv/reference_can/tx",
                        "state_topic": "/agv/vehicle_interface/state",
                        "shadow_mode": vehicle_interface_shadow_mode,
                        "autonomous_enable": vehicle_interface_autonomous_enable,
                        "mock_feedback": vehicle_interface_mock_feedback,
                        "steering_mode": "axle12",
                        "command_timeout": 0.2,
                        "max_velocity": 5.0,
                        "max_acceleration": 0.35,
                        "max_brake_decel": 4.0,
                        "max_steer": 0.47822,
                    }
                ],
                condition=IfCondition(enable_vehicle_interface),
            ),
            Node(
                package="agv_gazebo",
                executable="kinematic_state_to_gazebo_node",
                name="kinematic_state_to_gazebo",
                output="screen",
                parameters=[
                    {
                        "odom_topic": "/localization/kinematic_state",
                        "service_name": "/world/xiamen_empty/set_pose",
                        "entity_name": "ego_agv",
                        "z_offset": 0.0245,
                        "use_odom_z": False,
                        "initial_x": initial_x,
                        "initial_y": initial_y,
                        "initial_yaw": initial_yaw,
                        "request_timeout_ms": 100,
                    }
                ],
            ),
            Node(
                package="agv_gazebo",
                executable="gnss_imu_simulator_node",
                name="gnss_imu_simulator",
                output="screen",
                parameters=[
                    {
                        "odom_topic": "/localization/kinematic_state",
                        "imu_topic": imu_topic,
                        "gnss_topic": gnss_topic,
                        "imu_frame_id": imu_frame_id,
                        "gnss_frame_id": gnss_frame_id,
                        "gnss_x": gnss_x,
                        "gnss_y": gnss_y,
                        "gnss_z": gnss_z,
                        "secondary_gnss_topic": secondary_gnss_topic,
                        "secondary_gnss_frame_id": secondary_gnss_frame_id,
                        "secondary_gnss_x": secondary_gnss_x,
                        "secondary_gnss_y": secondary_gnss_y,
                        "secondary_gnss_z": secondary_gnss_z,
                        "reference_latitude": reference_latitude,
                        "reference_longitude": reference_longitude,
                        "reference_altitude": reference_altitude,
                    }
                ],
            ),
            Node(
                package="agv_gazebo",
                executable="dynamic_obstacle_simulator_node",
                name="dynamic_obstacle_simulator",
                output="screen",
                parameters=[
                    {
                        "service_name": "/world/xiamen_empty/set_pose",
                        "update_rate": 10.0,
                        "traffic_vehicle_speed": 1.5,
                        "pedestrian_speed": 0.8,
                        "yard_crane_speed": 1.2,
                        "request_timeout_ms": 100,
                        "start_delay_s": 6.0,
                    }
                ],
                condition=IfCondition(enable_dynamic_obstacles),
            ),
            Node(
                package="agv_map_visualizer",
                executable="agv_vehicle_marker_publisher",
                name="agv_vehicle_marker_publisher",
                output="screen",
                parameters=[
                    {
                        "input_topic": "/localization/kinematic_state",
                        "output_topic": "/visualization/agv_vehicle_marker",
                        "visual_z_offset": 0.0,
                    }
                ],
            ),
            Node(
                package="agv_map_visualizer",
                executable="static_obstacle_marker_publisher",
                name="static_scene_marker_publisher",
                output="screen",
                parameters=[
                    {
                        "output_topic": "/visualization/static_scene_marker",
                        "frame_id": "map",
                    }
                ],
                condition=IfCondition(enable_static_scene_markers),
            ),
            Node(
                package="agv_map_visualizer",
                executable="lidar_point_marker_publisher",
                name="lidar_point_marker_publisher",
                output="screen",
                parameters=[
                    {
                        "output_topic": "/visualization/lidar_point_markers",
                        "point_size": 0.07,
                        "sample_step": 1,
                        "max_points_per_cloud": 40000,
                        "z_offset": 0.0,
                        "use_cloud_stamp": False,
                        "marker_lifetime_sec": 1.0,
                    }
                ],
                condition=IfCondition(enable_lidar_markers),
            ),
            *[
                Node(
                    package="tf2_ros",
                    executable="static_transform_publisher",
                    name=f"{child_frame.split('/')[-1]}_tf",
                    output="screen",
                    arguments=[
                        x,
                        y,
                        z,
                        yaw,
                        pitch,
                        roll,
                        "base_link",
                        child_frame,
                    ],
                    condition=IfCondition(enable_lidar_topics),
                )
                for x, y, z, yaw, pitch, roll, child_frame in lidar_static_transforms
            ],
            *[
                Node(
                    package="tf2_ros",
                    executable="static_transform_publisher",
                    name=node_name,
                    output="screen",
                    arguments=[
                        x,
                        y,
                        z,
                        yaw,
                        pitch,
                        roll,
                        "base_link",
                        child_frame,
                    ],
                    condition=IfCondition(enable_sensor_tf),
                )
                for x, y, z, yaw, pitch, roll, child_frame, node_name in sensor_static_transforms
            ],
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2_gazebo",
                output="screen",
                additional_env={"LD_LIBRARY_PATH": rviz_library_path},
                arguments=[
                    "-d",
                    rviz_config,
                ],
                condition=IfCondition(use_rviz),
            ),
        ]
    )
