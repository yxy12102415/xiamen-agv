import os
import shutil
import tempfile
import xml.etree.ElementTree as ET

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
from launch.substitutions import PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


SENSOR_PROFILES = {
    "full": {
        "camera_front": (960, 540),
        "camera_other": (960, 540),
        "transport_front": (960, 540),
        "transport_other": (640, 360),
        "lidar_horizontal_cap": None,
    },
    "remote": {
        "camera_front": (960, 540),
        "camera_other": (640, 360),
        "transport_front": (960, 540),
        "transport_other": (640, 360),
        "lidar_horizontal_cap": 900,
    },
    "fast": {
        "camera_front": (800, 450),
        "camera_other": (480, 270),
        "transport_front": (800, 450),
        "transport_other": (480, 270),
        "lidar_horizontal_cap": 600,
    },
}


def normalized_sensor_profile():
    requested = os.environ.get("AGV_SENSOR_PROFILE", "full").strip().lower()
    aliases = {"balanced": "remote", "performance": "fast"}
    profile = aliases.get(requested, requested)
    if profile not in SENSOR_PROFILES:
        supported = ", ".join(SENSOR_PROFILES)
        raise RuntimeError(
            f"Unsupported AGV_SENSOR_PROFILE '{requested}'; choose {supported}"
        )
    return profile


def apply_sensor_profile(model_sdf_path, profile_name):
    profile = SENSOR_PROFILES[profile_name]
    if profile_name == "full":
        return

    tree = ET.parse(model_sdf_path)
    root = tree.getroot()
    for sensor in root.findall(".//sensor"):
        sensor_type = sensor.get("type", "")
        sensor_name = sensor.get("name", "")
        if sensor_type == "camera":
            size = (
                profile["camera_front"]
                if sensor_name == "front_camera"
                else profile["camera_other"]
            )
            width = sensor.find("./camera/image/width")
            height = sensor.find("./camera/image/height")
            if width is not None and height is not None:
                width.text = str(size[0])
                height.text = str(size[1])
        elif sensor_type == "gpu_lidar":
            samples = sensor.find("./lidar/scan/horizontal/samples")
            cap = profile["lidar_horizontal_cap"]
            if samples is not None and cap is not None:
                samples.text = str(min(int(samples.text), cap))

    tree.write(model_sdf_path, encoding="unicode", xml_declaration=True)


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


def runtime_models_path(models_path, sensor_profile, sdf_version=None):
    if sensor_profile == "full" and sdf_version is None:
        return models_path

    version_tag = sdf_version.replace(".", "_") if sdf_version else "native"
    patched_models_path = os.path.join(
        tempfile.gettempdir(),
        f"agv_runtime_models_{sensor_profile}_{version_tag}",
    )
    patched_model_path = os.path.join(patched_models_path, "agv_with_lidars")
    source_model_path = os.path.join(models_path, "agv_with_lidars")
    if os.path.exists(patched_model_path):
        shutil.rmtree(patched_model_path)
    shutil.copytree(source_model_path, patched_model_path)

    model_sdf_path = os.path.join(patched_model_path, "model.sdf")
    if sdf_version is not None:
        with open(model_sdf_path, "r", encoding="utf-8") as src:
            model = src.read().replace(
                '<sdf version="1.9">', f'<sdf version="{sdf_version}">'
            )
        with open(model_sdf_path, "w", encoding="utf-8") as dst:
            dst.write(model)
    apply_sensor_profile(model_sdf_path, sensor_profile)
    return os.pathsep.join([patched_models_path, models_path])


def ignition_compat_paths(world_path, models_path, sensor_profile):
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

    return (
        patched_path,
        runtime_models_path(models_path, sensor_profile, sdf_version="1.8"),
    )


def generate_launch_description():
    map_path = LaunchConfiguration("map_path")
    projector_info_path = LaunchConfiguration("projector_info_path")
    pkg_share = get_package_share_directory("agv_gazebo")
    models_path = os.path.join(pkg_share, "models")
    world_path = os.path.join(pkg_share, "worlds", "xiamen_empty.sdf")
    sensor_profile = normalized_sensor_profile()
    sensor_profile_config = SENSOR_PROFILES[sensor_profile]
    initial_x = LaunchConfiguration("initial_x")
    initial_y = LaunchConfiguration("initial_y")
    initial_yaw = LaunchConfiguration("initial_yaw")
    steering_mode = LaunchConfiguration("steering_mode")
    vehicle_load_profile = LaunchConfiguration("vehicle_load_profile")
    surface_friction_coefficient = LaunchConfiguration("surface_friction_coefficient")
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
    enable_camera_topics = LaunchConfiguration("enable_camera_topics")
    enable_lidar_markers = LaunchConfiguration("enable_lidar_markers")
    enable_sensor_tf = LaunchConfiguration("enable_sensor_tf")
    enable_dynamic_obstacles = LaunchConfiguration("enable_dynamic_obstacles")
    enable_static_scene_markers = LaunchConfiguration("enable_static_scene_markers")
    enable_centerline_trajectory = LaunchConfiguration("enable_centerline_trajectory")
    enable_mpc = LaunchConfiguration("enable_mpc")
    enable_remote_drive = LaunchConfiguration("enable_remote_drive")
    enable_remote_protocol_mock = LaunchConfiguration("enable_remote_protocol_mock")
    remote_protocol_local_host = LaunchConfiguration("remote_protocol_local_host")
    remote_protocol_local_port = LaunchConfiguration("remote_protocol_local_port")
    remote_protocol_vehicle_host = LaunchConfiguration("remote_protocol_vehicle_host")
    remote_protocol_vehicle_port = LaunchConfiguration("remote_protocol_vehicle_port")
    remote_protocol_endian = LaunchConfiguration("remote_protocol_endian")
    remote_protocol_tlv_length_includes_header = LaunchConfiguration(
        "remote_protocol_tlv_length_includes_header"
    )
    remote_network_delay_ms = LaunchConfiguration("remote_network_delay_ms")
    remote_network_jitter_ms = LaunchConfiguration("remote_network_jitter_ms")
    remote_network_loss_percent = LaunchConfiguration("remote_network_loss_percent")
    remote_network_seed = LaunchConfiguration("remote_network_seed")
    remote_video_delay_ms = LaunchConfiguration("remote_video_delay_ms")
    remote_video_jitter_ms = LaunchConfiguration("remote_video_jitter_ms")
    remote_video_loss_percent = LaunchConfiguration("remote_video_loss_percent")
    remote_video_transport = LaunchConfiguration("remote_video_transport")
    remote_video_encoder = LaunchConfiguration("remote_video_encoder")
    remote_video_bitrate_kbps = LaunchConfiguration("remote_video_bitrate_kbps")
    remote_video_direct_gz = LaunchConfiguration("remote_video_direct_gz")
    remote_video_link_capacity_mbps = LaunchConfiguration(
        "remote_video_link_capacity_mbps"
    )
    remote_network_delay_value = ParameterValue(remote_network_delay_ms, value_type=float)
    remote_network_jitter_value = ParameterValue(remote_network_jitter_ms, value_type=float)
    remote_network_loss_value = ParameterValue(
        remote_network_loss_percent, value_type=float
    )
    remote_network_seed_value = ParameterValue(remote_network_seed, value_type=int)
    remote_video_delay_value = ParameterValue(remote_video_delay_ms, value_type=float)
    remote_video_jitter_value = ParameterValue(remote_video_jitter_ms, value_type=float)
    remote_video_loss_value = ParameterValue(
        remote_video_loss_percent, value_type=float
    )
    remote_video_bitrate_value = ParameterValue(
        remote_video_bitrate_kbps, value_type=int
    )
    remote_video_per_camera_capacity_value = ParameterValue(
        PythonExpression([remote_video_link_capacity_mbps, " / 6.0"]),
        value_type=float,
    )
    remote_camera_names = (
        "front",
        "rear",
        "left_front",
        "left_rear",
        "right_front",
        "right_rear",
    )
    # The front view stays full resolution. Thumbnail views use a smaller
    # Keep the transport and receiver dimensions matched to the active sensor
    # profile so FFmpeg never needs an extra scale pass.
    remote_camera_output_sizes = {
        camera_name: (
            sensor_profile_config["transport_front"]
            if camera_name == "front"
            else sensor_profile_config["transport_other"]
        )
        for camera_name in remote_camera_names
    }
    remote_camera_receiver_sizes = {
        camera_name: (
            ParameterValue(
                PythonExpression([
                    str(width),
                    " if '",
                    remote_video_direct_gz,
                    "' == 'true' else 960",
                ]),
                value_type=int,
            ),
            ParameterValue(
                PythonExpression([
                    str(height),
                    " if '",
                    remote_video_direct_gz,
                    "' == 'true' else 540",
                ]),
                value_type=int,
            ),
        )
        for camera_name, (width, height) in remote_camera_output_sizes.items()
    }
    control_authority = LaunchConfiguration("control_authority")
    enable_control_stack = PythonExpression(
        [
            "'",
            enable_mpc,
            "' == 'true' or '",
            enable_remote_drive,
            "' == 'true'",
        ]
    )
    enable_remote_mock = PythonExpression(
        [
            "'",
            enable_remote_drive,
            "' == 'true' and '",
            enable_remote_protocol_mock,
            "' == 'true'",
        ]
    )
    enable_remote_h264_video = PythonExpression(
        [
            "'",
            enable_remote_drive,
            "' == 'true' and '",
            remote_video_transport,
            "' == 'h264'",
        ]
    )
    enable_remote_direct_h264 = PythonExpression(
        [
            "'",
            enable_remote_drive,
            "' == 'true' and '",
            remote_video_transport,
            "' == 'h264' and '",
            remote_video_direct_gz,
            "' == 'true'",
        ]
    )
    enable_remote_python_h264 = PythonExpression(
        [
            "'",
            enable_remote_drive,
            "' == 'true' and '",
            remote_video_transport,
            "' == 'h264' and not ('",
            remote_video_direct_gz,
            "' == 'true')",
        ]
    )
    enable_camera_topic_bridges = PythonExpression(
        [
            "'",
            enable_camera_topics,
            "' == 'true' and not ('",
            enable_remote_drive,
            "' == 'true' and '",
            remote_video_transport,
            "' == 'h264' and '",
            remote_video_direct_gz,
            "' == 'true')",
        ]
    )
    enable_remote_raw_video = PythonExpression(
        [
            "'",
            enable_remote_drive,
            "' == 'true' and '",
            remote_video_transport,
            "' == 'raw'",
        ]
    )
    enable_vehicle_interface = LaunchConfiguration("enable_vehicle_interface")
    enable_simulated_chassis_feedback = LaunchConfiguration("enable_simulated_chassis_feedback")
    vehicle_interface_shadow_mode = LaunchConfiguration("vehicle_interface_shadow_mode")
    vehicle_interface_autonomous_enable = LaunchConfiguration("vehicle_interface_autonomous_enable")
    vehicle_interface_mock_feedback = LaunchConfiguration("vehicle_interface_mock_feedback")
    target_speed = LaunchConfiguration("target_speed")
    route_lane_index = LaunchConfiguration("route_lane_index")
    normal_lane_change_guard_distance = LaunchConfiguration("normal_lane_change_guard_distance")
    normal_lane_change_guard_epsilon = LaunchConfiguration("normal_lane_change_guard_epsilon")
    enable_error_log = LaunchConfiguration("enable_error_log")
    error_log_path = LaunchConfiguration("error_log_path")
    vehicle_profile_file = PathJoinSubstitution(
        [
            FindPackageShare("agv_vehicle_model"),
            "config",
            PythonExpression(["'", vehicle_load_profile, "' + '.param.yaml'"]),
        ]
    )
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
        world_path, models_path = ignition_compat_paths(
            world_path, models_path, sensor_profile
        )
    else:
        models_path = runtime_models_path(models_path, sensor_profile)
    print(
        "[agv_gazebo] sensor profile=%s, camera=%sx%s/%sx%s, lidar cap=%s"
        % (
            sensor_profile,
            *sensor_profile_config["camera_front"],
            *sensor_profile_config["camera_other"],
            sensor_profile_config["lidar_horizontal_cap"] or "full",
        )
    )

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
    camera_frames = [
        ("front", "7.65", "0.0", "1.95", "0.0", "0.174533", "0.0"),
        ("rear", "-7.65", "0.0", "1.95", "3.141593", "0.174533", "0.0"),
        ("left_front", "3.75", "1.65", "1.95", "1.570796", "0.174533", "0.0"),
        ("left_rear", "-3.75", "1.65", "1.95", "1.570796", "0.174533", "0.0"),
        ("right_front", "3.75", "-1.65", "1.95", "-1.570796", "0.174533", "0.0"),
        ("right_rear", "-3.75", "-1.65", "1.95", "-1.570796", "0.174533", "0.0"),
    ]
    camera_bridges = [
        (
            camera_name,
            [
                f"/sensing/camera/{camera_name}/image@sensor_msgs/msg/Image[{gazebo_msgs}.Image",
                f"/sensing/camera/{camera_name}/camera_info@sensor_msgs/msg/CameraInfo[{gazebo_msgs}.CameraInfo",
            ],
        )
        for camera_name, *_ in camera_frames
    ]
    if gazebo_packages["supports_service_bridge"]:
        system_bridges.append(
            f"/world/xiamen_empty/set_pose@{set_pose_service_type}"
            f"@{gazebo_msgs}.Pose@{gazebo_msgs}.Boolean"
        )
    lidar_calibration_defaults = [
        ("fl_lidar_1", "7.500", "1.490", "1.25", "0.785398", "0.0", "0.0"),
        ("fr_lidar_1", "7.500", "-1.490", "1.25", "-0.785398", "0.0", "0.0"),
        ("rl_lidar_1", "-7.500", "-1.490", "1.25", "-2.356194", "0.0", "0.0"),
        ("rr_lidar_1", "-7.500", "1.490", "1.25", "2.356194", "0.0", "0.0"),
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
        *[
            (x, y, z, yaw, pitch, roll, f"{camera_name}_camera", f"{camera_name}_camera_tf")
            for camera_name, x, y, z, yaw, pitch, roll in camera_frames
        ],
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument("initial_x", default_value="17.9991"),
            DeclareLaunchArgument("initial_y", default_value="28.0013"),
            DeclareLaunchArgument("initial_yaw", default_value="-1.5708"),
            DeclareLaunchArgument("steering_mode", default_value="axle12_steer"),
            DeclareLaunchArgument("vehicle_load_profile", default_value="empty"),
            DeclareLaunchArgument("surface_friction_coefficient", default_value="0.8"),
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
            DeclareLaunchArgument("enable_camera_topics", default_value="true"),
            DeclareLaunchArgument("enable_lidar_markers", default_value="false"),
            DeclareLaunchArgument("enable_sensor_tf", default_value="true"),
            DeclareLaunchArgument("enable_dynamic_obstacles", default_value="true"),
            DeclareLaunchArgument("enable_static_scene_markers", default_value="false"),
            DeclareLaunchArgument("enable_centerline_trajectory", default_value="true"),
            DeclareLaunchArgument("enable_mpc", default_value="true"),
            DeclareLaunchArgument("enable_remote_drive", default_value="false"),
            DeclareLaunchArgument("enable_remote_protocol_mock", default_value="true"),
            DeclareLaunchArgument("remote_protocol_local_host", default_value="127.0.0.1"),
            DeclareLaunchArgument("remote_protocol_local_port", default_value="46000"),
            DeclareLaunchArgument("remote_protocol_vehicle_host", default_value="127.0.0.1"),
            DeclareLaunchArgument("remote_protocol_vehicle_port", default_value="46001"),
            DeclareLaunchArgument("remote_protocol_endian", default_value="little"),
            DeclareLaunchArgument(
                "remote_protocol_tlv_length_includes_header", default_value="false"
            ),
            DeclareLaunchArgument("remote_network_delay_ms", default_value="50.0"),
            DeclareLaunchArgument("remote_network_jitter_ms", default_value="0.0"),
            DeclareLaunchArgument("remote_network_loss_percent", default_value="0.0"),
            DeclareLaunchArgument("remote_network_seed", default_value="7100"),
            DeclareLaunchArgument(
                "remote_video_delay_ms", default_value=remote_network_delay_ms
            ),
            DeclareLaunchArgument(
                "remote_video_jitter_ms", default_value=remote_network_jitter_ms
            ),
            DeclareLaunchArgument(
                "remote_video_loss_percent", default_value=remote_network_loss_percent
            ),
            DeclareLaunchArgument("remote_video_transport", default_value="h264"),
            DeclareLaunchArgument("remote_video_encoder", default_value="h264_nvenc"),
            DeclareLaunchArgument("remote_video_bitrate_kbps", default_value="1500"),
            DeclareLaunchArgument("remote_video_direct_gz", default_value="true"),
            DeclareLaunchArgument(
                "remote_video_link_capacity_mbps", default_value="100.0"
            ),
            DeclareLaunchArgument("control_authority", default_value="AUTO"),
            DeclareLaunchArgument("enable_vehicle_interface", default_value="true"),
            DeclareLaunchArgument("enable_simulated_chassis_feedback", default_value="true"),
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
                launch_arguments={gazebo_packages["sim_args"]: ["-r -v 4 ", world_path]}.items(),
                condition=IfCondition(use_gazebo_gui),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gz_launch),
                launch_arguments={
                    gazebo_packages["sim_args"]: ["-r -s -v 4 ", world_path]
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
            ),
            *[
                Node(
                    package=gazebo_packages["bridge_package"],
                    executable="parameter_bridge",
                    name=f"agv_camera_bridge_{camera_name}",
                    output="screen",
                    arguments=bridges,
                    condition=IfCondition(enable_camera_topic_bridges),
                )
                for camera_name, bridges in camera_bridges
            ],
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
                        # Axle-center limits corresponding to the supplied inner/outer wheel limits.
                        "steer_lim": 0.5229795759,
                        "steer_lim_front": 0.5229795759,
                        "steer_lim_rear": 0.4505604976,
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
                        # From the supplied axle-to-center-of-gravity table.
                        "axle1_x": 4.65,
                        "axle2_x": 3.0,
                        "axle3_x": -3.0,
                        "axle4_x": -4.65,
                        # Per-axle split of the estimated front/rear tire stiffness above.
                        "axle1_cornering_stiffness_n_rad": 200000.0,
                        "axle2_cornering_stiffness_n_rad": 200000.0,
                        "axle3_cornering_stiffness_n_rad": 200000.0,
                        "axle4_cornering_stiffness_n_rad": 200000.0,
                        "axle1_normal_load_n": 56407.5,
                        "axle2_normal_load_n": 56407.5,
                        "axle3_normal_load_n": 56407.5,
                        "axle4_normal_load_n": 56407.5,
                        "tire_friction_coefficient": surface_friction_coefficient,
                        "vx_rate_lim": 0.35,
                        "max_drive_accel": 0.35,
                        "max_brake_decel": 4.0,
                        "steer_rate_lim": 0.261799,
                        "wheelbase_f": 4.65,
                        "wheelbase_r": 4.65,
                        "wheel_tread": 2.381384,
                        "wheel_radius": 0.66,
                        "enable_ackermann_wheel_angles": True,
                        # Back-calculated Ackermann equivalent lengths for dynamics and TF:
                        # axle 1: 27.4 deg outer -> 33.0 deg inner,
                        # axle 2: 22.6 deg outer -> 30.0 deg inner.
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
                        "wheel_tf_front_x": 4.65,
                        "wheel_tf_mid_front_x": 3.0,
                        "wheel_tf_mid_rear_x": -3.0,
                        "wheel_tf_rear_x": -4.65,
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
                    },
                    vehicle_profile_file,
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
                        "control_topic": "/control/command/auto/control_cmd",
                        "two_axis_steer_topic": "/agv/control/auto/two_axis_steer_cmd",
                        "gear_topic": "/control/command/auto/gear_cmd",
                        "control_rate": 30.0,
                        "prediction_dt": 0.2,
                        "horizon_steps": 12,
                        "target_speed": target_speed,
                        "vx_lim": 5.0,
                        "vx_rate_lim": 0.35,
                        "steer_lim": 0.5229795759,
                        "steer_lim_front": 0.5229795759,
                        "steer_lim_rear": 0.4505604976,
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
                        "wheelbase_f": 4.65,
                        "wheelbase_r": 4.65,
                        "axle2_x": 3.0,
                        "axle3_x": -3.0,
                        "axle4_x": -4.65,
                        "wheel_tread": 2.381384,
                        "ackermann_axle1_wheelbase": 6.116611371515338,
                        "ackermann_axle2_wheelbase": 3.5527407651152627,
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
                    },
                    vehicle_profile_file,
                ],
                condition=IfCondition(enable_mpc),
            ),
            Node(
                package="agv_remote_protocol",
                executable="domain_udp_gateway",
                name="domain_udp_gateway",
                output="screen",
                parameters=[
                    {
                        "local_host": remote_protocol_local_host,
                        "local_port": remote_protocol_local_port,
                        "vehicle_host": remote_protocol_vehicle_host,
                        "vehicle_port": remote_protocol_vehicle_port,
                        "endian": remote_protocol_endian,
                        "tlv_length_includes_header": remote_protocol_tlv_length_includes_header,
                        "send_rate": 40.0,
                        "network_delay_ms": remote_network_delay_value,
                        "network_jitter_ms": remote_network_jitter_value,
                        "network_loss_percent": remote_network_loss_value,
                        "network_seed": remote_network_seed_value,
                    }
                ],
                condition=IfCondition(enable_remote_drive),
            ),
            Node(
                package="agv_remote_protocol",
                executable="mock_br_controller",
                name="mock_br_controller",
                output="screen",
                parameters=[
                    {
                        "local_host": remote_protocol_vehicle_host,
                        "local_port": remote_protocol_vehicle_port,
                        "domain_host": remote_protocol_local_host,
                        "domain_port": remote_protocol_local_port,
                        "endian": remote_protocol_endian,
                        "tlv_length_includes_header": remote_protocol_tlv_length_includes_header,
                        "update_rate": 40.0,
                        "network_delay_ms": remote_network_delay_value,
                        "network_jitter_ms": remote_network_jitter_value,
                        "network_loss_percent": remote_network_loss_value,
                        "network_seed": remote_network_seed_value,
                    }
                ],
                condition=IfCondition(enable_remote_mock),
            ),
            *[
                Node(
                    package="agv_gazebo",
                    executable="h264_video_receiver_node",
                    name=f"h264_video_receiver_{camera_name}",
                    output="screen",
                    parameters=[
                        {
                            "camera_name": camera_name,
                            "decoder_input_port": 15200 + camera_index,
                            "width": remote_camera_receiver_sizes[camera_name][0],
                            "height": remote_camera_receiver_sizes[camera_name][1],
                            "frame_rate": 15.0,
                            "configured_delay_ms": remote_video_delay_value,
                            "stale_after_s": 1.0,
                            "diagnostic_rate": 5.0,
                        }
                    ],
                    condition=IfCondition(enable_remote_h264_video),
                )
                for camera_index, camera_name in enumerate(remote_camera_names)
            ],
            Node(
                package="agv_remote_drive",
                executable="remote_h264_udp_proxy",
                name="remote_h264_udp_proxy",
                output="screen",
                parameters=[
                    {
                        "input_base_port": 15100,
                        "output_base_port": 15200,
                        "delay_ms": remote_video_delay_value,
                        "jitter_ms": remote_video_jitter_value,
                        "loss_percent": remote_video_loss_value,
                        "seed": remote_network_seed_value,
                        "capacity_mbps": remote_video_link_capacity_mbps,
                        "max_queue_delay_ms": 250.0,
                    }
                ],
                condition=IfCondition(enable_remote_direct_h264),
            ),
            *[
                Node(
                    package="agv_gazebo",
                    executable="direct_h264_video_sender_node",
                    name=f"direct_h264_video_sender_{camera_name}",
                    output="screen",
                    parameters=[
                        {
                            "camera_name": camera_name,
                            "ignition_topic": f"/sensing/camera/{camera_name}/image",
                            "udp_port": 15100 + camera_index,
                            "encoder": (
                                "libx264"
                                if camera_name == "right_rear"
                                else remote_video_encoder
                            ),
                            "frame_rate": 15.0,
                            "bitrate_kbps": remote_video_bitrate_value,
                            "output_width": remote_camera_output_sizes[camera_name][0],
                            "output_height": remote_camera_output_sizes[camera_name][1],
                        }
                    ],
                    condition=IfCondition(enable_remote_direct_h264),
                )
                for camera_index, camera_name in enumerate(remote_camera_names)
            ],
            *[
                Node(
                    package="agv_remote_drive",
                    executable="remote_h264_video_link",
                    name=f"remote_h264_video_link_{camera_name}",
                    output="screen",
                    parameters=[
                        {
                            "camera_name": camera_name,
                            "decode_locally": False,
                            # This GPU exposes five concurrent NVENC sessions. Reserve
                            # those sessions for the front and four side/rear feeds so
                            # all six cameras start deterministically.
                            "encoder": (
                                "libx264"
                                if camera_name == "right_rear"
                                else remote_video_encoder
                            ),
                            "frame_rate": 15.0,
                            "bitrate_kbps": remote_video_bitrate_value,
                            "network_delay_ms": remote_video_delay_value,
                            "network_jitter_ms": remote_video_jitter_value,
                            "network_loss_percent": remote_video_loss_value,
                            "network_seed": ParameterValue(
                                PythonExpression(
                                    [remote_network_seed, f" + {camera_index}"]
                                ),
                                value_type=int,
                            ),
                            "link_capacity_mbps": remote_video_per_camera_capacity_value,
                            "proxy_input_base_port": 15100 + camera_index,
                            "decoder_input_base_port": 15200 + camera_index,
                        }
                    ],
                    condition=IfCondition(enable_remote_python_h264),
                )
                for camera_index, camera_name in enumerate(remote_camera_names)
            ],
            Node(
                package="agv_remote_drive",
                executable="remote_video_network_emulator",
                name="remote_video_network_emulator",
                output="screen",
                parameters=[
                    {
                        "network_delay_ms": remote_video_delay_value,
                        "network_jitter_ms": remote_video_jitter_value,
                        "network_loss_percent": remote_video_loss_value,
                        "network_seed": remote_network_seed_value,
                    }
                ],
                condition=IfCondition(enable_remote_raw_video),
            ),
            Node(
                package="agv_remote_drive",
                executable="remote_drive_command_mux",
                name="remote_drive_command_mux",
                output="screen",
                parameters=[
                    {
                        "default_mode": control_authority,
                        "input_timeout": 0.25,
                        "deadman_timeout": 0.35,
                        "switch_speed_limit": 0.05,
                        "remote_max_velocity": 3.0,
                        "remote_max_reverse_velocity": 0.5,
                        "remote_max_acceleration": 0.35,
                        "remote_max_brake_decel": 4.0,
                        "front_steer_limit": 0.5229795759,
                        "rear_steer_limit": 0.4505604976,
                        "require_remote_video": True,
                        "required_video_cameras": ["front"],
                        "remote_video_timeout": 1.5,
                    }
                ],
                condition=IfCondition(enable_control_stack),
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
                        "max_reverse_velocity": 0.5,
                        "max_drive_accel": 0.35,
                        "max_brake_decel": 4.0,
                        "front_steer_limit": 0.5229795759,
                        "rear_steer_limit": 0.4505604976,
                        "stop_acceleration": -1.0,
                        "stop_gear": "drive",
                    }
                ],
                condition=IfCondition(enable_control_stack),
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
                        "max_steer": 0.5229795759,
                    }
                ],
                condition=IfCondition(enable_vehicle_interface),
            ),
            Node(
                package="agv_vehicle_interface",
                executable="simulated_chassis_feedback_node",
                name="simulated_chassis_feedback",
                output="screen",
                parameters=[
                    {
                        "odom_topic": "/localization/kinematic_state",
                        "control_topic": "/control/command/control_cmd",
                        "front_steer_topic": "/agv/status/front_steer",
                        "rear_steer_topic": "/agv/status/rear_steer",
                        "gear_status_topic": "/vehicle/status/gear_status",
                        "control_mode_topic": "/vehicle/status/control_mode",
                        "emergency_stop_topic": "/agv/safety/emergency_stop",
                        "wheel_radius": 0.66,
                        "max_brake_pressure_bar": 160.0,
                        "max_brake_decel": 4.0,
                        "max_drive_accel": 0.35,
                    },
                    vehicle_profile_file,
                ],
                condition=IfCondition(enable_simulated_chassis_feedback),
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
                        "update_rate": 20.0,
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
