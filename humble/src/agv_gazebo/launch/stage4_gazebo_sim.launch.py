import os
import shutil
import tempfile

from ament_index_python.packages import PackageNotFoundError
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
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
    patched_model_path = os.path.join(patched_models_path, "agv_with_lidars")
    source_model_path = os.path.join(models_path, "agv_with_lidars")
    if os.path.exists(patched_model_path):
        shutil.rmtree(patched_model_path)
    shutil.copytree(source_model_path, patched_model_path)
    model_sdf_path = os.path.join(patched_model_path, "model.sdf")
    with open(model_sdf_path, "r", encoding="utf-8") as src:
        model = src.read().replace('<sdf version="1.9">', '<sdf version="1.8">')
    with open(model_sdf_path, "w", encoding="utf-8") as dst:
        dst.write(model)

    return patched_path, os.pathsep.join([patched_models_path, models_path])


def generate_launch_description():
    map_path = LaunchConfiguration("map_path")
    projector_info_path = LaunchConfiguration("projector_info_path")
    initial_x = LaunchConfiguration("initial_x")
    initial_y = LaunchConfiguration("initial_y")
    initial_yaw = LaunchConfiguration("initial_yaw")
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
    pkg_share = get_package_share_directory("agv_gazebo")
    models_path = os.path.join(pkg_share, "models")
    world_path = os.path.join(pkg_share, "worlds", "xiamen_empty.sdf")
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

    lidar_bridges = [
        f"/sensing/lidar/fl_lidar_1/points@sensor_msgs/msg/PointCloud2[{gazebo_msgs}.PointCloudPacked",
        f"/sensing/lidar/fr_lidar_1/points@sensor_msgs/msg/PointCloud2[{gazebo_msgs}.PointCloudPacked",
        f"/sensing/lidar/rl_lidar_1/points@sensor_msgs/msg/PointCloud2[{gazebo_msgs}.PointCloudPacked",
        f"/sensing/lidar/rr_lidar_1/points@sensor_msgs/msg/PointCloud2[{gazebo_msgs}.PointCloudPacked",
        f"/sensing/lidar/fl_lidar_2/points@sensor_msgs/msg/PointCloud2[{gazebo_msgs}.PointCloudPacked",
        f"/sensing/lidar/rr_lidar_2/points@sensor_msgs/msg/PointCloud2[{gazebo_msgs}.PointCloudPacked",
        f"/clock@rosgraph_msgs/msg/Clock[{gazebo_msgs}.Clock",
    ]
    camera_frames = [
        ("front", "7.65", "0.0", "1.35", "0.0", "0.0", "0.0"),
        ("rear", "-7.65", "0.0", "1.35", "3.141593", "0.0", "0.0"),
        ("left_front", "3.75", "1.65", "1.35", "1.570796", "0.0", "0.0"),
        ("left_rear", "-3.75", "1.65", "1.35", "1.570796", "0.0", "0.0"),
        ("right_front", "3.75", "-1.65", "1.35", "-1.570796", "0.0", "0.0"),
        ("right_rear", "-3.75", "-1.65", "1.35", "-1.570796", "0.0", "0.0"),
    ]
    camera_bridges = [
        bridge
        for camera_name, *_ in camera_frames
        for bridge in (
            f"/sensing/camera/{camera_name}/image@sensor_msgs/msg/Image[{gazebo_msgs}.Image",
            f"/sensing/camera/{camera_name}/camera_info@sensor_msgs/msg/CameraInfo[{gazebo_msgs}.CameraInfo",
        )
    ]
    lidar_bridges.extend(camera_bridges)
    if gazebo_packages["supports_service_bridge"]:
        lidar_bridges.append(
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
            DeclareLaunchArgument("map_path", default_value=default_map_path),
            DeclareLaunchArgument(
                "projector_info_path", default_value=default_projector_info_path
            ),
            DeclareLaunchArgument("initial_x", default_value="9.9991"),
            DeclareLaunchArgument("initial_y", default_value="28.0013"),
            DeclareLaunchArgument("initial_yaw", default_value="-1.5708"),
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
            DeclareLaunchArgument("reference_latitude", default_value="0.0050"),
            DeclareLaunchArgument("reference_longitude", default_value="0.0075"),
            DeclareLaunchArgument("reference_altitude", default_value="0.0"),
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
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gz_launch),
                launch_arguments={gazebo_packages["sim_args"]: ["-r -v 4 ", world_path]}.items(),
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
            ),
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
                        }
                    ],
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
            ),
            Node(
                package="agv_map_visualizer",
                executable="lanelet2_marker_publisher",
                name="lanelet2_marker_publisher",
                output="screen",
            ),
            Node(
                package="agv_vehicle_model",
                executable="agv_standalone_simulator_node",
                name="agv_two_axis_simulator",
                output="screen",
                parameters=[
                    {
                        "initial_x": initial_x,
                        "initial_y": initial_y,
                        "initial_yaw": initial_yaw,
                    }
                ],
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
                    }
                ],
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
                        "length": 15.0,
                        "width": 3.119,
                        "height": 2.54,
                        "wheelbase": 8.959003,
                        "wheel_tread": 2.381384,
                    }
                ],
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
                        "sample_step": 2,
                        "max_points_per_cloud": 20000,
                    }
                ],
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
                )
                for x, y, z, yaw, pitch, roll, child_frame, node_name in sensor_static_transforms
            ],
        ]
    )
