from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    map_path = LaunchConfiguration("map_path")
    use_rviz = LaunchConfiguration("use_rviz")
    use_static_sensor_tf = LaunchConfiguration("use_static_sensor_tf")
    enable_centerline_trajectory = LaunchConfiguration("enable_centerline_trajectory")
    enable_mpc = LaunchConfiguration("enable_mpc")
    enable_safety_supervisor = LaunchConfiguration("enable_safety_supervisor")
    enable_vehicle_interface = LaunchConfiguration("enable_vehicle_interface")
    target_speed = LaunchConfiguration("target_speed")
    route_lane_index = LaunchConfiguration("route_lane_index")
    steering_mode = LaunchConfiguration("steering_mode")
    transport = LaunchConfiguration("transport")
    can_interface = LaunchConfiguration("can_interface")
    udp_host = LaunchConfiguration("udp_host")
    udp_port = LaunchConfiguration("udp_port")
    udp_bind_host = LaunchConfiguration("udp_bind_host")
    udp_bind_port = LaunchConfiguration("udp_bind_port")
    dry_run = LaunchConfiguration("dry_run")
    shadow_mode = LaunchConfiguration("shadow_mode")
    autonomous_enable = LaunchConfiguration("autonomous_enable")
    publish_feedback_from_commands = LaunchConfiguration("publish_feedback_from_commands")
    max_velocity = LaunchConfiguration("max_velocity")
    max_acceleration = LaunchConfiguration("max_acceleration")
    max_brake_decel = LaunchConfiguration("max_brake_decel")
    rviz_config = LaunchConfiguration("rviz_config")

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

    default_map_path = PathJoinSubstitution(
        [FindPackageShare("agv_maps"), "map", "xiamen-prod.osm"]
    )
    default_rviz_config = PathJoinSubstitution(
        [FindPackageShare("agv_gazebo"), "config", "stage4_lidar.rviz"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("map_path", default_value=default_map_path),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("use_static_sensor_tf", default_value="true"),
            DeclareLaunchArgument("enable_centerline_trajectory", default_value="true"),
            DeclareLaunchArgument("enable_mpc", default_value="true"),
            DeclareLaunchArgument("enable_safety_supervisor", default_value="true"),
            DeclareLaunchArgument("enable_vehicle_interface", default_value="true"),
            DeclareLaunchArgument("target_speed", default_value="1.0"),
            DeclareLaunchArgument("route_lane_index", default_value="2"),
            DeclareLaunchArgument("steering_mode", default_value="axle12_steer"),
            DeclareLaunchArgument("transport", default_value="dry_run"),
            DeclareLaunchArgument("can_interface", default_value="can0"),
            DeclareLaunchArgument("udp_host", default_value="192.168.1.10"),
            DeclareLaunchArgument("udp_port", default_value="5000"),
            DeclareLaunchArgument("udp_bind_host", default_value="0.0.0.0"),
            DeclareLaunchArgument("udp_bind_port", default_value="0"),
            DeclareLaunchArgument("dry_run", default_value="true"),
            DeclareLaunchArgument("shadow_mode", default_value="true"),
            DeclareLaunchArgument("autonomous_enable", default_value="false"),
            DeclareLaunchArgument("publish_feedback_from_commands", default_value="false"),
            DeclareLaunchArgument("max_velocity", default_value="1.0"),
            DeclareLaunchArgument("max_acceleration", default_value="0.2"),
            DeclareLaunchArgument("max_brake_decel", default_value="1.0"),
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
            DeclareLaunchArgument("rviz_config", default_value=default_rviz_config),
            Node(
                package="agv_map_visualizer",
                executable="osm_marker_publisher.py",
                name="real_osm_marker_publisher",
                output="screen",
                parameters=[
                    {
                        "map_path": map_path,
                        "output_topic": "/map/vector_map_marker",
                        "frame_id": "map",
                    }
                ],
            ),
            Node(
                package="agv_mpc_controller",
                executable="centerline_trajectory_publisher",
                name="real_centerline_trajectory_publisher",
                output="screen",
                parameters=[
                    {
                        "map_path": map_path,
                        "output_topic": "/planning/scenario_planning/trajectory",
                        "marker_topic": "/visualization/centerline_trajectory_marker",
                        "frame_id": "map",
                        "target_speed": target_speed,
                        "min_speed": 0.2,
                        "max_lateral_accel": 0.2,
                        "curvature_window": 4,
                        "speed_smoothing_passes": 3,
                        "short_straight_speed_limit": 0.5,
                        "turn_speed_limit": 0.5,
                        "turn_curvature_threshold": 0.015,
                        "turn_entry_hold_distance": 8.0,
                        "turn_exit_hold_distance": 12.0,
                        "max_profile_accel": max_acceleration,
                        "max_profile_decel": max_brake_decel,
                        "path_smoothing_passes": 3,
                        "point_spacing": 0.5,
                        "route_mode": "port_loop",
                        "route_lane_index": route_lane_index,
                        "route_turn_radius": 15.0,
                        "lane_change_length": 16.0,
                    }
                ],
                condition=IfCondition(enable_centerline_trajectory),
            ),
            Node(
                package="agv_mpc_controller",
                executable="sampling_mpc_controller",
                name="real_sampling_mpc_controller",
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
                        "target_speed": target_speed,
                        "vx_lim": max_velocity,
                        "vx_rate_lim": max_acceleration,
                        "steer_lim": 0.47822,
                        "steer_lim_front": 0.47822,
                        "steer_lim_rear": 0.39444,
                        "wheelbase_f": 4.65,
                        "wheelbase_r": 4.65,
                        "axle2_x": 3.0,
                        "axle3_x": -3.0,
                        "axle4_x": -4.65,
                        "steering_mode": steering_mode,
                        "enable_loop_stop": True,
                        "closed_trajectory": True,
                        "enable_error_log": True,
                        "error_log_path": "/tmp/agv_real_mpc_error.csv",
                    }
                ],
                condition=IfCondition(enable_mpc),
            ),
            Node(
                package="agv_safety_supervisor",
                executable="safety_supervisor_node",
                name="real_agv_safety_supervisor",
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
                        "max_velocity": max_velocity,
                        "max_drive_accel": max_acceleration,
                        "max_brake_decel": max_brake_decel,
                        "front_steer_limit": 0.47822,
                        "rear_steer_limit": 0.39444,
                        "stop_acceleration": -1.0,
                        "stop_gear": "drive",
                    }
                ],
                condition=IfCondition(enable_safety_supervisor),
            ),
            Node(
                package="agv_vehicle_interface",
                executable="real_vehicle_interface_node",
                name="real_vehicle_interface",
                output="screen",
                parameters=[
                    {
                        "control_topic": "/control/command/control_cmd",
                        "two_axis_steer_topic": "/agv/control/two_axis_steer_cmd",
                        "gear_topic": "/control/command/gear_cmd",
                        "transport": transport,
                        "can_interface": can_interface,
                        "udp_host": udp_host,
                        "udp_port": udp_port,
                        "udp_bind_host": udp_bind_host,
                        "udp_bind_port": udp_bind_port,
                        "dry_run": dry_run,
                        "shadow_mode": shadow_mode,
                        "autonomous_enable": autonomous_enable,
                        "publish_feedback_from_commands": publish_feedback_from_commands,
                        "command_timeout": 0.2,
                        "max_velocity": max_velocity,
                        "max_acceleration": max_acceleration,
                        "max_brake_decel": max_brake_decel,
                        "front_steer_limit": 0.47822,
                        "rear_steer_limit": 0.39444,
                        "steering_mode": "axle12",
                    }
                ],
                condition=IfCondition(enable_vehicle_interface),
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="real_imu_tf",
                output="screen",
                arguments=[
                    imu_x,
                    imu_y,
                    imu_z,
                    imu_yaw,
                    imu_pitch,
                    imu_roll,
                    "base_link",
                    imu_frame_id,
                ],
                condition=IfCondition(use_static_sensor_tf),
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="real_gnss_tf",
                output="screen",
                arguments=[
                    gnss_x,
                    gnss_y,
                    gnss_z,
                    gnss_yaw,
                    gnss_pitch,
                    gnss_roll,
                    "base_link",
                    gnss_frame_id,
                ],
                condition=IfCondition(use_static_sensor_tf),
            ),
            Node(
                package="agv_map_visualizer",
                executable="agv_vehicle_marker_publisher",
                name="real_agv_vehicle_marker_publisher",
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
                package="rviz2",
                executable="rviz2",
                name="real_rviz2",
                output="screen",
                arguments=["-d", rviz_config],
                condition=IfCondition(use_rviz),
            ),
        ]
    )
