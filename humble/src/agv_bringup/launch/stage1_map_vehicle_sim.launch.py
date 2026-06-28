from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    map_path = LaunchConfiguration("map_path")
    projector_info_path = LaunchConfiguration("projector_info_path")
    rviz_config = LaunchConfiguration("rviz_config")
    use_rviz = LaunchConfiguration("use_rviz")

    default_map_path = PathJoinSubstitution(
        [FindPackageShare("agv_maps"), "map", "xiamen-prod.osm"]
    )
    default_projector_info_path = PathJoinSubstitution(
        [FindPackageShare("agv_maps"), "map", "map_projector_info.yaml"]
    )
    default_rviz_config = PathJoinSubstitution(
        [FindPackageShare("autoware_launch"), "rviz", "autoware.rviz"]
    )
    vehicle_info_param = PathJoinSubstitution(
        [
            FindPackageShare("autoware_vehicle_info_utils"),
            "config",
            "vehicle_info.param.yaml",
        ]
    )
    vehicle_characteristics_param = PathJoinSubstitution(
        [
            FindPackageShare("autoware_simple_planning_simulator"),
            "param",
            "vehicle_characteristics.param.yaml",
        ]
    )

    simulator_remappings = [
        ("input/vector_map", "/map/vector_map"),
        ("input/initialpose", "/initialpose3d"),
        ("input/ackermann_control_command", "/control/command/control_cmd"),
        ("input/manual_ackermann_control_command", "/vehicle/command/manual_control_cmd"),
        ("input/gear_command", "/control/command/gear_cmd"),
        ("input/manual_gear_command", "/vehicle/command/manual_gear_command"),
        ("input/turn_indicators_command", "/control/command/turn_indicators_cmd"),
        ("input/hazard_lights_command", "/control/command/hazard_lights_cmd"),
        ("input/trajectory", "/planning/trajectory"),
        ("input/engage", "/vehicle/engage"),
        ("input/control_mode_request", "/control/control_mode_request"),
        ("output/odometry", "/localization/kinematic_state"),
        ("output/acceleration", "/localization/acceleration"),
        ("output/pose", "/localization/pose_estimator/pose_with_covariance"),
        ("output/twist", "/vehicle/status/velocity_status"),
        ("output/imu", "/sensing/imu/imu_data"),
        ("output/steering", "/vehicle/status/steering_status"),
        ("output/gear_report", "/vehicle/status/gear_status"),
        ("output/turn_indicators_report", "/vehicle/status/turn_indicators_status"),
        ("output/hazard_lights_report", "/vehicle/status/hazard_lights_status"),
        ("output/control_mode_report", "/vehicle/status/control_mode"),
        ("output/actuation_status", "/vehicle/status/actuation_status"),
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument("map_path", default_value=default_map_path),
            DeclareLaunchArgument(
                "projector_info_path", default_value=default_projector_info_path
            ),
            DeclareLaunchArgument("rviz_config", default_value=default_rviz_config),
            DeclareLaunchArgument("use_rviz", default_value="true"),
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
                package="agv_map_visualizer",
                executable="agv_vehicle_marker_publisher",
                name="agv_vehicle_marker_publisher",
                output="screen",
                parameters=[
                    {
                        "input_topic": "/localization/kinematic_state",
                        "output_topic": "/visualization/agv_vehicle_marker",
                        "length": 3.0,
                        "width": 1.0,
                        "height": 0.8,
                        "wheelbase": 2.0,
                        "wheel_tread": 1.36,
                    }
                ],
            ),
            Node(
                package="autoware_simple_planning_simulator",
                executable="autoware_simple_planning_simulator_node",
                namespace="simulation",
                name="simple_planning_simulator",
                output="screen",
                parameters=[
                    vehicle_info_param,
                    vehicle_characteristics_param,
                    {
                        "simulated_frame_id": "base_link",
                        "origin_frame_id": "map",
                        "wheel_base": 2.0,
                        "wheel_tread": 1.36,
                        "front_overhang": 0.5,
                        "rear_overhang": 0.5,
                        "left_overhang": 0.1,
                        "right_overhang": 0.1,
                        "vehicle_model_type": "DELAY_STEER_VEL",
                        "initialize_source": "ORIGIN",
                        "initial_engage_state": True,
                        "timer_sampling_time_ms": 25,
                        "add_measurement_noise": False,
                        "enable_road_slope_simulation": False,
                        "vel_lim": 5.0,
                        "vel_rate_lim": 2.0,
                        "steer_lim": 0.7,
                        "steer_rate_lim": 1.5,
                        "vel_time_delay": 0.1,
                        "vel_time_constant": 0.2,
                        "steer_time_delay": 0.1,
                        "steer_time_constant": 0.2,
                        "steer_dead_band": 0.0,
                    },
                ],
                remappings=simulator_remappings,
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
                condition=IfCondition(use_rviz),
            ),
        ]
    )
