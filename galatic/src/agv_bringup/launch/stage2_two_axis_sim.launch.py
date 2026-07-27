from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    map_path = LaunchConfiguration("map_path")
    projector_info_path = LaunchConfiguration("projector_info_path")
    rviz_config = LaunchConfiguration("rviz_config")
    use_rviz = LaunchConfiguration("use_rviz")
    steering_mode = LaunchConfiguration("steering_mode")
    vehicle_load_profile = LaunchConfiguration("vehicle_load_profile")
    surface_friction_coefficient = LaunchConfiguration("surface_friction_coefficient")
    initial_x = LaunchConfiguration("initial_x")
    initial_y = LaunchConfiguration("initial_y")
    initial_yaw = LaunchConfiguration("initial_yaw")

    default_map_path = PathJoinSubstitution(
        [FindPackageShare("agv_maps"), "map", "xiamen-prod.osm"]
    )
    default_projector_info_path = PathJoinSubstitution(
        [FindPackageShare("agv_maps"), "map", "map_projector_info.yaml"]
    )
    default_rviz_config = PathJoinSubstitution(
        [FindPackageShare("autoware_launch"), "rviz", "autoware.rviz"]
    )
    vehicle_profile_file = PathJoinSubstitution(
        [
            FindPackageShare("agv_vehicle_model"),
            "config",
            PythonExpression(["'", vehicle_load_profile, "' + '.param.yaml'"]),
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("map_path", default_value=default_map_path),
            DeclareLaunchArgument(
                "projector_info_path", default_value=default_projector_info_path
            ),
            DeclareLaunchArgument("rviz_config", default_value=default_rviz_config),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("steering_mode", default_value="asymmetric_4ws"),
            DeclareLaunchArgument("vehicle_load_profile", default_value="empty"),
            DeclareLaunchArgument("surface_friction_coefficient", default_value="0.8"),
            DeclareLaunchArgument("initial_x", default_value="9.9991"),
            DeclareLaunchArgument("initial_y", default_value="28.0013"),
            DeclareLaunchArgument("initial_yaw", default_value="-1.5708"),
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
                executable="agv_two_axis_simulator_node",
                name="agv_two_axis_simulator",
                output="screen",
                parameters=[
                    {
                        "frame_id": "map",
                        "child_frame_id": "base_link",
                        "steering_mode": steering_mode,
                        "dt": 0.025,
                        "vx_lim": 5.0,
                        # Axle-center limits corresponding to the supplied inner/outer wheel limits.
                        "steer_lim": 0.5229795759,
                        "steer_lim_front": 0.5229795759,
                        "steer_lim_rear": 0.4505604976,
                        "vehicle_mass_kg": 23000.0,
                        "yaw_inertia_kgm2": 432363.7513,
                        "front_cornering_stiffness_n_rad": 400000.0,
                        "rear_cornering_stiffness_n_rad": 400000.0,
                        "axle_count": 4,
                        "wheel_count": 8,
                        "axle_load_kg": 5750.0,
                        "wheel_load_kg": 2875.0,
                        "front_normal_load_n": 112815.0,
                        "rear_normal_load_n": 112815.0,
                        "axle1_x": 4.65,
                        "axle2_x": 3.0,
                        "axle3_x": -3.0,
                        "axle4_x": -4.65,
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
                        "center_of_gravity_height_m": 0.8,
                        "rolling_resistance_coefficient": 0.012,
                        "aerodynamic_drag_coefficient": 1.10,
                        "frontal_area_m2": 7.936695,
                        "air_density_kg_m3": 1.225,
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
                        "initial_x": initial_x,
                        "initial_y": initial_y,
                        "initial_yaw": initial_yaw,
                        "publish_tf": True,
                    },
                    vehicle_profile_file,
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
                        "width": 2.98,
                        "height": 2.54,
                        "wheelbase": 9.30,
                        "wheel_tread": 2.381384,
                    }
                ],
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
