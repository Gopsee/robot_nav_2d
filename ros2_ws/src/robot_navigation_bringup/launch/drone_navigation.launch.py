from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    bringup_dir = get_package_share_directory('robot_navigation_bringup')
    rviz_config = os.path.join(bringup_dir, 'config', 'drone_nav.rviz')

    # ── Drone simulator nodes (namespace='drone_1' maps all relative topics to /drone/*) ──

    drone_kinematics = Node(
        package='drone_simulator',
        executable='drone_kinematics_node',
        namespace='drone_1',
        output='screen',
        parameters=[{
            'cruise_altitude': 5.0,
            'altitude_kp':     2.0,
            'max_vz':          2.0,
            'update_rate':    50.0,
        }],
    )

    drone_global_planner = Node(
        package='drone_simulator',
        executable='drone_global_planner_node',
        namespace='drone_1',
        output='screen',
        parameters=[{
            'global_frame':    'map',
            'base_frame':      'drone_base_link',
            'num_waypoints':   20,
            'cruise_altitude': 5.0,
        }],
    )

    drone_lidar = Node(
        package='drone_simulator',
        executable='drone_lidar_node',
        namespace='drone_1',
        output='screen',
        remappings=[
            ('/estimated_pose', 'estimated_pose'),
            ('/scan',           'scan'),
            ('/point_cloud',    'point_cloud'),
            ('/obstacles',      'obstacles'),
        ],
    )

    # ── Reused navigation stack — same packages as ground robot, different namespace ──
    # With namespace='drone_1', relative topics become /drone/input_path, /drone/smooth_path,
    # /drone/odom, /drone/cmd_vel, /drone/follow_path, etc.

    path_smoothing = Node(
        package='path_smoothing',
        executable='path_smoothing_node',
        namespace='drone_1',
        output='screen',
    )

    trajectory_generator = Node(
        package='trajectory_generator',
        executable='trajectory_generator_node',
        namespace='drone_1',
        output='screen',
        parameters=[{
            'v_const':  1.5,    # faster than ground robot
            'frame_id': 'map',
        }],
    )

    trajectory_controller = Node(
        package='trajectory_controller',
        executable='trajectory_controller_node',
        namespace='drone_1',
        output='screen',
        parameters=[{
            'lookahead_dist': 2.0,   # larger lookahead for aerial speed
            'linear_speed':   1.5,
        }],
    )

    # ── TF tree ───────────────────────────────────────────────────────────────

    # map → odom (identity, same as ground robot)
    static_tf_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_odom_drone',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
    )

    # drone_base_link → drone_lidar (lidar mounted 0.15 m above CoM)
    static_tf_drone_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_drone_lidar',
        arguments=['0', '0', '0.15', '0', '0', '0', 'drone_base_link', 'drone_lidar'],
    )

    # ── Visualization ─────────────────────────────────────────────────────────

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
    )

    return LaunchDescription([
        drone_kinematics,
        drone_global_planner,
        drone_lidar,
        path_smoothing,
        trajectory_generator,
        trajectory_controller,
        static_tf_map_odom,
        static_tf_drone_lidar,
        rviz,
    ])
