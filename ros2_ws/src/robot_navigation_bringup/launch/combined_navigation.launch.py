from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro


def generate_launch_description():
    bringup_dir = get_package_share_directory('robot_navigation_bringup')
    xacro_file  = os.path.join(bringup_dir, 'urdf', 'tractor.urdf.xacro')
    rviz_config = os.path.join(bringup_dir, 'config', 'combined_nav.rviz')

    robot_desc = xacro.process_file(xacro_file).toxml()

    # ── Robot 1 ────────────────────────────────────────────────────────────────

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='robot_1',
        output='screen',
        parameters=[{'robot_description': robot_desc}],
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        namespace='robot_1',
        output='screen',
    )

    robot_localization = Node(
        package='simulator',
        executable='simple_localization_node',
        namespace='robot_1',
        output='screen',
        remappings=[
            ('/cmd_vel',        'cmd_vel'),
            ('/odom',           'odom'),
            ('/estimated_pose', 'estimated_pose'),
        ],
    )

    robot_global_planner = Node(
        package='simulator',
        executable='global_planner_node',
        namespace='robot_1',
        output='screen',
    )

    robot_lidar = Node(
        package='simulator',
        executable='lidar_node',
        namespace='robot_1',
        output='screen',
        remappings=[
            ('/estimated_pose', 'estimated_pose'),
            ('/scan',           'scan'),
            ('/point_cloud',    'point_cloud'),
            ('/obstacles',      'obstacles'),
        ],
    )

    robot_path_smoothing = Node(
        package='path_smoothing',
        executable='path_smoothing_node',
        namespace='robot_1',
        output='screen',
    )

    robot_trajectory_generator = Node(
        package='trajectory_generator',
        executable='trajectory_generator_node',
        namespace='robot_1',
        output='screen',
    )

    robot_trajectory_controller = Node(
        package='trajectory_controller',
        executable='trajectory_controller_node',
        namespace='robot_1',
        output='screen',
    )

    # ── Drone 1 ────────────────────────────────────────────────────────────────

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

    drone_path_smoothing = Node(
        package='path_smoothing',
        executable='path_smoothing_node',
        namespace='drone_1',
        output='screen',
    )

    drone_trajectory_generator = Node(
        package='trajectory_generator',
        executable='trajectory_generator_node',
        namespace='drone_1',
        output='screen',
        parameters=[{
            'v_const':  1.5,
            'frame_id': 'map',
        }],
    )

    drone_trajectory_controller = Node(
        package='trajectory_controller',
        executable='trajectory_controller_node',
        namespace='drone_1',
        output='screen',
        parameters=[{
            'lookahead_dist': 2.0,
            'linear_speed':   1.5,
        }],
    )

    # ── Shared TF ─────────────────────────────────────────────────────────────

    static_tf_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
    )

    # Robot TF: base_footprint → base_link → rslidar_base
    static_tf_base_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_link',
        arguments=['0', '0', '0.01', '0', '0', '0', 'base_footprint', 'base_link'],
    )

    static_tf_rslidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_rslidar_base',
        arguments=['0', '0', '0.99', '0', '0', '0', 'base_link', 'rslidar_base'],
    )

    # Drone TF: drone_base_link → drone_lidar
    static_tf_drone_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_drone_lidar',
        arguments=['0', '0', '0.15', '0', '0', '0', 'drone_base_link', 'drone_lidar'],
    )

    # ── Random goal publisher ─────────────────────────────────────────────────

    random_goal = Node(
        package='simulator',
        executable='random_goal_node',
        output='screen',
    )

    # ── RViz ──────────────────────────────────────────────────────────────────

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
    )

    return LaunchDescription([
        # Robot 1
        robot_state_publisher,
        joint_state_publisher,
        robot_localization,
        robot_global_planner,
        robot_lidar,
        robot_path_smoothing,
        robot_trajectory_generator,
        robot_trajectory_controller,
        # Drone 1
        drone_kinematics,
        drone_global_planner,
        drone_lidar,
        drone_path_smoothing,
        drone_trajectory_generator,
        drone_trajectory_controller,
        # TF
        static_tf_map_odom,
        static_tf_base_link,
        static_tf_rslidar,
        static_tf_drone_lidar,
        # Goals + Visualization
        random_goal,
        rviz,
    ])
