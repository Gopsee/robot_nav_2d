from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

import xacro

def generate_launch_description():
    bringup_dir = get_package_share_directory('robot_navigation_bringup')

    xacro_file = os.path.join(bringup_dir, 'urdf', 'tractor.urdf.xacro')
    # urdf_file = os.path.join(bringup_dir, 'urdf', 'simple_robot.urdf')
    rviz_config = os.path.join(bringup_dir, 'config', 'turtlebot_nav.rviz')

    # # Read URDF contents
    # with open(urdf_file, 'r') as infp:
    #     robot_desc = infp.read()

    # Process xacro → URDF text
    robot_desc = xacro.process_file(xacro_file).toxml()
    
    # Robot State Publisher (publishes /robot_description and /tf)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}],
    )

    joint_state_pub_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )

    # Existing nodes in your stack
    path_smoothing_node = Node(
        package='path_smoothing',
        executable='path_smoothing_node',
        output='screen',
        # remappings=[('/path_smoothing/smooth_path', '/trajectory_generator/smooth_path')]
    )

    trajectory_generator_node = Node(
        package='trajectory_generator',
        executable='trajectory_generator_node',
        output='screen',
        # remappings=[('/path_smoothing/smooth_path', '/trajectory_generator/smooth_path')]
    )

    trajectory_controller_node = Node(
        package='trajectory_controller',
        executable='trajectory_controller_node',
        output='screen',
        # remappings=[('/trajectory_controller/cmd_vel', '/cmd_vel')]
    )

    simulator_node = Node(
        package='simulator',
        executable='simple_localization_node',
        output='screen'
    )

    global_planner_node = Node(
        package='simulator',
        executable='global_planner_node',
        output='screen',
        # remappings=[('/global_planner/input_path', '/path_smoothing/input_path')]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config]
    )

    static_tf_base_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_link',
        arguments=['0', '0', '0.01', '0', '0', '0', 'base_footprint', 'base_link']
    )

    static_tf_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )
    
    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_pub_node,
        path_smoothing_node,
        trajectory_generator_node,
        trajectory_controller_node,
        simulator_node,
        global_planner_node,
        rviz_node,
        static_tf_base_link,
        static_tf_map_odom
    ])