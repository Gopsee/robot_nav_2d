from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os

def generate_launch_description():
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    world_launch = os.path.join(turtlebot3_gazebo_dir, 'launch', 'turtlebot3_world.launch.py')

    turtlebot3_description_dir = get_package_share_directory('turtlebot3_description')
    robot_navigation_bringup_dir = get_package_share_directory('robot_navigation_bringup')

    return LaunchDescription([
        # # 1️⃣ Set TurtleBot model
        SetEnvironmentVariable(name='TURTLEBOT3_MODEL', value='burger'),

        # # 2️⃣ Include Gazebo world
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(world_launch)
        ),

        # # 3️⃣ Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='state_publisher',
            output='screen',
            arguments=[os.path.join(
                turtlebot3_description_dir,
                'urdf',
                'turtlebot3_burger.urdf.xacro')]
        ),

        # 4️⃣ Simple Localization
        Node(
            package='simulator',
            executable='simple_localization_node',
            name='simple_localization',
            output='screen'
        ),

        # 5️⃣ Path Smoothing
        Node(
            package='path_smoothing',
            executable='path_smoothing_node',
            name='path_smoothing',
            output='screen',
            parameters=[{'sigma': 0.5}]
        ),

        # 6️⃣ Trajectory Generation
        Node(
            package='trajectory_generator',
            executable='trajectory_generator_node',
            name='trajectory_generator',
            output='screen',
            parameters=[{'v_const': 0.3}]
        ),

        # 7️⃣ Trajectory Controller
        Node(
            package='trajectory_controller',
            executable='trajectory_controller_node',
            name='controller',
            output='screen',
            parameters=[
                {'lookahead_dist': 0.4},
                {'linear_speed': 0.3}
            ]
        ),

        # 8️⃣ RViz Visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(
                robot_navigation_bringup_dir,
                'config',
                'navigation_view.rviz')]
        ),
    ])
