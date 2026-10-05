"""
Launch the 'root' hexapod in Gazebo Fortress (gz sim) with ros2_control on ROS2 Humble.

Usage:
    ros2 launch root_description gz_sim.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription, RegisterEventHandler, TimerAction, ExecuteProcess
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import xacro


def generate_launch_description():
    pkg_share = get_package_share_directory('root_description')

    xacro_file = os.path.join(pkg_share, 'urdf', 'root.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {'robot_description': robot_description_config.toxml()}

    controllers_yaml = os.path.join(pkg_share, 'config', 'controllers.yaml')

    # --- robot_state_publisher ---
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}],
    )

    # --- Gazebo Fortress ---
    world_file = os.path.join(pkg_share, 'worlds', 'hexapod_world.sdf')
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py',
            ])
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    # --- Spawn robot ---
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'root',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.22',
            '-R', '1.5708',
            '-P', '0.0',
            '-Y', '1.5708',
        ],
        output='screen',
    )

    # --- Bridge /clock, /tf, /odom ---
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/model/root/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            '/model/root/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
        ],
        remappings=[
            ('/model/root/tf', '/tf'),
            ('/model/root/odometry', '/odom'),
        ],
        output='screen',
    )

    # --- Controller spawners ---
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '60',
        ],
    )

    leg_trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'leg_trajectory_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '60',
        ],
    )

    # --- RViz ---
    rviz_config = os.path.join(pkg_share, 'rviz', 'root.rviz')
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
    )

    # --- hexa_node ---
    # Remapped so it publishes to /hexa_node/joint_trajectory instead of
    # directly to the controller. sim_relay picks it up, applies direction
    # corrections, and forwards to /leg_trajectory_controller/joint_trajectory.
    hexa_node = Node(
        package='hexa_control',
        executable='hexa_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
        remappings=[
            ('/leg_trajectory_controller/joint_trajectory',
             '/hexa_node/joint_trajectory'),
        ],
    )

    # --- sim_relay: applies hardware direction corrections for simulation ---
    sim_relay = Node(
        package='hexa_control',
        executable='sim_relay',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # Controllers start after spawn_robot exits (spawn_robot exits reliably)
    delay_controllers_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[
                joint_state_broadcaster_spawner,
                leg_trajectory_controller_spawner,
            ],
        )
    )

    # hexa_node and sim_relay start 5s after launch via TimerAction
    delay_hexa_node = TimerAction(
        period=5.0,
        actions=[hexa_node, sim_relay],
    )

    return LaunchDescription([
        gz_sim,
        gz_bridge,
        robot_state_publisher,
        rviz2,
        spawn_robot,
        delay_controllers_after_spawn,
        delay_hexa_node,
    ])
