#!/usr/bin/env python3
"""
Full Demo Launch File - Launches all nodes and Gazebo simulation
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, NotSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directory
    pkg_share = FindPackageShare('bodyguard_drone')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    headless = LaunchConfiguration('headless', default='false')
    
    # Paths to world and models
    world_file = PathJoinSubstitution([pkg_share, 'worlds', 'bodyguard_world.sdf'])
    models_path = PathJoinSubstitution([pkg_share, 'models'])
    
    # Gazebo server (simulation)
    gazebo_server = ExecuteProcess(
        cmd=['gzserver', world_file, '-s', 'libgazebo_ros_init.so', 
             '-s', 'libgazebo_ros_factory.so'],
        output='screen',
        additional_env={'GAZEBO_MODEL_PATH': models_path}
    )
    
    # Gazebo client (GUI) - only if not headless
    gazebo_client = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
        condition=UnlessCondition(headless)
    )
    
    # Spawn drone model
    spawn_drone = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'bodyguard_drone',
            '-file', PathJoinSubstitution([pkg_share, 'models', 'drone', 'model.sdf']),
            '-x', '0', '-y', '0', '-z', '0.5'
        ],
        output='screen'
    )
    
    # Spawn person model
    spawn_person = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'person_with_ring',
            '-file', PathJoinSubstitution([pkg_share, 'models', 'person_with_ring', 'model.sdf']),
            '-x', '2', '-y', '0', '-z', '0'
        ],
        output='screen'
    )
    
    # Perception node
    perception_node = Node(
        package='bodyguard_drone',
        executable='perception_node.py',
        name='perception_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Navigation node
    navigation_node = Node(
        package='bodyguard_drone',
        executable='navigation_node.py',
        name='navigation_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # User event node
    user_event_node = Node(
        package='bodyguard_drone',
        executable='user_event_node.py',
        name='user_event_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Voice command node
    voice_cmd_node = Node(
        package='bodyguard_drone',
        executable='voice_cmd_node.py',
        name='voice_cmd_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # TTS node
    tts_node = Node(
        package='bodyguard_drone',
        executable='tts_node.py',
        name='tts_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Drone control node
    drone_control_node = Node(
        package='bodyguard_drone',
        executable='drone_control_node.py',
        name='drone_control_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Command parser node
    command_parser_node = Node(
        package='bodyguard_drone',
        executable='command_parser_node.py',
        name='command_parser_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # RViz for visualization (optional)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution([pkg_share, 'config', 'bodyguard_drone.rviz'])],
        condition=UnlessCondition(headless)
    )
    
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument('use_sim_time', default_value='true',
                            description='Use simulation time'),
        DeclareLaunchArgument('headless', default_value='false',
                            description='Run Gazebo headless (no GUI)'),
        
        # Gazebo
        gazebo_server,
        gazebo_client,  # Gazebo GUI client
        
        # Spawn models
        spawn_drone,
        spawn_person,
        
        # All nodes
        perception_node,
        navigation_node,
        user_event_node,
        voice_cmd_node,
        tts_node,
        drone_control_node,
        command_parser_node,
        
        # Visualization (optional)
        rviz_node,
    ])
