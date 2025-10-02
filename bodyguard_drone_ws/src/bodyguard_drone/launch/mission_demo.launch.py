#!/usr/bin/env python3
"""
Mission Demo Launch File - Headless mission execution
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get package directory
    pkg_share = FindPackageShare('bodyguard_drone')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Paths to world and models
    world_file = PathJoinSubstitution([pkg_share, 'worlds', 'bodyguard_world.sdf'])
    models_path = PathJoinSubstitution([pkg_share, 'models'])
    
    # Gazebo server (headless - no GUI)
    gazebo_server = ExecuteProcess(
        cmd=['gzserver', world_file, '-s', 'libgazebo_ros_init.so', 
             '-s', 'libgazebo_ros_factory.so'],
        output='screen',
        additional_env={'GAZEBO_MODEL_PATH': models_path}
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
    
    # Mission controller node (main mission executor)
    mission_controller_node = Node(
        package='bodyguard_drone',
        executable='mission_controller_node.py',
        name='mission_controller_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument('use_sim_time', default_value='true',
                            description='Use simulation time'),
        
        # Gazebo (headless)
        gazebo_server,
        
        # Spawn models
        spawn_drone,
        spawn_person,
        
        # Essential nodes for mission
        perception_node,
        navigation_node,
        tts_node,
        drone_control_node,
        mission_controller_node,
    ])
