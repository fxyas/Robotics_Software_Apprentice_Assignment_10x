#!/usr/bin/env python3
"""
Launch file for DWA planner node and RViz2.
Author: Fayas
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Generate launch description for DWA planner and RViz2."""
    
    # Get package directory
    pkg_dir = get_package_share_directory('custom_dwa_planner')
    
    # Path to config files
    config_file = os.path.join(pkg_dir, 'config', 'dwa_params.yaml')
    rviz_config_file = os.path.join(pkg_dir, 'config', 'dwa_rviz.rviz')
    
    # Declare launch arguments
    goal_x_arg = DeclareLaunchArgument(
        'goal_x',
        default_value='5.0',
        description='Goal X coordinate'
    )
    
    goal_y_arg = DeclareLaunchArgument(
        'goal_y',
        default_value='5.0',
        description='Goal Y coordinate'
    )
    
    # DWA planner node
    dwa_planner_node = Node(
        package='custom_dwa_planner',
        executable='dwa_planner_node.py',
        name='dwa_planner_node',
        output='screen',
        parameters=[
            config_file,
            {
                'goal_x': LaunchConfiguration('goal_x'),
                'goal_y': LaunchConfiguration('goal_y'),
            }
        ]
    )
    
    # RViz2 node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )
    
    return LaunchDescription([
        goal_x_arg,
        goal_y_arg,
        dwa_planner_node,
        rviz_node
    ])
