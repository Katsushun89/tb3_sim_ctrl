#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    # Launch arguments
    cost_threshold_arg = DeclareLaunchArgument(
        'cost_threshold',
        default_value='1',
    )

    # Node
    cyclic_goal_navigator_node = Node(
        package='cyclic_goal_navigator',
        executable='cyclic_goal_navigator_node',
        name='cyclic_goal_navigator',
        output='screen',
        parameters=[{
            'cost_threshold': LaunchConfiguration('cost_threshold'),
        }],
        remappings=[
            ('/local_costmap/costmap', '/local_costmap/costmap'),
        ]
    )
    ld.add_action(cost_threshold_arg)
    ld.add_action(cyclic_goal_navigator_node)

    return ld
