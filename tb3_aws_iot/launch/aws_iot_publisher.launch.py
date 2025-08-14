#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'base_frame',
            default_value='base_footprint',
            description='Base frame of the robot'
        ),
        DeclareLaunchArgument(
            'map_frame',
            default_value='map',
            description='Map frame for pose transformation'
        ),
        DeclareLaunchArgument(
            'publish_rate',
            default_value='1.0',
            description='Rate for publishing to AWS IoT (Hz)'
        ),
        DeclareLaunchArgument(
            'aws_iot_endpoint',
            default_value='',
            description='AWS IoT endpoint URL'
        ),
        DeclareLaunchArgument(
            'thing_name',
            default_value='turtlebot3',
            description='AWS IoT thing name'
        ),
        DeclareLaunchArgument(
            'output_file',
            default_value='/tmp/aws_iot_data.json',
            description='Output file path for JSON data'
        ),

        # AWS IoT Publisher Node
        Node(
            package='tb3_aws_iot',
            executable='aws_iot_publisher_node',
            name='aws_iot_publisher',
            output='screen',
            parameters=[{
                'base_frame': LaunchConfiguration('base_frame'),
                'map_frame': LaunchConfiguration('map_frame'),
                'publish_rate': LaunchConfiguration('publish_rate'),
                'aws_iot_endpoint': LaunchConfiguration('aws_iot_endpoint'),
                'thing_name': LaunchConfiguration('thing_name'),
                'output_file': LaunchConfiguration('output_file'),
            }],
            remappings=[
                ('/odom', '/odom'),
                ('/goal_markers', '/goal_markers'),
            ]
        ),
    ])