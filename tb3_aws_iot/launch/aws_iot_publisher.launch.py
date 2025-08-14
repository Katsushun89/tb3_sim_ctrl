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
        DeclareLaunchArgument(
            'cert_path',
            default_value='/home/s-katsu/.aws/tb3_aws_iot/certs/bf4e3ef806d187dfe9cc94a68290ae74a50d2083bd64665e5713bd844b9a03e3-certificate.pem.crt',
            description='Path to device certificate'
        ),
        DeclareLaunchArgument(
            'private_key_path',
            default_value='/home/s-katsu/.aws/tb3_aws_iot/certs/bf4e3ef806d187dfe9cc94a68290ae74a50d2083bd64665e5713bd844b9a03e3-private.pem.key',
            description='Path to private key'
        ),
        DeclareLaunchArgument(
            'ca_cert_path',
            default_value='/home/s-katsu/.aws/tb3_aws_iot/certs/AmazonRootCA1.pem',
            description='Path to CA certificate'
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
                'cert_path': LaunchConfiguration('cert_path'),
                'private_key_path': LaunchConfiguration('private_key_path'),
                'ca_cert_path': LaunchConfiguration('ca_cert_path'),
            }],
            remappings=[
                ('/odom', '/odom'),
                ('/goal_markers', '/goal_markers'),
            ]
        ),
    ])