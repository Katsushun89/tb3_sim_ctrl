#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit, OnExecutionComplete
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # カスタムRViz設定ファイルのパス
    custom_rviz_config = os.path.join(
        get_package_share_directory('tb3_ctrl_bringup'),
        'config',
        'nav2_view.rviz'
    )
    
    # Nav2のlaunchファイルを呼び出し、RViz設定を上書き
    # tb3_simulation_launch.pyは'rviz_config_file'パラメータを受け取る
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch',
                'tb3_simulation_launch.py'
            )
        ),
        launch_arguments={
            'rviz_config_file': custom_rviz_config  # 'rviz_config'から'rviz_config_file'に修正
        }.items()
    )
    
    # 初期位置設定スクリプトのパス
    initial_pose_script = os.path.join(
        get_package_share_directory('tb3_ctrl_bringup'),
        'scripts',
        'set_initial_pose.py'
    )
    
    # Nav2が起動してから5秒後に初期位置を設定
    # TimerActionを使用して遅延実行
    set_initial_pose = TimerAction(
        period=5.0,  # 5秒待機
        actions=[
            ExecuteProcess(
                cmd=['python3', initial_pose_script],
                name='set_initial_pose',
                output='screen'
            )
        ]
    )
    
    return LaunchDescription([
        nav2_launch,
        set_initial_pose  # Nav2起動後に初期位置設定
    ])