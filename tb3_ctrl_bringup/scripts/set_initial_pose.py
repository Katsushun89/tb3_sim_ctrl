#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import time
import sys

class InitialPoseSetter(Node):
    def __init__(self):
        super().__init__('initial_pose_setter')
        
        # パラメータ（デフォルトは手動設定値）
        self.declare_parameter('initial_x', -1.7018039226531982)
        self.declare_parameter('initial_y', -0.39197060465812683)
        self.declare_parameter('initial_yaw', -0.030202)  # orientation から計算した値
        
        self.initial_x = self.get_parameter('initial_x').value
        self.initial_y = self.get_parameter('initial_y').value
        self.initial_yaw = self.get_parameter('initial_yaw').value
        
        # Publisher for initial pose
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )
        
        self.get_logger().info('Initial pose setter started')
        
        # Wait for publisher to be ready
        time.sleep(2.0)
        
        # Set initial pose
        self.publish_initial_pose()
    
    def publish_initial_pose(self):
        """初期位置をパブリッシュ"""
        import math
        
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # Position
        msg.pose.pose.position.x = self.initial_x
        msg.pose.pose.position.y = self.initial_y
        msg.pose.pose.position.z = 0.0
        
        # Orientation (yaw to quaternion)
        # 手動設定値と同じクォータニオン値を使用
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = -0.015101008938464956
        msg.pose.pose.orientation.w = 0.9998859732634718
        
        # Covariance (初期不確実性)
        msg.pose.covariance[0] = 0.25  # x
        msg.pose.covariance[7] = 0.25  # y
        msg.pose.covariance[35] = 0.06853891909122467  # yaw
        
        # Publish multiple times to ensure reception
        for i in range(3):
            self.initial_pose_pub.publish(msg)
            self.get_logger().info(f'Initial pose published ({i+1}/3): x={self.initial_x:.3f}, y={self.initial_y:.3f}')
            time.sleep(0.2)
        
        self.get_logger().info('Initial pose setting completed')

def main(args=None):
    rclpy.init(args=args)
    
    node = InitialPoseSetter()
    
    # ノードを短時間実行して終了
    rclpy.spin_once(node, timeout_sec=1.0)
    
    node.destroy_node()
    rclpy.shutdown()
    
    return 0

if __name__ == '__main__':
    sys.exit(main())