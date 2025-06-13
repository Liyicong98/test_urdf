#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math
import time

def euler_to_quaternion(roll, pitch, yaw):
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    return qx, qy, qz, qw

class DynamicPosePublisher(Node):
    def __init__(self):
        super().__init__('dynamic_robot_pose_publisher')
        self.br = TransformBroadcaster(self)
        self.timer = self.create_timer(0.05, self.broadcast_pose)  # 20Hz 更新
        self.start_time = time.time()

    def broadcast_pose(self):
        t_now = time.time() - self.start_time

        # 动态变化的角度：做周期性正弦变化（单位：弧度）
        roll = math.radians(30) * math.sin(0.5 * t_now)
        pitch = math.radians(20) * math.sin(0.3 * t_now)
        yaw = math.radians(60) * math.sin(0.2 * t_now)

        qx, qy, qz, qw = euler_to_quaternion(roll, pitch, yaw)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.br.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = DynamicPosePublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
