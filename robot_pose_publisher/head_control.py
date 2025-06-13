#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
import time

class HeadOscillator(Node):
    def __init__(self):
        super().__init__('head_oscillator')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 每 0.05 秒执行一次
        self.joint_angle = 0.0
        self.direction = 1

    def timer_callback(self):
        # 创建 JointState 消息
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['joint1']
        msg.position = [self.joint_angle]
        
        # 角度来回摆动
        self.joint_angle += self.direction * 0.05
        if self.joint_angle > math.pi / 3:
            self.direction = -1
        elif self.joint_angle < -math.pi / 3:
            self.direction = 1

        # 发布消息
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = HeadOscillator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
