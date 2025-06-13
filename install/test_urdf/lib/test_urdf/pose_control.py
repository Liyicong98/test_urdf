#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState
from geometry_msgs.msg import Pose, Point, Quaternion
import math
import time

def euler_to_quaternion(roll, pitch, yaw):
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    return qx, qy, qz, qw

class GazeboPosePublisher(Node):
    def __init__(self):
        super().__init__('gazebo_pose_publisher')
        self.cli = self.create_client(SetEntityState, '/gazebo/set_entity_state')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待 Gazebo 服务启动...')

        self.timer = self.create_timer(0.1, self.send_pose)
        self.start_time = time.time()

    def send_pose(self):
        t = time.time() - self.start_time
        roll = math.radians(20) * math.sin(0.5 * t)
        pitch = math.radians(30) * math.sin(0.4 * t)
        yaw = math.radians(45) * math.sin(0.3 * t)

        qx, qy, qz, qw = euler_to_quaternion(roll, pitch, yaw)

        state = EntityState()
        state.name = 'robot'  # 保持与 spawn_entity.py 的 -entity 一致
        state.pose.position = Point(x=0.0, y=0.0, z=0.5)
        state.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)

        req = SetEntityState.Request()
        req.state = state
        self.cli.call_async(req)

def main(args=None):
    rclpy.init(args=args)
    node = GazeboPosePublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
