#!/usr/bin/env python3
"""
Safety filter for assisted teleop (ROS2 Humble).

Sits between the keyboard teleop and the robot:

    wasd_teleop  --(cmd_vel_raw)-->  safety_filter  --(cmd_vel)-->  robot

It listens to the LIDAR (/scan). If an obstacle is closer than SAFE_DIST
in the frontal arc AND the user is commanding forward motion, it zeroes the
forward velocity. Turning and reversing are always allowed, so you can steer
out of the obstacle.

Run the teleop with:
    ros2 run <your_pkg> wasd_teleop --ros-args -r cmd_vel:=cmd_vel_raw
Run this node with:
    ros2 run <your_pkg> safety_filter
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

SAFE_DIST = 0.35     # metres - stop forward motion if anything is closer than this
FRONT_ARC = 30       # degrees to each side of straight-ahead to watch


class SafetyFilter(Node):
    def __init__(self):
        super().__init__('safety_filter')
        self.last_cmd = Twist()
        self.obstacle_ahead = False

        self.create_subscription(Twist, 'cmd_vel_raw', self.cmd_cb, 10)
        self.create_subscription(LaserScan, 'scan', self.scan_cb, 10)
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # publish at a steady rate so the robot keeps getting commands
        self.create_timer(0.05, self.publish_cmd)

    def cmd_cb(self, msg):
        self.last_cmd = msg

    def scan_cb(self, msg):
        n = len(msg.ranges)
        if n == 0 or msg.angle_increment == 0.0:
            return

        # number of LIDAR samples covering the frontal arc
        arc = int(math.radians(FRONT_ARC) / msg.angle_increment)
        arc = max(1, min(arc, n // 2))

        # "front" is index 0 on most 360-degree LIDARs, so look at both ends
        front = list(range(0, arc)) + list(range(n - arc, n))

        min_dist = float('inf')
        for i in front:
            r = msg.ranges[i]
            if msg.range_min < r < msg.range_max:   # ignore inf / invalid returns
                min_dist = min(min_dist, r)

        self.obstacle_ahead = min_dist < SAFE_DIST

    def publish_cmd(self):
        out = Twist()
        out.linear.x = self.last_cmd.linear.x
        out.angular.z = self.last_cmd.angular.z

        if self.obstacle_ahead and out.linear.x > 0.0:
            out.linear.x = 0.0   # block forward motion; turning/reverse still allowed
            self.get_logger().warn(
                'Obstacle ahead - forward motion blocked',
                throttle_duration_sec=1.0,
            )

        self.pub.publish(out)


def main():
    rclpy.init()
    node = SafetyFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
