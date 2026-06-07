#!/usr/bin/env python3
"""
Map kickstart node.

At startup the robot is stationary, so the LIDAR only sees a thin slice of the
world and SLAM has almost no map -- which means frontier exploration finds
nothing and quits. This node fixes that automatically (no human driving):
it spins the robot slowly in place for a few seconds so SLAM builds a full
360-degree local map, then stops and exits, handing control to explore_lite.

Run this ONCE, right before explore_lite. It self-terminates when done.

    ros2 run my_bot map_kickstart
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


SPIN_SPEED = 0.5      # rad/s  (gentle, so software-rendered Gazebo keeps up)
SPIN_SECONDS = 14.0   # ~one full slow rotation; long enough to map all around
PUBLISH_HZ = 10.0


class MapKickstart(Node):
    def __init__(self):
        super().__init__('map_kickstart')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.ticks = 0
        self.max_ticks = int(SPIN_SECONDS * PUBLISH_HZ)
        self.timer = self.create_timer(1.0 / PUBLISH_HZ, self.tick)
        self.get_logger().info(
            f'Kickstart: spinning in place for {SPIN_SECONDS:.0f}s to seed the map...'
        )

    def tick(self):
        twist = Twist()
        if self.ticks < self.max_ticks:
            twist.angular.z = SPIN_SPEED
            self.pub.publish(twist)
            self.ticks += 1
        else:
            # Stop the robot and finish
            self.pub.publish(Twist())
            self.get_logger().info('Kickstart complete - map seeded. Handing over to explore.')
            self.timer.cancel()
            rclpy.shutdown()


def main():
    rclpy.init()
    node = MapKickstart()
    try:
        rclpy.spin(node)
    except Exception:
        pass


if __name__ == '__main__':
    main()
