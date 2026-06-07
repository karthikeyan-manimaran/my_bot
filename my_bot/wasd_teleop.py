#!/usr/bin/env python3
"""
WASD / Arrow-key teleop for ROS2 Humble.

Publishes geometry_msgs/Twist on the 'cmd_vel' topic.
By default it sends commands to 'cmd_vel'. When you use the safety filter
in Phase 1, remap this to 'cmd_vel_raw' at launch:

    ros2 run <your_pkg> wasd_teleop --ros-args -r cmd_vel:=cmd_vel_raw

Controls (press a key to set motion; it keeps going until you change it):
    w / Up arrow    : forward
    s / Down arrow  : backward
    a / Left arrow  : turn left
    d / Right arrow : turn right
    space           : stop
    q               : quit
"""

import sys
import termios
import tty
import select

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

LIN_VEL = 0.22   # m/s   (TurtleBot3 burger max is ~0.22)
ANG_VEL = 1.0    # rad/s

BANNER = """
------------------------------------------
   WASD / Arrow-key Teleop
------------------------------------------
   w / Up     : forward
   s / Down   : backward
   a / Left   : turn left
   d / Right  : turn right
   space      : stop
   q          : quit
------------------------------------------
"""


def get_key(settings, timeout=0.1):
    """Read a single keypress (or arrow-key escape sequence) without Enter."""
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    if rlist:
        key = sys.stdin.read(1)
        if key == '\x1b':                 # start of an arrow-key escape sequence
            key += sys.stdin.read(2)      # e.g. '\x1b[A' for the up arrow
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


class WasdTeleop(Node):
    def __init__(self):
        super().__init__('wasd_teleop')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)


def main():
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    node = WasdTeleop()
    print(BANNER)

    lin, ang = 0.0, 0.0
    try:
        while rclpy.ok():
            key = get_key(settings)

            if key in ('w', '\x1b[A'):
                lin, ang = LIN_VEL, 0.0
            elif key in ('s', '\x1b[B'):
                lin, ang = -LIN_VEL, 0.0
            elif key in ('a', '\x1b[D'):
                lin, ang = 0.0, ANG_VEL
            elif key in ('d', '\x1b[C'):
                lin, ang = 0.0, -ANG_VEL
            elif key == ' ':
                lin, ang = 0.0, 0.0
            elif key == 'q':
                break
            # empty key -> keep the previous command (robot keeps moving)

            twist = Twist()
            twist.linear.x = lin
            twist.angular.z = ang
            node.pub.publish(twist)

    except Exception as exc:
        print(exc)
    finally:
        node.pub.publish(Twist())   # stop the robot on exit
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
