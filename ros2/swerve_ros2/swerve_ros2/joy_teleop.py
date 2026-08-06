#!/usr/bin/env python3
"""Gamepad -> /cmd_vel for the swerve base. ROS 2 port of scripts/ps.py."""

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy


class JoyTeleop(Node):

    def __init__(self):
        super().__init__('joy_teleop')

        # Axis and button indices for a PS4 pad under the ROS 2 joy driver.
        self.declare_parameter('axis_linear_x', 1)
        self.declare_parameter('axis_linear_y', 0)
        self.declare_parameter('axis_angular_z', 3)
        self.declare_parameter('button_turbo', 5)
        self.declare_parameter('scale_linear', 2.0)
        self.declare_parameter('scale_angular', 1.5)
        self.declare_parameter('turbo_multiplier', 2.0)
        self.declare_parameter('deadzone', 0.1)

        p = self.get_parameter
        self.axis_x = p('axis_linear_x').value
        self.axis_y = p('axis_linear_y').value
        self.axis_z = p('axis_angular_z').value
        self.button_turbo = p('button_turbo').value
        self.scale_linear = p('scale_linear').value
        self.scale_angular = p('scale_angular').value
        self.turbo_multiplier = p('turbo_multiplier').value
        self.deadzone = p('deadzone').value

        self.pub_move = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Joy, 'joy', self.on_joy, 10)

        self.get_logger().info('joy teleop up, waiting for /joy')

    def _axis(self, axes, index):
        """Read an axis, tolerating pads that report fewer axes than expected."""
        if index < 0 or index >= len(axes):
            return 0.0
        value = axes[index]
        return 0.0 if abs(value) < self.deadzone else value

    def on_joy(self, data):
        turbo = (0 <= self.button_turbo < len(data.buttons)
                 and data.buttons[self.button_turbo] == 1)
        boost = self.turbo_multiplier if turbo else 1.0

        move = Twist()
        move.linear.x = self._axis(data.axes, self.axis_x) * self.scale_linear * boost
        move.linear.y = self._axis(data.axes, self.axis_y) * self.scale_linear * boost
        # The ROS 1 version drove yaw off a button, so rotation was on/off with
        # no proportional control. An axis gives continuous yaw.
        move.angular.z = self._axis(data.axes, self.axis_z) * self.scale_angular * boost

        self.pub_move.publish(move)


def main(args=None):
    rclpy.init(args=args)
    node = JoyTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
