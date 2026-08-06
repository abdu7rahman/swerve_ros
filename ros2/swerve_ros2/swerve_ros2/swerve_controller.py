#!/usr/bin/env python3
"""Swerve drive inverse kinematics: /cmd_vel -> per-module speed and angle.

ROS 2 port of scripts/swerve_controller.py.

Module numbering matches the ROS 1 version:

      front
    1 ------ 2        +x forward, +y left, +z yaw (REP-103)
    |        |
    4 ------ 3
      rear

For a chassis twist (vx, vy, omega), the velocity at a module mounted at body
offset r is  v_module = v_chassis + omega x r.  With r = (rx, ry, 0) that is

    vx_m = vx - omega * ry
    vy_m = vy + omega * rx

which gives the module's ground speed as |v_module| and its steer angle as
atan2(vy_m, vx_m).
"""

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray


class SwerveController(Node):

    def __init__(self):
        super().__init__('swerve_controller')

        # Wheelbase (front-to-rear) and track (left-to-right), in metres.
        # The ROS 1 version used l = w = 450 with no unit conversion, so the
        # rotation term swamped the translation term by three orders of
        # magnitude against an m/s Twist. These are metres.
        self.declare_parameter('wheelbase', 0.45)
        self.declare_parameter('track', 0.45)
        self.declare_parameter('max_module_speed', 1.0)
        self.declare_parameter('publish_rate', 20.0)

        self.wheelbase = self.get_parameter('wheelbase').value
        self.track = self.get_parameter('track').value
        self.max_module_speed = self.get_parameter('max_module_speed').value

        half_l = self.wheelbase / 2.0
        half_w = self.track / 2.0

        # Module mounting offsets (rx, ry) in the base frame, front-left first,
        # then clockwise, matching the 1-2-3-4 numbering above.
        self.module_offsets = [
            (+half_l, +half_w),  # 1 front-left
            (+half_l, -half_w),  # 2 front-right
            (-half_l, -half_w),  # 3 rear-right
            (-half_l, +half_w),  # 4 rear-left
        ]

        self.create_subscription(Twist, 'cmd_vel', self.on_cmd_vel, 10)

        # Flat [speed1, angle1, ... speed4, angle4], consumed by the drive board.
        # Angles are degrees here to stay compatible with the firmware.
        self.final_vel_pub = self.create_publisher(
            Float32MultiArray, 'finalvel', 10)

        self.last_twist = Twist()

        # The ROS 1 node created a Publisher inside the subscriber callback on
        # every message and then spun on a rate loop that never actually
        # throttled anything. Publishing on a timer instead gives a steady rate
        # and keeps the last command latched if /cmd_vel goes quiet.
        period = 1.0 / self.get_parameter('publish_rate').value
        self.create_timer(period, self.publish_modules)

        self.get_logger().info(
            f'swerve controller up: wheelbase={self.wheelbase} m '
            f'track={self.track} m')

    def on_cmd_vel(self, msg):
        self.last_twist = msg

    def compute_modules(self, vx, vy, omega):
        """Return [(speed, angle_deg), ...] for the four modules."""
        modules = []
        for rx, ry in self.module_offsets:
            vx_m = vx - omega * ry
            vy_m = vy + omega * rx

            speed = math.hypot(vx_m, vy_m)
            # atan2(y, x) — the ROS 1 code passed the arguments the other way
            # round, which measured the angle off the +y axis instead of +x.
            angle = math.degrees(math.atan2(vy_m, vx_m))
            modules.append((speed, angle))

        # Desaturate: if any module is commanded past what it can do, scale the
        # whole set down so the chassis still moves in the requested direction.
        fastest = max(speed for speed, _ in modules)
        if self.max_module_speed > 0.0 and fastest > self.max_module_speed:
            scale = self.max_module_speed / fastest
            modules = [(speed * scale, angle) for speed, angle in modules]

        return modules

    def publish_modules(self):
        vx = self.last_twist.linear.x
        vy = self.last_twist.linear.y
        omega = self.last_twist.angular.z

        modules = self.compute_modules(vx, vy, omega)

        msg = Float32MultiArray()
        # Eight entries, in order. The ROS 1 node packed only six and repeated
        # angle4 where angle3 belonged, so the subscriber board read past the
        # end of the array for modules 3 and 4.
        msg.data = [value for pair in modules for value in pair]
        self.final_vel_pub.publish(msg)

        self.get_logger().debug(' '.join(
            f'm{i + 1}=({s:.2f} m/s, {a:.1f} deg)'
            for i, (s, a) in enumerate(modules)))


def main(args=None):
    rclpy.init(args=args)
    node = SwerveController()
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
