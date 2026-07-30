#!/usr/bin/env python3
from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class DebugMonitor(Node):
    def __init__(self) -> None:
        super().__init__('vmc_debug_monitor')
        self.declare_parameter('topic', '/vmc/debug')
        self.declare_parameter('period_s', 0.25)
        self.topic = self.get_parameter('topic').value
        self.period_s = float(self.get_parameter('period_s').value)
        self.last_print_ns = 0
        self.create_subscription(Float64MultiArray, self.topic, self.callback, 10)
        self.get_logger().info(f'Listening to {self.topic}')

    def callback(self, msg: Float64MultiArray) -> None:
        d = msg.data
        if len(d) < 60:
            self.get_logger().warning(f'Expected at least 60 values, got {len(d)}')
            return
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self.last_print_ns < int(self.period_s * 1e9):
            return
        self.last_print_ns = now_ns
        state = 'BALANCE' if d[3] > 0.5 else ('ARMED' if d[0] > 0.5 else 'DISARMED')
        self.get_logger().info(
            f'state={state:8s} pitch={math.degrees(d[4]):+6.2f}deg '
            f'roll={math.degrees(d[51]):+6.2f}/{math.degrees(d[53]):+6.2f}deg '
            f'dLeg={1000*d[56]:+6.1f}mm right_x={d[59]:+.2f} '
            f'qerr_data=see raw x={d[33]:+.4f}m v={d[34]:+.4f}m/s '
            f'B_L=({1000*d[7]:.1f},{1000*d[8]:.1f})mm '
            f'B_R=({1000*d[11]:.1f},{1000*d[12]:.1f})mm '
            f'wheel=({d[43]:+.3f},{d[44]:+.3f})Nm'
        )


def main() -> None:
    rclpy.init()
    node = DebugMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
