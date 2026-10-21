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
        if len(d) < 93:
            self.get_logger().warning(f'Expected at least 93 values, got {len(d)}')
            return
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self.last_print_ns < int(self.period_s * 1e9):
            return
        self.last_print_ns = now_ns
        recovery_active = len(d) >= 140 and d[93] > 0.5
        state = (
            'RECOVERY' if recovery_active else
            ('BALANCE' if d[3] > 0.5 else ('ARMED' if d[0] > 0.5 else 'DISARMED'))
        )
        recovery_text = ''
        if recovery_active:
            recovery_text = (
                f' rec_align={math.degrees(d[104]):+5.1f}deg '
                f'rec_height={1000*d[98]:5.1f}/{1000*d[100]:5.1f}mm '
                f'unlock={int(d[94] > 0.5)} '
                f'rec_action=({d[108]:+.2f},{d[109]:+.2f},{d[110]:+.2f}) '
                f'rec_tau={d[107]:+.3f}Nm rec_infer={d[111]:.1f}us'
            )
        self.get_logger().info(
            f'state={state:8s} pitch={math.degrees(d[4]):+6.2f}deg '
            f'roll={math.degrees(d[51]):+6.2f}/{math.degrees(d[53]):+6.2f}deg '
            f'dLeg={1000*d[56]:+6.1f}mm right_x={d[59]:+.2f} '
            f'height={1000*d[64]:5.1f}/{1000*d[62]:5.1f}mm left_y={d[60]:+.2f} '
            f'qerr_data=see raw x={d[33]:+.4f}m v={d[34]:+.4f}m/s '
            f'B_L=({1000*d[7]:.1f},{1000*d[8]:.1f})mm '
            f'B_R=({1000*d[11]:.1f},{1000*d[12]:.1f})mm '
            f'policy={d[69]:+.3f} residual={d[70]:+.3f}/{d[71]:+.3f}Nm '
            f'infer={d[73]:.1f}us gravity=({d[86]:+.3f},{d[87]:+.3f},{d[88]:+.3f}) '
            f'g_ready={int(d[90] > 0.5)} wheel=({d[43]:+.3f},{d[44]:+.3f})Nm'
            f'{recovery_text}'
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
