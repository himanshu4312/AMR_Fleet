"""
Fail-safe watchdog for fleet_manager_node.

fleet_manager_node cannot detect its own death - if that process crashes
while a robot is paused (speed_limit published at 1%, see
fleet_manager_node.PAUSE_SPEED_LIMIT_PERCENT), nothing else in the system
will ever clear it, since controller_server just holds onto the last
SpeedLimit value it received indefinitely. This node exists purely to
prevent that: it watches a heartbeat fleet_manager_node publishes every
tick, and if that heartbeat goes stale, force-publishes RESUME to every
robot's speed_limit topic and keeps doing so (loudly, via ERROR logs) until
the heartbeat returns. It deliberately does nothing else - the fewer
responsibilities this node has, the less likely it is to have its own bugs
undermining the one guarantee it exists to provide.
"""

import rclpy
from rclpy.node import Node

from nav2_msgs.msg import SpeedLimit
from std_msgs.msg import Empty

from amr_fleet_manager.fleet_manager_node import (
    DEFAULT_ROBOT_NAMES,
    HEARTBEAT_TOPIC,
    RESUME_SPEED_LIMIT_PERCENT,
)

DEFAULT_HEARTBEAT_TIMEOUT_S = 2.0  # a few missed 0.5s ticks' worth of grace
CHECK_PERIOD_S = 0.5


class FleetWatchdogNode(Node):

    def __init__(self):
        super().__init__('fleet_watchdog_node')

        self.declare_parameter('robot_names', DEFAULT_ROBOT_NAMES)
        self.declare_parameter('heartbeat_timeout_s', DEFAULT_HEARTBEAT_TIMEOUT_S)
        self.robot_names = list(self.get_parameter('robot_names').value)
        self.heartbeat_timeout_s = float(self.get_parameter('heartbeat_timeout_s').value)

        self.speed_limit_pubs = {
            name: self.create_publisher(SpeedLimit, f'/{name}/speed_limit', 10)
            for name in self.robot_names
        }

        # Grace period before the very first check: give fleet_manager_node
        # a chance to have started and published at least one heartbeat.
        self.last_heartbeat_at = self.get_clock().now()
        self.tripped = False

        self.create_subscription(Empty, HEARTBEAT_TOPIC, self._on_heartbeat, 10)
        self.create_timer(CHECK_PERIOD_S, self._check)

        self.get_logger().info(
            f'fleet_watchdog watching for {HEARTBEAT_TOPIC} '
            f'(timeout={self.heartbeat_timeout_s}s) covering {self.robot_names}')

    def _on_heartbeat(self, _msg: Empty):
        self.last_heartbeat_at = self.get_clock().now()
        if self.tripped:
            self.tripped = False
            self.get_logger().info(
                'fleet_manager heartbeat resumed - handing control back to it.')

    def _check(self):
        age_s = (self.get_clock().now() - self.last_heartbeat_at).nanoseconds / 1e9
        if age_s > self.heartbeat_timeout_s:
            if not self.tripped:
                self.tripped = True
                self.get_logger().error(
                    f'fleet_manager heartbeat lost for {age_s:.1f}s '
                    f'(timeout {self.heartbeat_timeout_s}s) - forcing RESUME on all robots '
                    f'as a fail-safe. Fleet conflict coordination is OFFLINE until '
                    f'fleet_manager_node is restarted.')
            # Keep re-publishing every tick while tripped, same reasoning as
            # fleet_manager_node's own "republish every tick" pattern: a
            # controller_server that only just (re)connected still needs to
            # receive the clear.
            self._publish_resume_all()

    def _publish_resume_all(self):
        for name in self.robot_names:
            msg = SpeedLimit()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'map'
            msg.percentage = True
            msg.speed_limit = RESUME_SPEED_LIMIT_PERCENT
            self.speed_limit_pubs[name].publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FleetWatchdogNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
