import rclpy
from rclpy.node import Node


class FleetManagerNode(Node):

    def __init__(self):
        super().__init__('fleet_manager_node')
        self.timer = self.create_timer(1.0, self.on_timer)

    def on_timer(self):
        self.get_logger().info('fleet_manager alive')


def main(args=None):
    rclpy.init(args=args)
    node = FleetManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
