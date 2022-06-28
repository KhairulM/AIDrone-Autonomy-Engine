import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy


class MonitorNode(Node):
    def __init__(self):
        super().__init__("monitor_node")


def main(args=None):
    rclpy.init(args=args)

    monitor_node = MonitorNode()
    rclpy.spin(monitor_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
