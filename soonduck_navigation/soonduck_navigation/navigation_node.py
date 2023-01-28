import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from soonduck_communication import CommunicationNode


class NavigationNode(Node):
    def __init__(self):
        super().__init__("navigation_node")
        self.comm = CommunicationNode()
        self.publisher = self.comm.create_my_publisher(String, "nav")
        self.subscriber = self.comm.create_my_subscriber(String, "nav", self.callback)

    def callback(self, msg):
        self.get_logger().info(msg.data)

    def publish_msg(self, msg):
        self.comm.publish("nav", msg)

    def spin(self):
        self.comm.spin()


def main(args=None):
    rclpy.init(args=args)

    print("Communication node started")

    node = NavigationNode()
    node.publish_msg(String(data="Hello from navigation node"))
    node.spin()


if __name__ == "__main__":
    main()
