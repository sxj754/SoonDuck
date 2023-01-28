import rclpy
from rclpy.node import Node


class CommunicationNode(Node):
    def __init__(self):
        super().__init__("communication_node")
        self.publishers_ = {}
        self.subscribers_ = {}

    def create_my_publisher(self, msg_type, topic_name, qos_history_depth=10):
        self.publishers_[topic_name] = self.create_publisher(
            msg_type, topic_name, qos_history_depth
        )
        return self.publishers_[topic_name]

    def create_my_subscriber(
        self, msg_type, topic_name, callback, qos_history_depth=10
    ):
        self.subscribers_[topic_name] = self.create_subscription(
            msg_type, topic_name, callback, qos_history_depth
        )
        return self.subscribers_[topic_name]

    def publish(self, topic_name, msg):
        self.publishers_[topic_name].publish(msg)

    def spin(self):
        rclpy.spin(self)


def main(args=None):
    rclpy.init(args=args)

    print("Communication node started")

    comm = CommunicationNode()

    rclpy.spin(comm)

    comm.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
