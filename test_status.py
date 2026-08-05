import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys

class StatusListener(Node):
    def __init__(self):
        super().__init__('status_listener')
        self.subscription = self.create_subscription(
            String,
            '/kuka_bridge/status',
            self.listener_callback,
            10)
        self.msg_count = 0

    def listener_callback(self, msg):
        print(f"Status: {msg.data}")
        self.msg_count += 1
        if self.msg_count >= 1:
            sys.exit(0)

def main():
    rclpy.init()
    node = StatusListener()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
