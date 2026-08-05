import rclpy
from rclpy.node import Node

rclpy.init()
node = Node('test_node')
nodes = node.get_node_names()
print("Nodes:", nodes)
rclpy.shutdown()
