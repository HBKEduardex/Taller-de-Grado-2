import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

rclpy.init()
node = Node('test_pub')
pub = node.create_publisher(Float64MultiArray, 'test_topic', 10)

arr = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]

try:
    msg1 = Float64MultiArray(data=arr)
    pub.publish(msg1)
    print("Publish with data=arr succeeded")
except Exception as e:
    print(f"Publish with data=arr failed: {e}")

try:
    msg2 = Float64MultiArray()
    msg2.data = [float(x) for x in arr]
    pub.publish(msg2)
    print("Publish with msg2.data succeeded")
except Exception as e:
    print(f"Publish with msg2.data failed: {e}")

rclpy.shutdown()
