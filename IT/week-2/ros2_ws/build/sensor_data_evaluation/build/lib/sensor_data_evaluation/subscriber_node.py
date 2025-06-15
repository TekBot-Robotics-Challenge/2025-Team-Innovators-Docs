import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class SensorSubscriber(Node):
    def __init__(self):
        super().__init__('sensor_subscriber')
        self.subscription = self.create_subscription(String, 'sensor_data', self.listener_callback, 10)

    def listener_callback(self, msg):
        data = json.loads(msg.data)
        temp = data['temperature']
        hum = data['humidity']
        pres = data['pressure']

        if 15 <= temp <= 35 and 30 <= hum <= 70 and 950 <= pres <= 1050:
            self.get_logger().info('Data OK ✅')
        else:
            self.get_logger().warn('Data OUT OF RANGE ⚠️')

def main(args=None):
    rclpy.init(args=args)
    node = SensorSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
