import rclpy
from rclpy.node import Node
import sensor_msgs.msg as sensor_msgs

class DiagnosticRosNode(Node):

    def __init__(self):
        super.__init__("DiagnosticNode")
        self.pcd_subscriber = self.create_subscription(
            sensor_msgs.PointCloud2,    # Msg type
            'pcd',                      # topic
            self.listener_callback,      # Function to call
            10                          # QoS
        )
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)


def __main__():
    print("Wowza")