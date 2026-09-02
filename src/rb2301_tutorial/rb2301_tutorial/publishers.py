import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from rclpy.qos import (
    QoSProfile,
    HistoryPolicy,
    DurabilityPolicy,
    qos_profile_sensor_data
)
class PublishersNode(Node):

    sensor_pub = None
    latch_pub = None

    def __init__(self):
        super().__init__('publishers')

        # QoS
        qos_profile_latch = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.i = 0

        #Handlers
        self.sensor_pub = self.create_publisher(Header, '/sensor', qos_profile_sensor_data)
        self.latch_pub = self.create_publisher(Header, '/latch', qos_profile_latch)
        self.timer = self.create_timer(0.05, self.timer_callback)
    def timer_callback(self):
        self.i += 1
        msg = Header()
        msg.stamp = self.get_clock().now().to_msg()
        msg.frame_id = str(self.i)

        print(f'Publish: {self.i}')
        
        if self.sensor_pub is not None:
            self.sensor_pub.publish(msg)

        if self.latch_pub is not None:
            self.latch_pub.publish(msg)
    
def main(args=None):
    rclpy.init(args=args)
    node = PublishersNode()
    rclpy.spin(node) 
    rclpy.shutdown()
if __name__ == '__main__':
    main()