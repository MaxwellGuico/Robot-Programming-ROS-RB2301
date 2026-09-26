# 1. IMPORT
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

# NODE CLASS
class Sim(Node):

    def __init__(self):
        super().__init__('sim')
        # 2. NODE PROPERTIES
        self.odom_msg = None

        # 3. NODE HANDLES
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_sub_callback, 10)
        self.timer_callback = self.create_timer(0.2, self.callback_print)
    # 4. NODE CALLBACKS
    def odom_sub_callback(self, msg):
        self.odom_msg = msg
        
    def callback_print(self):
        # 5. HOW TO USE
        if self.odom_msg is None:
            return
        
        print(f"x: {self.odom_msg.pose.pose.position.x} y: {self.odom_msg.pose.pose.position.y}")


# MAIN BOILER PLATE
def main(args=None):
    rclpy.init(args=args)
    node = Sim() 
    rclpy.spin(node)
    rclpy.shutdown()
if __name__ == '__main__':
    main()