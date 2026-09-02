# 1. IMPORT
import rclpy
from rclpy.node import Node
from rclpy.logging import LoggingSeverity, set_logger_level

# NODE CLASS
class Logger(Node):
    def __init__(self):
        super().__init__('logger')
        # 2. NODE PROPERTIES
        self.i = 0
        # 3. NODE HANDLES
        self.timer = self.create_timer(0.5, self.timer_callback)

        set_logger_level(self.get_logger().name, LoggingSeverity.WARN)
    # 4. NODE CALLBACKS
    def timer_callback(self):
        self.i += 1 # increments every 0.5s
        
        self.get_logger().debug('this is debug')
        self.get_logger().info(f"{self.i}")
        self.get_logger().warn(f"warn {self.i}")
        self.get_logger().error('an error')
        
    # 5. HOW TO USE

# MAIN BOILER PLATE
def main(args=None):
    rclpy.init(args=args)
    node = Logger() # same name as the class above.
    rclpy.spin(node)
    rclpy.shutdown()
if __name__ == '__main__':
    main()