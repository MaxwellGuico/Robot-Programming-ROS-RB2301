# 1. IMPORT
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose

# NODE CLASS
class Recorder(Node):
    def __init__(self):
        super().__init__('recorder')
        # 2. NODE PROPERTIES
        self.recorded_text = open('data.txt', 'w')
        self.recorded_text.write(f'x\ty\ttheta\n') # first row of headers
        # 3. NODE HANDLES
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.subscriber = self.create_subscription(Pose,'/turtle1/pose',self.sub_callback,10)
        
    # 4. NODE CALLBACKS
    def timer_callback(self): # not sure what the timer callback does
        pass

    def sub_callback(self,msg:Pose):
        self.recorded_text.write(f'{msg.x}\t{msg.y}\t{msg.theta}\n')
        
    # 5. HOW TO USE

# MAIN BOILER PLATE
def main(args=None):
    rclpy.init(args=args)
    node = Recorder() # same name as the class above.
    rclpy.spin(node)
    rclpy.shutdown()
if __name__ == '__main__':
    main()