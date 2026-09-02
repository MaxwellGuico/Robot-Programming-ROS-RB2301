# 1. IMPORT
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn
# NODE CLASS
class Tutorial(Node):
    def __init__(self):
        super().__init__('tutorial')
        # 2. NODE PROPERTIES
        self.counter = 0
        self.current_pose = None

        self.pub_msg = Twist()
        self.pub_msg.linear.x = 1.00

        self.srv_requested = False
        self.future = None

        # 3. NODE HANDLES
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.sub = self.create_subscription(Pose, '/turtle1/pose', self.sub_callback, 10)
        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel',10)
        self.srv_client = self.create_client(Spawn, '/spawn')
        

    # 4. NODE CALLBACKS
    def timer_callback(self):
        self.counter += 1
        print(f"Counter: {self.counter}")
        if self.current_pose is not None:
            print(f'({self.current_pose.x}, {self.current_pose.y})')

        # Publisher
        self.pub.publish(self.pub_msg)
        self.pub_msg.linear.x *= -1.00 # Alternates between 1 and -1

        if not self.srv_requested:
            self.srv_request()
            self.srv_requested = True

        if self.future is not None and self.future.done(): # Check if future is not empty and srv request is done
            self.response = self.future.result()
            print(f"New Turtle is called {self.response.name}")
            self.future = None

    # Subscriber
    def sub_callback(self,msg:Pose):
        self.current_pose = msg

    # Service Client
    def srv_request(self):
        self.srv_msg = Spawn.Request()
        self.srv_msg.x = 1.00
        self.srv_msg.y = 1.00
        self.srv_msg.name = ''
        self.future = self.srv_client.call_async(self.srv_msg)
    # 5. HOW TO USE


# MAIN BOILER PLATE
def main(args=None):
    rclpy.init(args=args)
    node = Tutorial() # same name as the class above.
    rclpy.spin(node)
    rclpy.shutdown()
if __name__ == '__main__':
    main()