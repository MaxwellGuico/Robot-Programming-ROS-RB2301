# 1. IMPORT
import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters
from rcl_interfaces.msg import ParameterType, Parameter
from rcl_interfaces.srv import SetParameters

# NODE CLASS
class ParameterServices(Node):
    def __init__(self):
        super().__init__('parameter_services')
        # 2. NODE PROPERTIES

        self.get_params_future = None
        self.set_params_future = None

        # 3. NODE HANDLES
        self.set_params_cli = self.create_client(SetParameters, '/turtlesim/set_parameters') # Set parameters
        self.get_params_cli = self.create_client(GetParameters, '/turtlesim/get_parameters') # Get Parameters
        self.timer = self.create_timer(0.5, self.timer_callback) # Call back function for get parameters

    # 4. NODE CALLBACKS
    def timer_callback(self):
        r_value = None
        g_value = None
        if self.get_params_future is None:
            request = GetParameters.Request()
            request.names = ['background_r', 'background_g']
            self.get_params_future = self.get_params_cli.call_async(request)
            print(f'Send Get Param Request')

        elif self.get_params_future.done():
            response = self.get_params_future.result()
            r_value = response.values[0].integer_value
            g_value = response.values[1].integer_value
            print(f'Got Get Param Response: {response}')
            print(f'\tGot background_r: {r_value}') 
            print(f'\tGot background_g: {g_value}') 
            self.get_params_future = None # reset the future so another request can be sent.
        if self.set_params_future is None:
            if r_value is None or g_value is None:
                return
            request = SetParameters.Request()
            new_r = (r_value + 50) % 256
            new_g = (g_value- 40 ) % 256

            new_background_r = Parameter()
            new_background_r.name = 'background_r'
            new_background_r.value.type = ParameterType.PARAMETER_INTEGER # if integer
            new_background_r.value.integer_value = new_r

            new_background_g = Parameter()
            new_background_g.name = 'background_g'
            new_background_g.value.type = ParameterType.PARAMETER_INTEGER # if integer
            new_background_g.value.integer_value = new_g
            request.parameters = [new_background_r, new_background_g]
            self.set_params_future = self.set_params_cli.call_async(request)

            print(f'Setting Red({new_r:3d}) and Green({new_g:3d})')  
        elif self.set_params_future.done():
            response = self.set_params_future.result()
            print(f'Got Set Param Response: {response}')
            self.set_params_future = None # reset service
                


    # 5. HOW TO USE

# MAIN BOILER PLATE
def main(args=None):
    rclpy.init(args=args)
    node = ParameterServices() # same name as the class above.
    rclpy.spin(node)
    rclpy.shutdown()
if __name__ == '__main__':
    main()