# 1. IMPORT
import rclpy
from rclpy.node import Node
from rclpy import Parameter as Param
from rcl_interfaces.msg import SetParametersResult
# NODE CLASS
class Parameters(Node):
    def __init__(self):
        super().__init__('parameters')
        # 2. NODE PROPERTIES
        self.declare_parameter('values.bool_v',value=False)
        self.declare_parameter('values.int_v', value=0)
        self.declare_parameter('values.dbl_v',value=0.0)
        self.declare_parameter('values.str_v',value='')


        self.declare_parameter('bool_arr_v', value=[False])
        self.declare_parameter('int_arr_v', value=[0])
        self.declare_parameter('dbl_arr_v', value=[0.0])
        self.declare_parameter('str_arr_v',value=[''])

        # 3. NODE HANDLES
        self.add_on_set_parameters_callback(self.set_parameters_callback)
        self.timer = self.create_timer(1.0, self.timer_callback)

    # 4. NODE CALLBACKS
    def set_parameters_callback(self, parameter_list):
        for parameter in parameter_list:
            print(f'Setting "{parameter.name}": {parameter.value}')
        return SetParametersResult(successful=True)
    def timer_callback(self):
        str_v_value = self.get_parameter('values.str_v').value
        self.get_logger().info(f'{str_v_value}')
    # 5. HOW TO USE 

# MAIN BOILER PLATE
def main(args=None):
    rclpy.init(args=args)
    node = Parameters()
    rclpy.spin(node)
    rclpy.shutdown()
if __name__ == '__main__':
    main()