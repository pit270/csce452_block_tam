import rclpy
from rclpy.node import Node
# from rclpy.parameter import Parameter
from rcl_interfaces.msg import Parameter
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import ParameterValue
from rcl_interfaces.srv import SetParameters

from geometry_msgs.msg import Twist
from turtlesim.msg import Color
from turtlesim.srv import SetPen

class TurtleClient(Node):
    def __init__(self):
        super().__init__('turtle_client')
        self.param_client = self.create_client(SetParameters, 'turtlesim/set_parameters')
        while not self.param_client.wait_for_service(timeout_sec=1.0):
            print("turtle set param service not yet availible...")
        self.param_request = SetParameters.Request()

        self.pen_client = self.create_client(SetPen, '/turtle1/set_pen')
        while not self.pen_client.wait_for_service(timeout_sec=1.0):
            print("turtle pen service not yet availible...")
        self.pen_request = SetPen.Request()
        self.pen_request.width = 2

    def set_color(self, r, g, b):
        self.param_request.parameters = [
            Parameter(name='background_r', value=ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=r)), 
            Parameter(name='background_g', value=ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=g)), 
            Parameter(name='background_b', value=ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=b)),
        ]
        self.future = self.param_client.call_async(self.param_request)
        return self.future.result()

    def set_pen_color(self, r, g, b):
        self.pen_request.r = r
        self.pen_request.g = g
        self.pen_request.b = b
        self.future = self.pen_client.call_async(self.pen_request)
        return self.future.result()

    def set_pen_state(self, isOff: bool):
        self.pen_request.off = 1 if isOff == True else 0
        self.future = self.pen_client.call_async(self.pen_request)
        return self.future.result()

def drawT(vel_pub):
    #TODO: Draw a Block T
    pass #remove this line when you develop this function

def drawA(vel_pub):
    #TODO: Draw a Block A
    #assume starting in bottom left corner facing up
    #move up 
    pass #remove this line when you develop this function

def drawM(vel_pub):
    #TODO: Draw a Block M
    pass #remove this line when you develop this function

def main():
    rclpy.init()
    tam_node = rclpy.create_node('block_tam_pub')
    vel_pub = tam_node.create_publisher(Twist, '/turtle1/cmd_vel', 10)

    #Background to Maroon
    client = TurtleClient()
    client.set_color(80, 0, 0)
    #Set pen color to White
    client.set_pen_color(255, 255, 255)

    # test code. Remove this when writing your code
    vel_msg = Twist()
    vel_msg.linear.x = 1.0
    vel_msg.linear.y = 0.0
    vel_msg.linear.z = 0.0
    vel_msg.angular.y = 0.0
    vel_msg.angular.z = 0.0
    vel_msg.angular.z = 3.1415 / 4
    while(True):
        vel_pub.publish(vel_msg)


    drawT(vel_pub)
    #TODO: transistion from T to A
    drawA(vel_pub)
    #TODO: transistion from A to M
    drawM(vel_pub)
    #TODO: move away from drawing

if __name__ == '__main__':
    main()