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

from time import sleep
from copy import deepcopy

PI = 3.141592654

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

VELOCITY = 1.0 # ??? / sec
ANG_VEL = PI / 2
UPDATE_RATE = 20 #HZ
PERIOD = 1 / UPDATE_RATE

MOVE_MSG = Twist()
MOVE_MSG.linear.x = VELOCITY
MOVE_MSG.linear.y = 0.0
MOVE_MSG.linear.z = 0.0
MOVE_MSG.angular.y = 0.0
MOVE_MSG.angular.z = 0.0
MOVE_MSG.angular.z = 0.0

TURN_MSG = Twist()
TURN_MSG.linear.x = 0.0
TURN_MSG.linear.y = 0.0
TURN_MSG.linear.z = 0.0
TURN_MSG.angular.y = 0.0
TURN_MSG.angular.z = 0.0
TURN_MSG.angular.z = ANG_VEL

STOP_MSG = Twist()
STOP_MSG.linear.x = 0.0
STOP_MSG.linear.y = 0.0
STOP_MSG.linear.z = 0.0
STOP_MSG.angular.y = 0.0
STOP_MSG.angular.z = 0.0
STOP_MSG.angular.z = 0.0

def moveDistance(vel_pub, dist):
    duration = abs(dist / VELOCITY)
    time_elapsed = 0
    while time_elapsed < duration:
        print(duration, time_elapsed)
        vel_pub.publish(MOVE_MSG)
        sleep(PERIOD)
        time_elapsed += PERIOD
    vel_pub.publish(STOP_MSG)
    sleep(PERIOD)

def turnAngle(vel_pub, angle):
    turn_msg = deepcopy(TURN_MSG)
    if angle < 0:
        turn_msg.angular.z *= -1
    
    duration = abs(angle / ANG_VEL)
    time_elapsed = 0
    while time_elapsed < duration:
        vel_pub.publish(turn_msg)
        sleep(PERIOD)
        time_elapsed += PERIOD
    vel_pub.publish(STOP_MSG)
    sleep(PERIOD)

def drawT(vel_pub):
    #TODO: Draw a Block T
    pass #remove this line when you develop this function

def drawA(vel_pub):
    #TODO: Draw a Block A
    #assume starting in bottom left corner facing up
    moveDistance(vel_pub, 0.562)
    turnAngle(vel_pub, -PI/2)
    moveDistance(vel_pub, 0.3011)
    turnAngle(vel_pub, 1.169)
    moveDistance(vel_pub, 1.8266)
    turnAngle(vel_pub, 1.973)
    
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
    # vel_msg = Twist()
    # vel_msg.linear.x = VELOCITY
    # vel_msg.linear.y = 0.0
    # vel_msg.linear.z = 0.0
    # vel_msg.angular.y = 0.0
    # vel_msg.angular.z = 0.0
    # vel_msg.angular.z = 0.0
    # t = 0
    # while(t < 1):
    #     vel_pub.publish(vel_msg)
    #     t += PERIOD
    #     sleep(PERIOD)


    drawT(vel_pub)
    #TODO: transistion from T to A
    drawA(vel_pub)
    #TODO: transistion from A to M
    drawM(vel_pub)
    #TODO: move away from drawing

if __name__ == '__main__':
    main()