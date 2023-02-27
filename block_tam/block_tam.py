import rclpy
from rclpy.node import Node
# from rclpy.parameter import Parameter
from rcl_interfaces.msg import Parameter
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import ParameterValue
from rcl_interfaces.srv import SetParameters

from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
from turtlesim.msg import Color
from turtlesim.srv import SetPen

from time import sleep
from copy import deepcopy

PI = 3.14159265359

class TurtleClient(Node):
    def __init__(self):
        super().__init__('turtle_client')

        #Defining clients for the services
        self.clear_client = self.create_client(Empty, '/clear')
        while not self.clear_client.wait_for_service(timeout_sec=1.0):
            print("turtle clear param service not yet availible...")
        self.clear_request = Empty.Request()

        self.param_client = self.create_client(SetParameters, '/turtlesim/set_parameters')
        while not self.param_client.wait_for_service(timeout_sec=1.0):
            print("turtle set param service not yet availible...")
        self.param_request = SetParameters.Request()

        self.pen_client = self.create_client(SetPen, '/turtle1/set_pen')
        while not self.pen_client.wait_for_service(timeout_sec=1.0):
            print("turtle pen service not yet availible...")
        self.pen_request = SetPen.Request()
        self.pen_request.width = 2

    def make_clear(self):
        self.future = self.clear_client.call_async(self.clear_request)
        return self.future.result()

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

VELOCITY = 0.5 # ??? / sec
ANG_VEL = PI / 4
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

def drawA(vel_pub, client):
    #TODO: Draw a Block A
    #assume starting in bottom left corner facing up

    # define common angles for ease of use
    rightAngle = PI/2
    # angle1 = 1.16868231114 # + 0.03490658504
    # angle2 = PI-angle1
    angle1 = 1.15898902143 
    angle2 = PI-angle1

    moveDistance(vel_pub, 0.562)
    turnAngle(vel_pub, -rightAngle)
    moveDistance(vel_pub, 0.3011)
    turnAngle(vel_pub, angle1)
    moveDistance(vel_pub, 1.8266)
    turnAngle(vel_pub, angle2-0.01)
    moveDistance(vel_pub, 0.1606)
    turnAngle(vel_pub, -rightAngle)
    moveDistance(vel_pub, 0.562)
    turnAngle(vel_pub, -rightAngle)
    moveDistance(vel_pub, 1.2044)
    turnAngle(vel_pub, -rightAngle)
    moveDistance(vel_pub, 0.562)
    turnAngle(vel_pub, -rightAngle)
    moveDistance(vel_pub, 0.1606)
    turnAngle(vel_pub, angle2-0.01)
    moveDistance(vel_pub, 1.8266)
    turnAngle(vel_pub, angle1)
    moveDistance(vel_pub, 0.3011)
    turnAngle(vel_pub, -rightAngle)
    moveDistance(vel_pub, 0.562)
    turnAngle(vel_pub, -rightAngle)
    moveDistance(vel_pub, 1.0839)
    turnAngle(vel_pub, -rightAngle)
    moveDistance(vel_pub, 0.562)
    turnAngle(vel_pub, -rightAngle)
    moveDistance(vel_pub, 0.1606)
    turnAngle(vel_pub, angle2)
    moveDistance(vel_pub, 0.3252)
    turnAngle(vel_pub, angle1-0.08)
    moveDistance(vel_pub, 0.9033)
    turnAngle(vel_pub, angle1)
    moveDistance(vel_pub, 0.3252)
    turnAngle(vel_pub, angle2-0.05)
    moveDistance(vel_pub, 0.1606)
    turnAngle(vel_pub, -rightAngle)
    moveDistance(vel_pub, 0.562)
    turnAngle(vel_pub, -rightAngle)
    moveDistance(vel_pub, 1.0839)

    client.set_pen_state(True)
    turnAngle(vel_pub, -rightAngle)
    moveDistance(vel_pub, 1.4047)
    turnAngle(vel_pub, -rightAngle)
    moveDistance(vel_pub, 1.3252)
    client.set_pen_state(False)

    turnAngle(vel_pub, angle1)
    moveDistance(vel_pub, 0.5034)
    turnAngle(vel_pub, -(2*angle1))
    moveDistance(vel_pub, 0.5034)
    turnAngle(vel_pub, -(angle2-0.05))
    moveDistance(vel_pub, 0.4416)

def drawM(vel_pub):
    #TODO: Draw a Block M
    turnAngle(vel_pub, PI/2)

    moveDistance(vel_pub, .56204)
    turnAngle(vel_pub, -PI/2)

    moveDistance(vel_pub,.20073)
    turnAngle(vel_pub, PI/2)

    moveDistance(vel_pub, 1.7263)
    turnAngle(vel_pub, PI/2)

    moveDistance(vel_pub, .20073)
    turnAngle(vel_pub, -PI/2)

    moveDistance(vel_pub, .56204)
    turnAngle(vel_pub, -PI/2)

    moveDistance(vel_pub, .88321)
    turnAngle(vel_pub, -1.107)

    moveDistance(vel_pub, 1.3465)
    turnAngle(vel_pub, 2.214)

    moveDistance(vel_pub, 1.3465)
    turnAngle(vel_pub, -1.107)

    moveDistance(vel_pub, .88321)
    turnAngle(vel_pub, -PI/2)

    moveDistance(vel_pub, 0.56204)
    turnAngle(vel_pub, -PI/2)

    moveDistance(vel_pub, .20073)
    turnAngle(vel_pub, PI/2)

    moveDistance(vel_pub, 1.7263)
    turnAngle(vel_pub, PI/2)

    moveDistance(vel_pub, .20073)
    turnAngle(vel_pub, -PI/2)

    moveDistance(vel_pub, .56204)
    turnAngle(vel_pub, -PI/2)

    moveDistance(vel_pub, 1.0237)
    turnAngle(vel_pub, -PI/2)

    moveDistance(vel_pub, .56204)
    turnAngle(vel_pub, -PI/2)

    moveDistance(vel_pub, .20074)
    turnAngle(vel_pub, PI/2)

    moveDistance(vel_pub, 1.2044)
    turnAngle(vel_pub, 2.6012)

    moveDistance(vel_pub, 1.4045)
    turnAngle(vel_pub, -2.06078)

    moveDistance(vel_pub, 1.4045)
    turnAngle(vel_pub, 2.6012)

    moveDistance(vel_pub, 1.2044)
    turnAngle(vel_pub, PI/2)

    moveDistance(vel_pub, .20073)
    turnAngle(vel_pub, -PI/2)

    moveDistance(vel_pub, .56204)
    turnAngle(vel_pub, -PI/2)

    moveDistance(vel_pub, 1.0237)
    
    pass #remove this line when you develop this function

def main():
    rclpy.init()
    tam_node = rclpy.create_node('block_tam_pub')
    vel_pub = tam_node.create_publisher(Twist, '/turtle1/cmd_vel', 10)


    client = TurtleClient()
    #Clear existing drawings
    client.make_clear()

    #Background to Maroon
    client.set_color(80, 0, 0)
    
    #Set pen color to White
    client.set_pen_color(255, 255, 255)

    #move from (5.5,5.5,0) to T starting positon (4.135, 2.148, PI/2)
    client.set_pen_state(True)
    turnAngle(vel_pub, PI)
    moveDistance(vel_pub, 5.5-4.13503649635)
    turnAngle(vel_pub, PI/4)
    moveDistance(vel_pub, 5.5-2.14781021898)
    turnAngle(vel_pub, PI)
    client.set_pen_state(False)

    drawT(vel_pub)

    #TODO: transistion from T to A

    drawA(vel_pub, client)

    #TODO: transistion from A to M
    
    drawM(vel_pub)

    # move away from drawing
    client.set_pen_state(True)
    moveDistance(vel_pub, 10)

if __name__ == '__main__':
    main()