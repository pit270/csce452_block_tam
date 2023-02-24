import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from turtlesim.msg import Color

def drawT(vel_pub):
    #TODO: Draw a Block T
    pass #remove this line when you develop this function

def drawA(vel_pub):
    #TODO: Draw a Block A
    pass #remove this line when you develop this function

def drawM(vel_pub):
    #TODO: Draw a Block M
    pass #remove this line when you develop this function

def main():
    rclpy.init()
    tam_node = rclpy.create_node('block_tam_pub')
    vel_pub = tam_node.create_publisher(Twist, '/turtle1/cmd_vel', 10)

    #TODO: Set Background to Maroon
    #TODO: Set pen color to White

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