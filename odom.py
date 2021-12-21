#! /usr/bin/python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan
import math

class myRobot():

    def __init__(self):
        print('init')
        # Subscriber odometria
        self.sub = rospy.Subscriber('/mobile_base_controller/odom', Odometry, self.callback, queue_size=1)
        self.row = 0
        self.pitch = 0
        self.yaw = 0
        self.kP = 0.5 #controlador arbitrario
        # Subscriber laser
        #self.sub_laser = rospy.Subscriber('/scan_raw', LaserScan, self.callback_laser, queue_size=1)
        # Client Service camera
        # Publisher base
        self.pub = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=1)
        self.cmd = Twist()
        # Publisher cabeca

    def callback(self, msg):
        print('callback odometria')
        # Armazenar os dados de odometria
        self.orientation = msg.pose.pose.orientation #posicao quaternion
        self.orientation_list = [self.orientation.x, self.orientation.y, self.orientation.z, self.orientation.w]
        (self.row, self.pitch, self.yaw) = euler_from_quaternion(self.orientation_list)
        print(self.yaw)

    def turn(self):
        print('turn')
        target_angle = 90 #target_angleu funcao do que vai ser captado no sens
        target_rad = target_angle * math.pi/180 #degree to rad
        error = target_rad - self.yaw
        while(abs(error) > 0.01):
            error = target_rad - self.yaw
            self.cmd.angular.z = self.kP * error
            self.pub.publish(self.cmd)

if __name__ == '__main__':
    # Define the node
    rospy.init_node('TIAGo_odem_node')
    # Create an object of class mySub and run the init function
    subObj = myRobot()
    subObj.turn()
    # While ROS is running
    rospy.spin()
