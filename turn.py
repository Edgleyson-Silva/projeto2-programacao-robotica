#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan
from programacao.srv import addImage, addImageResponse
from sensor_msgs.msg import Image
import math
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from camera import myCamera

class myRobot():

    def __init__(self):
        print('init')
        # Subscriber odometria
        self.sub_odom = rospy.Subscriber('/mobile_base_controller/odom', Odometry, self.callback_odom, queue_size=1)
        self.row = 0
        self.midview = 0
        self.leftview = 0
        self.rightview = 0
        self.pitch = 0
        self.yaw = 0
        self.kP = 1 #controlador arbitrario
        # Subscriber laser
        self.sub_laser = rospy.Subscriber('/scan_raw', LaserScan, self.callback_laser, queue_size=1)
        # Client Service camera

        # Publisher base
        self.pub_cmd = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=1)
        self.cmd = Twist()
        # Publisher cabeca
        self.pub_head = rospy.Publisher('/head_controller/command', JointTrajectory, queue_size=1)
        self.cmd_head = JointTrajectory()
        self.points = JointTrajectoryPoint()

    def callback_odom(self, msg):
        print('callback odometria')
        # Armazenar os dados de odometria
        self.orientation = msg.pose.pose.orientation #posicao quaternion
        self.orientation_list = [self.orientation.x, self.orientation.y, self.orientation.z, self.orientation.w]
        (self.row, self.pitch, self.yaw) = euler_from_quaternion(self.orientation_list)
        print(self.yaw)

    def error(self,target):
        error = target - self.yaw
        print(error)
        print(self.yaw)
        return error

    def turn(self, sensor, target):
        print('turn')
        while (abs(self.error(target)) > 0.01):
            self.cmd.angular.z = sensor*self.kP*self.error(target)
            self.pub_cmd.publish(self.cmd)
        self.cmd.angular.z = 0
        self.pub_cmd.publish(self.cmd)

if __name__ == '__main__':
    # Define the node
    rospy.init_node('TIAGo_odem_node')

    # Create an object of class mySub and run the init function
    subObj = myRobot()
    target = -90 * math.pi / 180 + subObj.yaw
    subObj.error(target)
    sensor = 1
    subObj.turn(sensor, target)
    subObj.moveStraight()