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
        self.sub_odom = rospy.Subscriber('/mobile_base_controller/odom', Odometry, self.callback_odom, queue_size=1)
        self.row = 0
        self.midview = 0
        self.leftview = 0
        self.rightview = 0
        self.pitch = 0
        self.yaw = 0
        self.kP = 0.5 #controlador arbitrario
        # Subscriber laser
        self.sub_laser = rospy.Subscriber('/scan_raw', LaserScan, self.callback_laser, queue_size=1)
        # Client Service camera
        # Publisher base
        self.pub_cmd = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=1)
        self.cmd = Twist()
        # Publisher cabeca

    def callback_odom(self, msg):
        print('callback odometria')
        # Armazenar os dados de odometria
        self.orientation = msg.pose.pose.orientation #posicao quaternion
        self.orientation_list = [self.orientation.x, self.orientation.y, self.orientation.z, self.orientation.w]
        (self.row, self.pitch, self.yaw) = euler_from_quaternion(self.orientation_list)
        print(self.yaw)

    def callback_laser(self, msg):
        self.midview = msg.ranges[333]
        leftdata = 0
        rightdata = 0
        for i in range(0,83):
            leftdata = leftdata + msg.ranges[i]
        for k in range(583,666):
            rightdata = rightdata + msg.ranges[k]
        self.leftview = leftdata/83
        self.rightview = rightdata/83
        print(self.leftview)
        print(self.rightview)
        print(self.midview)

    def moveStraight(self):
        target_distance = 1
        error = target_distance - self.midview

        while (abs(error)>0.1):
            error = target_distance - self.midview
            print(error)
            self.cmd.linear.x = self.kP*abs(error)
            self.pub_cmd.publish(self.cmd)
        self.move_error = error

    def turn(self):
        print('turn')
        if self.move_error < 0.1:
            target_angle = 90
            target_rad = target_angle * math.pi/180 #degree to rad
            error = target_rad - self.yaw
            while(abs(error) > 0.1):
                error = target_rad - self.yaw
                print('odom {}' .format(error))
                self.cmd.angular.z = self.kP * error
                self.pub_cmd.publish(self.cmd)
       # elif self.move_error < 0.1 and self.rightview < 1.5:
        #    target_angle = -90
         #   target_rad = target_angle * math.pi / 180  # degree to rad
          #  error = target_rad - self.yaw
          #  while (abs(error) > 0.1):
           #     error = target_rad - self.yaw
            #    self.cmd.angular.z = self.kP * error
             #   self.pub_cmd.publish(self.cmd)

if __name__ == '__main__':
    # Define the node
    rospy.init_node('TIAGo_odem_node')
    # Create an object of class mySub and run the init function
    subObj = myRobot()
    subObj.moveStraight()
    subObj.turn()
    subObj.moveStraight()
    # While ROS is running
    rospy.spin()