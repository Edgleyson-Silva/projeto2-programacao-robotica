#! shebang

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math

class myRobot():

    def __init__(self):
        print('init')
        # Subscriber odometria
        self.odometria_sub = rospy.Subscriber('/mobile_base_controller/odom', Odometry, self.callback_odometria(), queue_size=1)
        self.row = 0
        self.pitch = 0
        self.yaw = 0
        self.kP = 0.5
        # Subscriber laser
        # Client Service camera
        # Publisher base
        self.base_pub = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=1)
        self.cmd = Twist()
        # Publisher cabeca

    def callback_odometria(self, msg):
        print('callback odometria')
        # Armazenar os dados de odometria
        self.orientation = msg.pose.pose.orientation
        self.orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        (self.row, self.pitch, self.yaw) = euler_from_quaternion(orientation_list)

    def callback_laser(self, msg):
        print('callback laser')
        # Armazenar os dados do laser

    def moveStaright(self):
        print('move straight')
        # error = ...
        # while(abs(error) < value):

    def turn(self, sens):
        print('turn')
        target_angle = 90
        target_rad = target_angle * math.pi/180
        while(abs(error) < value):
            error = target_rad - self.yaw
            cmd.angular.z = self.kP * error
            self.base_pub.publish(cmd)

    def decision(self):
        print('decision')
        #

if __name__ == '__main__':

    rospy.init_node('nodeName')

    tiago = myRobot()

    state = 0
    # while(...):
     # if state == 0:
        # decision
        # compute next state
     # else if state == 1
        # image porcessing
        # compute next state
     # else if state == 3
        # move straight
        # compute next state
     # else if state == 4
        # turn
        # compute next state
