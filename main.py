#! /usr/bin/python

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

    def callback_laser(self, msg):
        self.midview = msg.ranges[333] #sensor do ranges central
        leftdata = 0
        rightdata = 0
        for i in range(0,83):
            rightdata = rightdata + msg.ranges[i]
        for k in range(583,666):
            leftdata = leftdata + msg.ranges[k]
        self.leftview = leftdata/83 #faixa considerada como visao esquerda
        self.rightview = rightdata/83 #faixa considerada como visao direita
       # print(self.leftview)
        print(self.rightview)
        #print(self.midview)

    def moveStraight(self):
        target_distance = 0.7
        error = target_distance - self.midview

        while (abs(error)>0.01):
            error = target_distance - self.midview
            print(error)
            self.cmd.linear.x = self.kP*abs(error)
            self.pub_cmd.publish(self.cmd)
            print('esquerda: {}' .format(self.leftview))
            print('direita: {}'.format(self.rightview))
        self.move_error = error

    def error(self,target):
        error = target - self.yaw
        print('erro: {}'.format(error))
        print('angulo atual: {}'.format(self.yaw))
        return error

    def turn(self, sensor, target):
        print('turn')
        while (abs(self.error(target)) > 0.01):
            self.cmd.angular.z = sensor*self.kP*self.error(target)
            self.pub_cmd.publish(self.cmd)
        self.cmd.angular.z = 0
        self.pub_cmd.publish(self.cmd)
        
    def SubscribeCamera(self, msg):
        print('callback camera')
        rospy.wait_for_service('addImage_service_name')
        try:
            h_addImage = rospy.ServiceProxy('addImage_service_name', addImage)
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            #request =addImageRequest()
            self.response = h_addImage()
        except CvBridgeError as e:
            print(e)

if __name__ == '__main__':
    # Define the node
    rospy.init_node('TIAGo_odem_node')

    # Create an object of class mySub and run the init function
    subObj = myRobot()
    state = 0
    while(...):
        if state == 0:
            if subObj.midview > 0.7:
                state = 3
            else:
                state = 1
        elif state == 1:
            if subObj.leftview < 1.3:
                target = -90 * math.pi / 180 + subObj.yaw
                subObj.error(target)
                sensor = 1
                subObj.turn(sensor, target)
                state = 0
            elif subObj.rightview < 1.3:
                target = 90 * math.pi / 180 + subObj.yaw
                subObj.error(target)
                sensor = -1
                subObj.turn(sensor,target)
                subObj.moveStraight()
                state = 0
        elif state == 3:
            subObj.moveStraight()
            state = 0
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

    if subObj.leftview < 1.3 and subObj.rightview < 1.3:
        subObj.moveStraight()
    if subObj.move_error < 0.01:
        if subObj.leftview < 1.3:
            target = -90 * math.pi / 180 + subObj.yaw
            subObj.error(target)
            sensor = 1
            subObj.turn(sensor,target)
            subObj.moveStraight()
        elif subObj.rightview < 1.3:
            target = 90 * math.pi / 180 + subObj.yaw
            subObj.error(target)
            sensor = -1
            subObj.turn(sensor,target)
            subObj.moveStraight()
    if subObj.leftview > 1.3 or subObj.rightview > 1.3:
        subObj.cmd_head.joint_names.append("head_1_joint")
        subObj.cmd_head.joint_names.append("head_2_joint")
        subObj.points.positions = [0] * 2
        subObj.points.time_from_start = rospy.Duration(1)
        subObj.cmd_head.points.append(subObj.points)
        angle = 0.1
        while angle < 2:
            subObj.cmd_head.points[0].positions[0] = angle
            subObj.cmd_head.points[0].time_from_start = rospy.Duration(1)
            subObj.pub_head.publish(subObj.cmd_head)
            angle = angle + 0.01
            print(angle)
        print('camera')
        rospy.sleep(3)
       # rospy.init_node('addImage_service_name')
        subCam = myCamera()
        my_service = rospy.Service('addImage_service_name', addImage, subCam.callback_ServiceCamera)
        if subCam.resposta == 'green':
            target = 90 * math.pi / 180 + subObj.yaw
            subObj.error(target)
            sensor = -1
            subObj.turn(sensor, target)
            subObj.moveStraight()
        elif subCam.resposta == 'red':
	    target = -90 * math.pi / 180 + subObj.yaw
            subObj.error(target)
            sensor = 1
            subObj.turn(sensor,target)
            subObj.moveStraight()

           
    # While ROS is running
    rospy.spin()
