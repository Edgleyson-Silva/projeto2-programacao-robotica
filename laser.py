#! /usr/bin/python

import rospy
from sensor_msgs.msg import LaserScan

# Definition of the class
class mySub():

    def __init__(self):
        # Define the subscriber
        self.sub = rospy.Subscriber('/scan_raw', LaserScan, self.callback, queue_size=1)

    # Definition of the function called by the subscriber
    def callback(self, msg):
        self.midview = msg.ranges[333]
        leftdata = 0
        rightdata = 0
        for i in range(0,83):
            leftdata = leftdata + msg.ranges[i]
        for k in range(583,666):
            rightdata = rightdata + msg.ranges[k]
        self.leftview = leftdata/83
        self.rightview = rightdata / 83
        print(self.leftview)
        print(self.rightview)
        print(self.midview)

    def moveStraight(self):
        target_distance = 1
        error = target_distance - self.midview

        while (abs(error)>1):
            error = target_distance - self.midview
            self.cmd.linear.x = kP*error
            self.pub.publish(self.cmd)

# Main program
if __name__ == '__main__':
    # Define the node
    rospy.init_node('TIAGo_laser_node')
    # Creaete an object of class mySub and run the init function
    subObj = mySub()
    # While ROS is running
    rospy.spin()
