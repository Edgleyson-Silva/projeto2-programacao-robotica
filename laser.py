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
        #self.counterValue = msg.data
        leftdata = 0
        middata = 0
        rightdata = 0
        for i in range(0,198):
            leftdata = leftdata + msg.ranges[i]
        for j in range(198,470):
            middata = middata + msg.ranges[j]
        for k in range(470,665):
            rightdata = rightdata + msg.ranges[k]
        leftview = leftdata/198
        midview = middata/272
        rightview = rightdata/198
        print(leftview)
        print(midview)
        print(rightview)

# Main program
if __name__ == '__main__':
    # Define the node
    rospy.init_node('TIAGo_laser_node')
    # Creaete an object of class mySub and run the init function
    subObj = mySub()
    # While ROS is running
    rospy.spin()
