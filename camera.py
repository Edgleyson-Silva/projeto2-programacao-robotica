#! shebang

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from programacao.srv import addImage, addImageResponse

class myCamera():

    def __init__(self):
        print('init camera')
        # Bridge to convert ROS message to openCV
        self.bridge = CvBridge()
        self.resposta = ''
        # Subscriber to the camera image
        self.image_sub = rospy.Subscriber("/xtion/rgb/image_color",Image,SubscribeCamera)

        # Server Service camera


    def callback_ServiceCamera(self, request):
        print('executing addImage service')
        request = self.cv_image
        (blueChannel, greenChannel, redChannel) = cv2.split(request)
        #cv2.imshow("Green", greenChannel)
        ret, t = cv2.threshold(greenChannel, 50, 255, cv2.THRESH_BINARY)
        blackpx = 0
        row = len(t)
        column = len(t[0])
       # cv2.imshow("thresh", t)
       # cv2.waitKey(0)
        for i in range(0, row):
            for j in range(0, column):
                px = t[i][j]
                if px == 0:
                    blackpx = blackpx + 1
                    print('blackpx: {}'.format(blackpx))
        if blackpx > 100:
            self.resposta = addImageResponse('red')
            print(self.resposta)
            return self.resposta
        else:
            self.resposta = addImageResponse('green')
            print(self.resposta)
            return self.resposta




        # cv_image[linha][coluna][bgr] bgr-> 0:blue, 1:green, 2:red
        #print(self.cv_image[0][0])
        #print(self.cv_image[0][0][0])

        # Display the image
        #cv2.imshow("raw", self.cv_image)
        #cv2.waitKey(3)
