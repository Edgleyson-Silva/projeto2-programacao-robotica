#! shebang

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from programacao.srv import add_image, add_imageResponse

class myCamera():

    def __init__(self):
        print('init camera')
        # Bridge to convert ROS message to openCV
        self.bridge = CvBridge()

        # Subscriber to the camera image
        self.image_sub = rospy.Subscriber("/xtion/rgb/image_color",Image,self.callback_SubscribeCamera)

        # Server Service camera




    def callback_ServiceCamera(self, request):
        print('executing add_image service')
        (blueChannel, greenChannel, redChannel) = cv2.split(request.img)
        cv2.imshow("Green", greenChannel)
        ret, t = cv2.threshold(canalVerde, 160, 255, cv2.THRESH_BINARY)
        blackpx = 0
        whitepx = 0
        row = len(t)
        column = len(t[0])
        cv2.imshow("thresh", t)
        cv2.waitKey(0)
        for i in range(0, row):
            for j in range(0, column):
                px = t[i][j]
                if px == 0:
                    blackpx = blackpx + 1 #pixels verdes
                elif px == 255:
                    whitepx = whitepx + 1 #pixels vermelhos
        if whitepx > blackpx:
            return add_imageResponse('green')
        elif blackpx >  whitepx:
            return add_imageResponse('red')


    def callback_SubscribeCamera(self, msg):
        print('callback camera')
        rospy.wait_for_service('add_image_service_name')
        try:
            h_add_image = rospy.ServiceProxy('add_image_service_name', add_image)
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            request =add_imageRequest()
            request.img = self.cv_image
            self.response = h_add_image(request)
        except CvBridgeError as e:
            print(e)

        # cv_image[linha][coluna][bgr] bgr-> 0:blue, 1:green, 2:red
        print(self.cv_image[0][0])
        print(self.cv_image[0][0][0])

        # Display the image
        cv2.imshow("raw", self.cv_image)
        cv2.waitKey(3)
