#!/usr/bin/env python

import rospy
from cv2 import namedWindow, cvtColor, imshow, inRange

from cv2 import destroyAllWindows, startWindowThread
from cv2 import COLOR_BGR2GRAY, waitKey
from cv2 import blur, Canny, resize, INTER_CUBIC
from numpy import mean
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class image_converter:

    def __init__(self):

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/thorvald_001/kinect2_left_camera/hd/image_color_rect",
                                          Image, self.image_callback)
        # self.image_sub = rospy.Subscriber(
        #     "/camera/rgb/image_raw",
        #     Image, self.callback)

    def image_callback(self, data):
        namedWindow("Image window")
        namedWindow("masked")
        # namedWindow("canny")
        # namedWindow("mask_out_green")
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        cv_image = resize(cv_image, None, fx=0.2, fy=0.2, interpolation = INTER_CUBIC)

        mask = inRange(cv_image, (120, 120, 0), (160, 160, 160))
        imshow("masked", mask)

        gray_img = cvtColor(cv_image, COLOR_BGR2GRAY)
        
        # img3 = Canny(gray_img, 10, 200)
        # imshow("canny", img3)

        # masked_green = inRange(cv_image, (71, 83, 97), (255, 255, 255))
        # imshow("mask_out_green", masked_green)

        imshow("Image window", cv_image)
        waitKey(1)





#startWindowThread()
rospy.init_node('image_converter')
ic = image_converter()
rospy.spin()

#destroyAllWindows()



# hsv_thresh = cv2.inRange(hsv_img,
#     numpy.array((0, 150, 50)),
#     numpy.array((255, 255, 255)))