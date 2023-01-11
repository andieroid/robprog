#!/usr/bin/env python

import rospy
import cv2
import numpy as np
import matplotlib.pyplot as plt
from cv2 import namedWindow, cvtColor, imshow, inRange
from cv2 import destroyAllWindows, startWindowThread
from cv2 import COLOR_BGR2GRAY, waitKey, COLOR_BGR2HSV
from cv2 import blur, Canny, resize, INTER_CUBIC

from numpy import mean
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class image_converter:

    def __init__(self):

        self.bridge = CvBridge()
        # self.image_sub = rospy.Subscriber("/thorvald_001/kinect2_right_camera/hd/image_color_rect",
        #                                   Image, self.image_callback)
        self.image_sub = rospy.Subscriber("/thorvald_001/kinect2_right_camera/hd/image_color_rect", Image, self.image_callback)



    def image_callback(self, data):
        namedWindow("Raw Image")
        namedWindow("Single mask")
        # namedWindow("Eroded")
        # namedWindow("Dilated")

        namedWindow("Image Closing Function")

        ############################################################################
        # CREATING MASK
        ############################################################################

        # namedWindow("Composite mask")

        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        cv_image = resize(cv_image, None, fx=0.5, fy=0.5, interpolation = INTER_CUBIC)

        hsv_img = cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        mask = inRange(hsv_img, (102, 49, 0), (111, 96, 73))

        # (hMin = 71 , sMin = 29, vMin = 0), (hMax = 124 , sMax = 176, vMax = 91)
        # (hMin = 102 , sMin = 49, vMin = 0), (hMax = 111 , sMax = 96, vMax = 73)

        imshow("Single mask", mask)

        ############################################################################
        # COMPOSITE MASK APPROACH (NOT USED IN THE END)
        ############################################################################

        # #Add additional masks
        # mask1 = inRange(hsv_img, (82, 50, 50), (90, 255, 255)) 
        # mask2 = inRange(hsv_img, (109, 50, 50), (112, 255, 255))
        # mask3 = inRange(hsv_img, (100, 50, 50), (100, 255, 255))
        # mask4 = inRange(hsv_img, (108, 50, 50), (100, 255, 255))
        # mask5 = inRange(hsv_img, (120, 50, 50), (100, 255, 255))

        # new_image = (mask + mask1 + mask2 + mask3 + mask4 + mask5)
        # # new_image = new_image.clip(0, 255).astype("uint8")

        # imshow("Composite mask", new_image)

        ############################################################################

        # kernel = np.ones((5,5),np.uint8)
        # dilated = cv2.dilate(new_image,kernel,iterations = 2)
        # imshow("dilated", dilated)

        imshow("Raw Image", cv_image)
        waitKey(1)

        # # Erode image to attempt to remove noisy pixels - turns out that this was too harsh
        # img_eroded = mask
        # kernel = np.ones((3,3),np.uint8)
        # erosion = cv2.erode(img_eroded,kernel,iterations = 1)
        # imshow("Eroded", erosion)

        # # 'Opening' image to attempt to remove noisy pixels, followed by dilation
        # img_opening = mask
        # kernel = np.ones((2,2),np.uint8)
        # opening = cv2.morphologyEx(img_opening,cv2.MORPH_OPEN,kernel)
        # imshow("Opening", opening)

        # dilate white pixels to expand larger block (assuming grape bunches)
        img_dilated = img_closing = img_opening = mask
        kernel = np.ones((5,5),np.uint8)
        # dilation = cv2.dilate(img_dilated,kernel,iterations = 6)
        closing_mask = cv2.morphologyEx(img_closing,cv2.MORPH_CLOSE,kernel, iterations = 5)
        # opening_mask = cv2.morphologyEx(img_opening,cv2.MORPH_OPEN,kernel)
        # imshow("Dilated", dilation)
        # imshow("Opening", opening)
        imshow("Image Closing Function", closing_mask)

        ############################################################################
        # FIND CENTRE OF MULTIPLE BLOBS IN THE IMAGE 
        ############################################################################
        
        # Need to convert closing mask from B&W to 3 channel image 
        bw_grape_image = cv2.cvtColor(closing_mask, cv2.COLOR_GRAY2BGR)
        closing_mask = cv2.cvtColor(bw_grape_image, cv2.COLOR_BGR2GRAY)

        img = closing_mask
       
        # convert the grayscale image to binary image
        ret,thresh = cv2.threshold(img,127,255,0)
        
        # find contours in the binary image
        im2, contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        for c in contours:
        # calculate moments for each contour
            M = cv2.moments(c)
        # calculate x,y coordinate of center
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            cv2.circle(img, (cX, cY), 5, (255, 255, 255), -1)
            cv2.putText(img, "centroid", (cX - 25, cY - 25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        
        # display the image
            cv2.imshow("Image", img)
            cv2.waitKey(0)

        ############################################################################





#startWindowThread()
rospy.init_node('image_converter')
ic = image_converter()
rospy.spin()

destroyAllWindows()







