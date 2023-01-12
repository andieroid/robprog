#!/usr/bin/env python

import rospy
import cv2
import numpy as np
import math
import matplotlib.pyplot as plt
import time
from cv2 import namedWindow, cvtColor, imshow, inRange
from cv2 import destroyAllWindows, startWindowThread
from cv2 import COLOR_BGR2GRAY, waitKey, COLOR_BGR2HSV
from cv2 import blur, Canny, resize, INTER_CUBIC

from numpy import mean
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# THIS PROGRAM IS BASED AROUND AND RELIES ON A RANGE OF MODIFIED PROGRAMS AND CONFIG FILES
# PROVIDED IN THE UoL MODULE EXAMPLES.  THESE ARE:
# OPENCV_TEST.PY
# TOPO_NAV.LAUNCH
# TEST.MAP2

class image_converter:

    def __init__(self):

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/thorvald_001/kinect2_right_camera/hd/image_color_rect", Image, self.image_callback)

    def image_callback(self, data):
        namedWindow("Raw Image")

        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        cv_image = resize(cv_image, None, fx=0.5, fy=0.5, interpolation = INTER_CUBIC)
        hsv_img = cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        mask = inRange(hsv_img, (102, 49, 0), (111, 96, 73))
        imshow("Raw Image", cv_image)
        waitKey(1)
        img_dilated = img_closing = img_opening = mask
        kernel = np.ones((5,5),np.uint8)
        closing_mask = cv2.morphologyEx(img_closing,cv2.MORPH_CLOSE,kernel, iterations = 5)
        bw_grape_image = cv2.cvtColor(closing_mask, cv2.COLOR_GRAY2BGR)
        img = bw_grape_image
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret,thresh = cv2.threshold(gray,170,255,0)
        contours,hierarchy = cv2.findContours(thresh, cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        batch = 0
        x1 = 0
        print("waiting for vine to fill screen...")
        cv2.imshow("Grape Bunch Count", img)
        cv2.waitKey(1)
        grapecounter = [] 
        batch_schedule = [10, 20, 30, 40, 50, 60, 70, 80, 90]
        totaltime = 0
        for i, cnt in enumerate(contours):
            x1, y1 = cnt[0,0]
            img1 = cv2.drawContours(img, [cnt], -1, (0,255,0), 1)
            cv2.putText(img1, f'{y1}', (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            if x1>900:
                # start the timer
                print("Counting started")
                starttime = time.time()
                lasttime = starttime
                while totaltime <30:
                    print('Time elapsed:', totaltime)
                    for samplepoint in batch_schedule:  
                        if totaltime == samplepoint:
                            print("Sampling at ", sampletime, "seconds")             
                            print("Counting batch",batch)
                            batch_count = len(contours)
                            grapecounter.append([batch_count])
                            print('Batch count[batch]', batch_count)
                            img1 = cv2.drawContours(img, [cnt], -1, (0,255,0), 1)
                            cv2.putText(img1, f'{y1}:COUNTED', (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                            batch +=1
                            totaltime = round((time.time() - starttime), 2)
                            img1 = cv2.drawContours(img, [cnt], -1, (0,255,0), 1)
                            cv2.imshow("Grape Bunch Count", img)
                            cv2.waitKey(1)
                            # Updating the previous total time and lap number
                            lasttime = time.time()
                            print("Total Time: "+str(totaltime))
                            print("Grape count for Left of vine:",grapecounter)

        cv2.imshow("Grape Bunch Count", img)
        cv2.waitKey(1)
        # cv2.destroyAllWindows()

#startWindowThread()
rospy.init_node('image_converter')
ic = image_converter()
rospy.spin()

destroyAllWindows()

