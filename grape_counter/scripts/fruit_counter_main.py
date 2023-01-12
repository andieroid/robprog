#!/usr/bin/env python

import rospy
import cv2
import numpy as np
import math
import matplotlib.pyplot as plt
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
        # self.image_sub = rospy.Subscriber("/thorvald_001/kinect2_right_camera/hd/image_color_rect",
        #                                   Image, self.image_callback)
        self.image_sub = rospy.Subscriber("/thorvald_001/kinect2_right_camera/hd/image_color_rect", Image, self.image_callback)

    def image_callback(self, data):
        namedWindow("Raw Image")
        # namedWindow("Single mask")
        # namedWindow("Eroded")
        # namedWindow("Dilated")
        # namedWindow("Image Closing Function")

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

        # imshow("Single mask", mask)

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
        # MODIFYING THE GREYSCALE IMAGE TO REMOVE NOISE AND GROW REGIONS THAT APPEAR
        # TO BE LEGITIMATE GRAPE BUNCHES
        # https://docs.opencv.org/4.x/d9/d61/tutorial_py_morphological_ops.html
        # COMMENTED-OUT CODE WAS IMPLEMENTED AND REJECTED 
        ############################################################################

        # kernel = np.ones((5,5),np.uint8)
        # dilated = cv2.dilate(new_image,kernel,iterations = 2)
        # imshow("dilated", dilated)

        # for reference
        imshow("Raw Image", cv_image)
        waitKey(1)

        # # Erode image to attempt to remove noisy pixels 
        # Too harsh - black image
        # img_eroded = mask
        # kernel = np.ones((3,3),np.uint8)
        # erosion = cv2.erode(img_eroded,kernel,iterations = 1)
        # imshow("Eroded", erosion)

        # # 'Opening' image to attempt to remove noisy pixels, followed by dilation
        # Over zeleous - resuted in almost totally black image on lowest threshold setting
        # img_opening = mask
        # kernel = np.ones((2,2),np.uint8)
        # opening = cv2.morphologyEx(img_opening,cv2.MORPH_OPEN,kernel)
        # imshow("Opening", opening)

        ############################################################################
        # EFFECTIVE IN EXPANDING LAREGER COLLECTIONS (GRAPE BUNCHES) BUT ALSO EXPANDS 
        # NOISE PIXELS TURING THEM INTO AREAS SIMILAR TO GRAPE BUNCHES SO HARD TO 
        # DISCRIMINATE/REMOVE LATER
        ############################################################################ 
        # dilate white pixels to expand larger block (assuming grape bunches)
        img_dilated = img_closing = img_opening = mask
        kernel = np.ones((5,5),np.uint8)
        # dilation = cv2.dilate(img_dilated,kernel,iterations = 6)
        closing_mask = cv2.morphologyEx(img_closing,cv2.MORPH_CLOSE,kernel, iterations = 5)
        # opening_mask = cv2.morphologyEx(img_opening,cv2.MORPH_OPEN,kernel)
        # imshow("Dilated", dilation)
        # imshow("Opening", opening)
        # imshow("Image Closing Function", closing_mask)

        ############################################################################
        # COUNT MULTIPLE BLOBS IN THE IMAGE 
        # INEFFICIENT METHOD
        ############################################################################
        
        # # Need to convert closing mask from B&W to 3 channel image 
        # bw_grape_image = cv2.cvtColor(closing_mask, cv2.COLOR_GRAY2BGR)
        # closing_mask = cv2.cvtColor(bw_grape_image, cv2.COLOR_BGR2GRAY)

        # img = closing_mask
       
        # # convert the grayscale image to binary image
        # ret,thresh = cv2.threshold(img,127,255,0)
        
        # # find contours in the binary image
        # im2, contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        # for c in contours:
        # # calculate moments for each contour
        #     M = cv2.moments(c)
        # # calculate x,y coordinate of center
        #     cX = int(M["m10"] / M["m00"])
        #     cY = int(M["m01"] / M["m00"])
        #     cv2.circle(img, (cX, cY), 5, (255, 255, 255), -1)
        #     cv2.putText(img, "centroid", (cX - 25, cY - 25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        
        # # display the image
        #     cv2.imshow("Image", img)
        #     cv2.waitKey(0)

        ############################################################################
        # IDENTIFY AND COUNT INDIVIDUAL CONTOURS
        # https://www.tutorialspoint.com/how-to-compute-image-moments-in-opencv-python        
        ############################################################################

        # Need to convert closing mask from gray to 3 channel image for this code to function
        bw_grape_image = cv2.cvtColor(closing_mask, cv2.COLOR_GRAY2BGR)
        img = bw_grape_image
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret,thresh = cv2.threshold(gray,170,255,0)
        contours,hierarchy = cv2.findContours(thresh, cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        batch = 0

        # declare how many contours (bunches) can be detected 
        # print("Number of Contours detected:",len(contours))
        # consider each contour (blob) in the visible image
        for i, cnt in enumerate(contours):
            # M = cv2.moments(cnt)
            # Creating and initialise a list to contain lists of [area, y-co-ordinate] - i.e. unique IDs
            keyblobtracker = []
            # create and initialise list to record blob count of individual batches (to allow audit against real values on completion)
            grapecounter = [] 
            keyblobtracker = []   # start with list values at zero for blob_area, blob_height_from_top          
            x1, y1 = cnt[0,0]
            #[comments A came from here]
            # provide visual confirmation of blob areas
            print('Area of grape blob=', cv2.contourArea(cnt), 'Y COORDINATE=',y1,')')
            ################################################################
            # For ALL blobs appearing in the left 20 pixels of the image
            ################################################################

            if x1<20:
                # print to confirm tracker is reset
                # print('Key Blob Tracker List:',keyblobtracker)  
                # draw a green contour around key blob    
                img1 = cv2.drawContours(img, [cnt], -1, (0,255,0), 1)
                # Print distance from top of the image next to blob
                cv2.putText(img1, f'{y1}:KEY', (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                # increment batch (will become 1 at start of counting)
                batch +=1 # equivalent to a kinect camera frame width of 940px
                # Don't start tracking grapes until until after the first batch has been counted or 
                # grapes will be lost from first batch
                if batch==1: # don't track blobs
                    # keyblobtracker[batch] = [cv2.contourArea(cnt),y1]
                    # print("FIRST COUNT:",keyblobtracker[1])
                    cv2.putText(img1, f'{y1}:BATCH{batch}', (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                    batch_count = len(contours)
                    grapecounter.append([batch_count])
                    print('FIRST BATCH COUNT:', grapecounter)
                    batch+=1
                if batch>1: #start tracking blobs
                    cv2.putText(img1, f'{y1}:BATCH{batch}', (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                    # add unique identifier of tracked blob to blob-tracker list
                    keyblobtracker.append([batch, cv2.contourArea(cnt), y1])
                    print('KEY BLOB TRACKER CONTAINS:', keyblobtracker)
                    # tracker ID loaded
                    #robot moving forward (x1 increasing)
                    # as the robot advances, changes are likely to occur in lighting etc that will change the area and y values for 
                    # the blobs.  The use of the maths function "isclose" is used to try and match within 5% of the stored and live values. 
                
                if x1 >= 20:
                    print('Stored:', keyblobtracker[0][1],keyblobtracker[0][2],'Live: ',cv2.contourArea(cnt), y1)
                    if math.isclose(keyblobtracker[0][1], cv2.contourArea(cnt), rel_tol=0.10, abs_tol=0.0) or math.isclose(keyblobtracker[0][2], y1, rel_tol=0.10, abs_tol=0.0):
                        print('Stored:', keyblobtracker[0][1],keyblobtracker[0][2],'Live: ',cv2.contourArea(cnt), y1)
                        img1 = cv2.drawContours(img, [cnt], -1, (0,255,0), 1)
                        cv2.putText(img1, f'{y1}:<=TRACKING', (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

                ################################################################
                # for ANY blobs appearing within 20 pixels of right edge of image
                ################################################################
                    # If any blobs on RHS of image match contour AND y1 AND area 
                    # in keyblobtracker then the robot has (probably) moved forward by one complete kinect 
                    # image frame, so count grapes

                if x1>900:
                    # draw a red contour around key blob    
                    img1 = cv2.drawContours(img, [cnt], -1, (0,100,255), 1)
                    # identify blobs in right hand zone of image, print distance from top of the image
                    cv2.putText(img1, f'{y1}', (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 100, 255), 1)
                    # COMPARE IF ANY BLOBS APPEARING ON RHS OF IMAGE (940+ pixels) MATCH TRACKER DICTIONARY

        cv2.imshow("Grape Bunch Count", img)
        cv2.waitKey(1)
        # cv2.destroyAllWindows()

#startWindowThread()
rospy.init_node('image_converter')
ic = image_converter()
rospy.spin()

destroyAllWindows()

############################################################
# ALTERNATIVE APPROACH USING TIME-BASED SAMPLING
############################################################
# THE ROBOT TAKES 30 SECONDS TO TRAVERSE THE VINE
# IT ALSO TAKES 10 SECONDS TO FULLY REPLACE EACH OPTICAL FRAME WITH A NEW
# SECTION OF VINE.  THIS CODE WOULD BE IMPLEMENTED TO START COUNTING
# WHEN GRAPES ARE DETECTED FROM THE EXTREME LEFT TO EXTREME RIGHT
# THEN EVERY 10 SECONDS, A NEW COUNT WOULD TAKE PLACE
# COUNTING WOULD NOT OCCUR BETWEEN 30 SECONDS END POINT AND 
# DETECTION OF NEW FULL FRAME OF GRAPES ON OTHER SIDE OF VINE

# THE SAME ADJUSTMENT FACTOR WOULD BE APPLIED TO ACCOUNT FOR DOUBLE COUNTING

        # print("waiting for vine to fill screen...")
        # cv2.imshow("Grape Bunch Count", img)
        # cv2.waitKey(1)
        # grapecounter = [] 
        # batch_schedule = [10, 20, 30, 40, 50, 60, 70, 80, 90]
        # totaltime = 0
        # for i, cnt in enumerate(contours):
        #     x1, y1 = cnt[0,0]
        #     img1 = cv2.drawContours(img, [cnt], -1, (0,255,0), 1)
        #     cv2.putText(img1, f'{y1}', (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        #     if x1>900:
        #         # start the timer
        #         print("Counting started")
        #         starttime = time.time()
        #         lasttime = starttime
        #         while totaltime <30:
        #             print('Time elapsed:', totaltime)
        #             for samplepoint in batch_schedule:  
        #                 if totaltime == samplepoint:
        #                     print("Sampling at ", sampletime, "seconds")             
        #                     print("Counting batch",batch)
        #                     batch_count = len(contours)
        #                     grapecounter.append([batch_count])
        #                     print('Batch count[batch]', batch_count)
        #                     img1 = cv2.drawContours(img, [cnt], -1, (0,255,0), 1)
        #                     cv2.putText(img1, f'{y1}:COUNTED', (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        #                     batch +=1
        #                     totaltime = round((time.time() - starttime), 2)
        #                     img1 = cv2.drawContours(img, [cnt], -1, (0,255,0), 1)
        #                     cv2.imshow("Grape Bunch Count", img)
        #                     cv2.waitKey(1)
        #                     # Updating the previous total time and lap number
        #                     lasttime = time.time()
        #                     print("Total Time: "+str(totaltime))
        #                     print("Grape count for Left of vine:",grapecounter)