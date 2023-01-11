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


        ############################################################################
        # IDENTIFY AND COUNT INDIVIDUAL CONTOURS
        # https://www.tutorialspoint.com/how-to-compute-image-moments-in-opencv-python        
        ############################################################################

        # Need to convert closing mask from gray to 3 channel image for this code to function

        bw_grape_image = cv2.cvtColor(closing_mask, cv2.COLOR_GRAY2BGR)
        # closing_mask_gray = cv2.cvtColor(bw_grape_image, cv2.COLOR_BGR2GRAY)
        img = bw_grape_image
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret,thresh = cv2.threshold(gray,170,255,0)
        contours,hierarchy = cv2.findContours(thresh, cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        print("Number of Contours detected:",len(contours))
        for i, cnt in enumerate(contours):
            M = cv2.moments(cnt)
            # print(f"Moments of Contour {i+1}:\n", M). # USED FOR TESTING KEYS AGAINST CONTOUR DATA
            # EXTRACT CONTOUR DATA FROM GRAPES ON FAR LHS 
            # CONTOURS ON FAR LEFT OF IMAGE
            # FIND (HOPEFULLY) UNIQUE ELEMENTS OF BLOBS - INCLUDING THEIR y CO-ORDINATE, WHICH IT IS ASSUMED WILL NOT CHANGE IN THE 
            # SIMULATED WORLD.
            # CREATE A UNIQUE ID FOR ALL LEFT-HAND BLOBS FROM CONTOURS DATA
            # WHEN ID IN RIGHT HAND COLUMN MATCHES IDS FROM LEFT THEN A FULL SCREEN IMAGE HAS BEEN ACHIEVED
            # NOW COUNT ALL BLOBS AND CREATE NEW UNIQUE IDS FOR LEFT-HAND BLOBS
            
            # Creating an empty dictionary (y-co-ordinate, area, count)
            keyblobtracker = {}
            # Adding list as value
            keyblobtracker['batch'] = [0,0,0]   # blob_area, blob_height_from_bottom, grape-count
            # set batch number to zero before counting starts
            batch = 0
            # print to confirm tracker is reset
            # print(keyblobtracker)            
            x1, y1 = cnt[0,0]
            # get centre-x and centre-y positions
            if M['m00']>0:   # to prevent division by zero
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                # c_coordinates = (cx, cy)
                print('Area of grape blob', cv2.contourArea(cnt), '(X,Y COORDINATES: (',x1,',',y1,')')
                # ASSIGN KEY BLOB
                # if blob appears in the left 20 pixels of the image
                if x1<20:
                    # draw a red contour around key blob    
                    img1 = cv2.drawContours(img, [cnt], -1, (0,100,255), 1)
                    # identify key blob on image, print height from bottom of the image
                    cv2.putText(img1, f'KEY:{y1}', (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                    # if this is the first batch, count the grapes in view without tracking
                    # increment batch (will become 1 at start of counting)
                    batch +=1
                    if batch==1:
                        keyblobtracker[batch] = [cv2.contourArea(cnt),y1,len(contours)]
                        print("FIRST COUNT:",keyblobtracker) 
                    # TEST IF TRACKER BLOB APPEARS ON RHS OF IMAGE
                               


        cv2.imshow("Grape Bunch Count", img)
        cv2.waitKey(1)
        # cv2.destroyAllWindows()

        ############################################################################
        # THIS SECTION IS THE ALGORITHM THAT ATTEMPTS TO TRACK INDIVIDUAL BUNCHES
        # 
        # ADD BUNCHES TO LIST WITH X, Y & Z INFORMATION - COUNTING ALGORITHM    
        ############################################################################
        # WHAT WILL BE ATTEMPTED IS TO CREATE A KEY BASED ON THE APPROXIMATION OF THE 
        # CONTOUR VALUES.
        # BY STORING THE VALUES OF SELECTED CONTOURS ON THE LHS AND ASSIGNING A KEY 
        # THIS WILL BE COMPARED WITH CONTOURS APPEARING ON THE RHS AND USING THE 
        # math.isclose(a, b, *, rel_tol=1e-09, abs_tol=0.0) FUNCTION TO SEE IF THE CONTOURS
        # MATCH.  IF THEY DO, THEN THE GRAPE BUNCH WILL HAVE TRAVELED THE FULL DISTANCE 
        # ACROSS THE IMAGE - AT WHICH POINT A GRAPE BUNCH COUNT WILL TAKE PLACE
        #  
        #  KEY = IMAGE = 0
        # IF ROBOT IS READY TO ADVANCE TO WAYPOINT 2 OR WAYPOINT 4
        # WRITE KEY BLOB TO X = 0 ON IMAGE
        # WAIT WHILE ROBOT ADVANCES
        # IF KEY BLOB DETECTED ON RHS OF IMAGE (X = 940)
            # ADD CONTOUR COUNT TO DICTIONARY {IMAGE[X],[COUNT[KEY]}
            # KEY = KEY + 1
            # X = X + 1
        # WRITE IMAGE BLOB[KEY] 

        # RESTART COUNTING WHEN ROBOT REACHES WAYPOINT 4 AND COMPLETED ORIENTATION TOWARDS WAYPOINT 5 [END]
        # APPLY ADJUSTMENT FACTOR TO COMPENSATE FOR DOUBLE-COUNTING
        # COMPENSATION FACTOR COULD BE: (MANUAL GRAPE COUNT - ROBOT GRAPE COUNT) * ROBOT GRAPE COUNT
        # PRINT RESULT



#startWindowThread()
rospy.init_node('image_converter')
ic = image_converter()
rospy.spin()

destroyAllWindows()

