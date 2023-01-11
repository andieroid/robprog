# INSPIRED BY https://stackoverflow.com/questions/10948589/choosing-the-correct-upper-and-lower-hsv-boundaries-for-color-detection-withcv

import cv2
import numpy as np

img = cv2.imread('images/grapes_LHS.png')

# GRAPE_MIN = np.array([82, 50, 50],np.uint8)
# GRAPE_MAX = np.array([100, 255, 255],np.uint8)
# 210, 21, 18

hsv_img = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)

frame_threshed = cv2.inRange(hsv_img, GRAPE_MIN, GRAPE_MAX)
cv2.imwrite('images/grapes_output.png', frame_threshed)



# 45, 53, 58