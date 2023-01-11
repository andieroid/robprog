import cv2
img = cv2.imread('blobs.png')
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
ret,thresh = cv2.threshold(gray,170,255,0)
contours,hierarchy = cv2.findContours(thresh, cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
print("Number of Contours detected:",len(contours))
for i, cnt in enumerate(contours):
   M = cv2.moments(cnt)
   print(f"Moments of Contour {i+1}:\n", M)
   x1, y1 = cnt[0,0]
   img1 = cv2.drawContours(img, [cnt], -1, (0,255,255), 3)
   cv2.putText(img1, f'Contour:{i+1}', (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
cv2.imshow("Contours", img)
cv2.waitKey(0)
cv2.destroyAllWindows()