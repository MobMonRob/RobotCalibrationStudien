# import the necessary packages
import imutils
import cv2
import numpy as np

def GetGrayImage(image):
    h, w, c = image.shape
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    if cv2.countNonZero(gray) > ((w*h)//2):
        # More White than Black, so invert the color
        gray = cv2.bitwise_not(gray)
    
    return gray

def GetCenterOfCountours(image):
    h, w, c = image.shape

    # find contours in the thresholded image
    thresh = cv2.threshold(image, 60, 255, cv2.THRESH_BINARY)[1]
    cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)

    centers = []

    # loop over the contours
    for c in cnts:
        M = cv2.moments(c)    
        if not M["m00"] == 0.0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])

            #add centerpoints to centers
            center = [cX, cY]
            centers.append(center)

            # draw the contour and center of the shape on the image
            cv2.drawContours(image, [c], -1, (0, 255, 255), 1)
            cv2.circle(image, (cX, cY), 1, (255, 255, 255), -1)

    if len(centers) > 5:
        print ("Mehr als 4 Marker erkannt. Kalibrierung kann nicht durchgeführt werden.")
    else:
        #check wich point is the left, the right, the top and he bottom one
        for center in centers:
            if center[0] < w / 4 and center[1] < h / 4:
                leftTop = center
                continue
            if center[0] > w * 3/4 and center[1] < h / 4:
                rightTop = center
                continue
            if center[0] < w / 4 and center[1] > h * 3/4:
                leftBottom = center
                continue
            if center[0] > w * 3/4 and center[1] > h * 3/4:
                rightBottom = center                
                
        pts = np.array([leftTop, leftBottom, rightBottom, rightTop], np.int32)    
        ptsReshaped = pts.reshape((-1, 1, 2))
        image = cv2.polylines(image, [ptsReshaped], True, (255,0,0), 1)

    return pts, image