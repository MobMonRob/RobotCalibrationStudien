from telnetlib import NOP
from urllib.parse import uses_params
import cv2
import numpy as np
import poly_point_isect as bot

def GetCenterPoint(img):

    blured = __GetBluredImage(img)
    points = []
    sumX, sumY = 0, 0

    low_threshold = 50
    high_threshold = 150
    edges = cv2.Canny(blured, low_threshold, high_threshold)

    rho = 1  # distance resolution in pixels of the Hough grid
    theta = np.pi / 180  # angular resolution in radians of the Hough grid
    threshold = 15  # minimum number of votes (intersections in Hough grid cell)
    min_line_length = 50  # minimum number of pixels making up a line
    max_line_gap = 20  # maximum gap in pixels between connectable line segments
    line_image = np.copy(img) * 0  # creating a blank to draw lines on

    # Run Hough on edge detected image
    # Output "lines" is an array containing endpoints of detected line segments
    lines = cv2.HoughLinesP(edges, rho, theta, threshold, np.array([]), min_line_length, max_line_gap)

    for line in lines:
        for x1,y1,x2,y2 in line:        
            points.append(((x1, y1), (x2, y2)))
            cv2.line(line_image, (x1, y1), (x2, y2), (255, 0, 0), 5)

    img = cv2.addWeighted(img, 0.8, line_image, 1, 0)

    intersections = bot.isect_segments(points)

    for inter in intersections:
        a, b = inter
        sumX += a
        sumY += b

    x = round(sumX / len(intersections))
    y = round(sumY / len(intersections))

    for i in range(-2,3):
        for j in range(-2,3):      
            img[y + i, x + j] = [0, 255, 0]            
    
    angle = __CalculateAngle(points)

    return x, y, img, angle

def __CalculateAngle(points):    
    angles= []
    sumAngles = 0
    usedAngels = 0 

    for point in points:
        first, second = point
        angles.append(np.rad2deg(np.arctan2(first[1] - second[1], first[0] - second[0])))

    for angle in angles:
        if angle > 90 and angle < 180:
            sumAngles += 180 - angle
            usedAngels += 1
            continue
        if angle > -180 and angle < -90:
            sumAngles += -(90 + angle)
            usedAngels += 1
            continue

    if usedAngels == 0 and sumAngles == 0: 
        return 0
    return sumAngles / usedAngels

def __GetBluredImage(img):
    img = __GetGrayImage(img)
    kernel_size = 5
    return cv2.GaussianBlur(img,(kernel_size, kernel_size),0)

def __GetGrayImage(img):
    return cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)