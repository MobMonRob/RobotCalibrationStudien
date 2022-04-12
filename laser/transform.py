import cv2
import numpy as np
import LineDetection as detection
from BaslerCamera import Camera


def TransformImage(img, pts1, w, h):
    pts2 = np.float32([[0,0],[w,0],[0,h],[w,h]])
    matrix = cv2.getPerspectiveTransform(pts1, pts2)
    return cv2.warpPerspective(img, matrix,(w, h))


if __name__ == "__main__":

    path = "D:\\repos\RobotCalibrationStudien\laser\Test\Test11.png"

    img = cv2.imread(path)
    #camera = Camera()
    #img = camera.GetImage()
    h, w, c = img.shape
    #w,h = 1000,500
    
    pts1 = np.float32([[138,131],[644,92],[138,410],[644,463]])

    output = TransformImage(img, pts1, w, h)
    
    for i in range(0,4):
        cv2.circle(img, (pts1[i][0],pts1[i][1]), 5, (0,255,255),cv2.FILLED)
    #blue, green, red = cv2.split(img)
    blured = detection.GetBluredImage(img)
    #cv2.imshow("img", red)
    x, y, lines = detection.GetCenterPoint(img, blured)

    cv2.imshow("Original", img)
    cv2.imshow("Transformed", output)
    cv2.imshow("Lines", lines)
    cv2.waitKey(0)