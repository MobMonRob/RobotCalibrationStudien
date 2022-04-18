import cv2
import numpy as np

def TransformImage(img, pts1, w, h):
    pts2 = np.float32([[0,0],[w,0],[0,h],[w,h]])
    matrix = cv2.getPerspectiveTransform(pts1, pts2)
    return cv2.warpPerspective(img, matrix,(w, h))