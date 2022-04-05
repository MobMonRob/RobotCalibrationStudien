import cv2
import numpy as np
import LineDetection as detection




def TransformImage(img, pts1, w, h):
    pts2 = np.float32([[0,0],[w,0],[0,h],[w,h]])
    matrix = cv2.getPerspectiveTransform(pts1, pts2)
    output = cv2.warpPerspective(img, matrix,(w, h))

    for i in range(0,4):
        cv2.circle(img, (pts1[i][0],pts1[i][1]), 5, (0,255,255),cv2.FILLED)
    return img, output

if __name__ == "__main__":
    path = "Test\Test11.png"

    img = cv2.imread(path)
    h, w, c = img.shape
    w,h = 1000,500
    
    pts1 = np.float32([[138,131],[644,92],[138,410],[644,463]])

    imgWithCyrcle, output = TransformImage(img, pts1, w, h)

    blured = detection.GetBluredImage(output)
    x, y, lines = detection.GetCenterPoint(output, blured)

    cv2.imshow("Original", imgWithCyrcle)
    cv2.imshow("Transformed", output)
    cv2.imshow("Lines", lines)
    cv2.waitKey(0)