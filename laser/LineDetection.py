from telnetlib import NOP
import cv2
import numpy as np
import poly_point_isect as bot

def GetCenterPoint(img, blured):

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

    lines_edges = cv2.addWeighted(img, 0.8, line_image, 1, 0)

    intersections = bot.isect_segments(points)

    for inter in intersections:
        a, b = inter
        sumX += a
        sumY += b

    x = round(sumX / len(intersections))
    y = round(sumY / len(intersections))

    for i in range(-2,3):
        for j in range(-2,3):      
            lines_edges[y + i, x + j] = [0, 255, 0]            
    
    return x, y, lines_edges


def GetBluredImage(img):
    kernel_size = 5
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    return cv2.GaussianBlur(gray,(kernel_size, kernel_size),0)

if __name__ == "__main__":

    path = "Test\Test6.png"

    img = cv2.imread(path)
    h, w, c = img.shape

    

    cv2.imshow("Blured", blur_gray)

    x, y, lines_edges = GetCenterPoint(img, blur_gray)

    print (str(x))
    print (str(y))
    cv2.imshow("Result", lines_edges)

    if cv2.waitKey(0) == 27: 
        cv2.destroyAllWindows()