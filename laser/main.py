from turtle import distance
from BaslerCamera import Camera
import Transform
import ShapeDetection
import LineDetection
import cv2
import math
import time

# Size of Transformed Image
# one Pixel equals 1mm
w = 1280
h = 562

distanceToWallInmm = 1000   # => 1m

class LaserCalibration():

    def __init__(self):
        #self.camera = Camera()
        self.CenterDistanceToWall = self.__CalculateCenterDistanceToWallInPixel()

    def GetPoint(self):
        #img = self.camera.GetImage()  
        img = cv2.imread("D:\\repos\RobotCalibrationStudien\laser\Test\Shape2.png")      
        shapeImg = img.copy()
        
        pts, shapeImg = ShapeDetection.GetCenterOfCountours(shapeImg)        
        transformImg = Transform.TransformImage(img, pts, w, h)
        x, y, centerImg, angle = LineDetection.GetCenterPoint(transformImg)
        return [x, y], angle

    def __CalculateCenterDistanceToWallInPixel(self):
        return math.sqrt(distanceToWallInmm ** 2 - (w/2) ** 2)

    def CalculateLengthOnWallInPixel(self, angle):
        return round(math.tan(angle * math.pi / 180) * self.CenterDistanceToWall)


if __name__ == "__main__":    

    laser = LaserCalibration()

    angleToMove = 20
    targetDistance = laser.CalculateLengthOnWallInPixel(angleToMove)

    #move to first Point
    first, firstAngle = laser.GetPoint()

    #move to second point
    second, secondAngle = laser.GetPoint()

    distanceX = abs(first[0] - second[0])
    distanceY = abs(first[1] - second[1])

    if distanceX == 0:
        actualDistance = distanceY
    
    if distanceY == 0:
        actualDistance = distanceX

    if distanceX != 0 and distanceY !=0:
        actualDistance = round(math.sqrt(distanceX ** 2 + distanceY ** 2))


    print("Target distance: {}, measured distance: {}".format(targetDistance, actualDistance))