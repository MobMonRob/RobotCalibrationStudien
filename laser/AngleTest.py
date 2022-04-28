import numpy as np
import cv2
import LineDetection

if __name__ == "__main__":

    errorX = []
    errorY = []
    errorAngle = []
    
    for i in range(0,90,5):

        path = "D:\\repos\RobotCalibrationStudien\laser\Test\AngleTest\\{}.png".format(i)
        img = cv2.imread(path)
        
        x, y, lineImg, angle = LineDetection.GetCenterPoint(img)

        errorX.append(x)
        errorY.append(y)
        errorAngle.append(abs(i - angle))

    variationX = max(errorX) - min(errorX)
    averageErrorX = np.average(errorX)

    variationY = max(errorY) - min(errorY)
    averageErrorY = np.average(errorY)    

    variationAngle = max(errorAngle) - min(errorAngle)
    averageErrorAngle = np.average(errorAngle)

    print("The Average X-Coordinate of the calculated CenterPoint is {} with an Variation of {}(min: {}, max: {}).\n".format(averageErrorX, variationX, min(errorX), max(errorX))
        + "The Average Y-Coordinate is {} with an Variation of {} (min: {}, max: {}). \n".format(averageErrorY, variationY, min(errorY), max(errorY))
        + "The Avverage Error of the Calculated Angle is {} with an Variation of {} (min: {}, max: {}).".format(averageErrorAngle, variationAngle, min(errorAngle), max(errorAngle)))