class CalculateSphereMount:
    """class to calculate sphere centers of the mounting system"""

    def __init__(self):
        NOP

    def GetFixedPoints(self):
        print('float values must have format \"uuu.www\"')
        myInput = input().split()
        while(len(myInput) != 3)
            print("Passed Arguments did not match the expected amount! Try again.")
            myInput = input().split()
        return myInput

    def CalculateSphereCenters(self, xP1, yP1, zP1, xP2, yP2, zP2):
        #sphere radius equals 7mm
        #coordinates in mm:
        #169    9   25 #P5
        #9      9   25 #fixed P1
        #-71    9   25 #P4
        #9      129 25 #P3
        #9      249 25 #fixed P2

        deltaX21 = xP1 - xP2
        deltaY21 = yP1 - yP2
        deltaZ21 = zP1 - zP2

        xP3 = xP2 + 0.5 * deltaX21
        yP3 = yP2 + 0.5 * deltaY21

        #get vectors for values of the top side 
        xVectorOfOtherSide = -deltaY21
        yVectorOfOtherSide = deltaX21
        gradiantYOfOtherSide = deltaX21/deltaY21
        gradiantXOfOtherSide = 1/xVectorOfOtherSide

        if(deltaY21 > 0 and not yVectorOfOtherSide ) # long side rises then gradiantX < 0 and gradiantY > 0



        xP4 = 
        yP4 = 

        xP5 = 
        yP5 = 

        zP3 = zP2
        zP4 = zP2
        zP5 = zP2


    def GetSphereCenters(self):
        print('enter coordinates of fixed point 1 (center top of the mounting system) in format \"x y z\"')
        readP1 = False
        while(not readP1)
            try:
                fixedP1 = self.GetFixedPoints()
                xP1 =  float(fixedP1[0])
                yP1 = float(fixedP1[1])
                zP1 float(fixedP1[2])
                readP1 = true
            except:
                print("could not parse input to float values")

        print('enter coordinates of fixed point 2 (center bottom of the mounting system) in format \"x y z\"')
        readP2 = False
        while(not readP2)
            try:
                fixedP2 = self.GetFixedPoints()
                xP2 =  float(fixedP2[0])
                yP2 = float(fixedP2[1])
                zP2 float(fixedP2[2])
                readP2 = true
            except:
                print("could not parse input to float values")
            
        return CalculateSphereCenters()

if __name__ == "__main__":
    print("main entered")
    calc = CalculateSphereMount()
    calc.getSphereCenters()