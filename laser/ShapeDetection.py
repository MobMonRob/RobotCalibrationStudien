# import the necessary packages
import imutils
import cv2


path = r'D:\Nextcloud\Marcel\DHBW\Studienarbeit\Laser Bilderkennung\TestShape2.png'

image = cv2.imread(path)
h, w, c = image.shape

gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

if cv2.countNonZero(gray) > ((w*h)//2):
    # More White than Black, so invert the color
    gray = cv2.bitwise_not(gray)

blurred = cv2.GaussianBlur(gray, (5, 5), 0)
thresh = cv2.threshold(blurred, 60, 255, cv2.THRESH_BINARY)[1]

# find contours in the thresholded image
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
        #cv2.putText(image, "center", (cX - 20, cY - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)


if len(centers) > 5:
    print ("Mehr als 4 Marker erkannt. Kalibrierung kann nicht durchgeführt werden.")
else:
    #check wich point is the left, the right, the top and he bottom one
    for center in centers:
        if(center[0] < w / 4):
            left = (center[0], center[1])
            continue
        if(center[0] > w * 3/4):
            right = (center[0], center[1])
            continue
        if(center[1] < h / 4):
            top = (center[0], center[1])
            continue
        if(center[1] > h * 3/4):
            bottom = (center[0], center[1])
            
    cv2.line(image, left, right, (255,0,0), 1)
    cv2.line(image, top, bottom, (0,255,0), 1)

    #show Image
    cv2.imshow("Image", image)

    while True:
        if cv2.waitKey(1) == 27: 
            break  # esc to quit
    cv2.destroyAllWindows()