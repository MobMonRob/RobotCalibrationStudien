import cv2

if __name__ == "__main__":
    print("Running")

    mirror = True
    width = 600
    height = 400
    source = 0
    cap = cv2.VideoCapture(source)
    
    path = "Test\Test11.png"

    while True:
        #ret, frame = cap.read()

        frame = cv2.imread(path)

        if mirror: 
            frame = cv2.flip(frame, 1)

        blue, green, red = cv2.split(frame)

        redFrame = frame.copy()
        # set green and red channels to 0 (BGR Format)
        redFrame[:, :, 0] = 0
        redFrame[:, :, 1] = 0
        
        gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)

        cv2.imshow('Gray', gray)
        cv2.imshow('Red color', redFrame)
        cv2.imshow('Red gray', red)
        cv2.imshow('Blue gray', blue)
        #cv2.namedWindow('my webcam',cv2.WINDOW_FULLSCREEN)
        #cv2.resizeWindow('my webcam', width, height)
        if cv2.waitKey(1) == 27: 
            break  # esc to quit
    cv2.destroyAllWindows()