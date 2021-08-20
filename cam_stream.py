#script that just runs the cameras for visual check
import cv2
import numpy as np
import utilities

def opencamstream():
    ncam = cv2.VideoCapture(1)  #narrow fov
    wcam = cv2.VideoCapture(0)  #wide fov
    if not ncam.isOpened() & wcam.isOpened():
        print("Cannot open both cameras")
        exit()
    while True:
        # Capture frame-by-frame
        ret, nframe = ncam.read()
        ret, wframe = wcam.read()
        # if frame is read correctly ret is True
        if not ret:
            print("Can't receive frame (stream end?). Exiting ...")
            break
        # Our operations on the frame come here
        #color correct each camera separately
        #when this works, sub into imshow() function
        nrgb = cv2.cvtColor(nframe, cv2.COLOR_RGB2Lab)
        wrgb = cv2.cvtColor(wframe, cv2.COLOR_RGB2Lab)
        
        #sharpen with universal kernel application
        #later on, try doing varied kernel
        nframe = utilities.Process.sharpen(nframe,1)
        #wframe = utilities.Process.sharpen(wframe,1)

        #flip images
        nframe=cv2.rotate(nframe,cv2.ROTATE_90_CLOCKWISE)
        nframe=cv2.flip(nframe,1)
        wframe=cv2.flip(wframe,0)

        # Display the resulting frame
        cv2.imshow('nframe',nframe)
        cv2.imshow('wframe',wframe)
        if cv2.waitKey(1) == ord('q'):
            break
    # When everything done, release the capture
    ncam.release()
    wcam.release()
    cv2.destroyAllWindows()
opencamstream()
