import cv2
import numpy as np

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
        
        #denoise with universal kernel application
        #later on, try doing varied kernel
        nframe = denoise(nframe,8)
        wframe = denoise(wframe,8)

        # Display the resulting frame
        cv2.imshow('nframe',nframe)
        cv2.imshow('wframe',wframe)
        if cv2.waitKey(1) == ord('q'):
            break
    # When everything done, release the capture
    ncam.release()
    wcam.release()
    cv2.destroyAllWindows()

def denoise(image,sz):
    xy = np.arange(81).reshape(9,9)
    #this kernel isn't good, replace with gaussian after check
    for k in range(len(xy)):
        xy[k]=.2*abs(xy[k]-sz/2)**2
    bimage=cv2.filter2D(image,1,xy)
    return bimage

opencamstream()
