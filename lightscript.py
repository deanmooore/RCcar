from typing import ChainMap
from adafruit_motorkit import MotorKit
import time
import cv2
import io
import numpy as np

def main():

    cam = cv2.VideoCapture(0)
    r = Robot()
    before = time.time()
    while time.time() < (before+60):
        # Capture frame-by-frame
        ret, nframe = cam.read()
        if not ret:
            print("Can't receive frame (stream end?). Exiting ...")
            break
        grey = cv2.cvtColor(cam, cv2.RGB2GRAY)
        while time.time() < (before+60): #perform loop for 60s
            #check if lights are on; perform action for 0.1s accordingly.
            if ison(grey) == True:
                time.sleep(0.1)
            else:
                r.forward(0.1)
                time.sleep(0.1)
                r.stop()

def ison(image):    #returns true if lights in image are on, false if not
    #basically just take average grey value
    avg = np.average(np.average(image))    #average grey value
    if avg > 120:
        return True
    if avg < 120:
        return False

#maybe add function to plot grey values over time?
    
main()