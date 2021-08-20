from adafruit_motorkit import MotorKit
import time
import cv2
import io
import numpy as np

def main():
    kit = MotorKit(address=0x40)
    cam = cv2.VideoCapture(0)
    before = time.time()
    while time.time() < (before+240):
        # Capture frame-by-frame
        ret, frame = cam.read()
        if not ret:
            print("Can't receive frame (stream end?). Exiting ...")
            break
        grey = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        cv2.imshow('grey',grey)
        #check if lights are on; perform action for 0.1s accordingly.
        if ison(grey) == True:
            kit.motor1.throttle=0
            time.sleep(0.1)
        else:
            kit.motor1.throttle = 1 #1 is reverse, -1 is forwards
            time.sleep(0.1)
    kit.motor1.throttle=0   #after loop, make sure it turns off

def ison(image):    #returns true if lights in image are on, false if not
    #basically just take average grey value
    avg = np.average(np.average(image))    #average grey value
    if avg > 90:
        print(avg)
        return True
    if avg < 90:
        print(avg)
        return False

#maybe add function to plot grey values over time?
    
main()