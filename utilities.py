import cv2 
import time
from adafruit_motorkit import MotorKit
import numpy as np

class Movement:
    def __init__(self):
        self.a=[]
    def slowthrottle(frames): #finds throttle value to hold slow speed based on cameras
        #returns propper throttle amount
        t=[]
        return t
    def stepleft(car):
        car.motor2.throttle = 1
        time.sleep(0.1)
        car.motor1.throttle = -1
        time.sleep(0.1)
        car.motor1.throttle = 1
        time.sleep(0.05)    #reverse just for a sec
        car.motor1.throttle = 0
        time.sleep(0.1)
        car.motor2.throttle = 0
        time.sleep(0.1)
        car.motor2.throttle = -1
        time.sleep(0.1)
        car.motor1.throttle = -1
        time.sleep(0.1)
        car.motor1.throttle = 1
        time.sleep(0.05)    #reverse just for a sec
        car.motor1.throttle = 0
        time.sleep(0.1)
        car.motor2.throttle = 0
        time.sleep(0.1)
        car.motor1.throttle = 1 #correct for movement backwards which happened
        time.sleep(0.15)
        car.motor1.throttle = -1
        time.sleep(0.05)
        return
    def stepright(car):
        car.motor2.throttle = -1
        time.sleep(0.1)
        car.motor1.throttle = -1
        time.sleep(0.1)
        car.motor1.throttle = 1
        time.sleep(0.05)    #reverse just for a sec
        car.motor1.throttle = 0
        time.sleep(0.1)
        car.motor2.throttle = 0
        time.sleep(0.1)
        car.motor2.throttle = 1
        time.sleep(0.1)
        car.motor1.throttle = -1
        time.sleep(0.1)
        car.motor1.throttle = 1
        time.sleep(0.05)    #reverse just for a sec
        car.motor1.throttle = 0
        time.sleep(0.1)
        car.motor2.throttle = 0
        time.sleep(0.1)
        car.motor1.throttle = 1 #correct for movement backwards which happened
        time.sleep(0.15)
        car.motor1.throttle = -1
        time.sleep(0.05)
        return
    def fwbcheckdecenter():
        #throttle forward some amount
        #throttle back some amount
        #check how much target point center moves left/right
        error=[]
        return error
    def steprightsmart():#keeps track of camera movement to decide how much to move second time
        return []
    def stepleftsmart():    #keeps track of camera movement to decide how much to move second time
        return []


class Calibrate:    #calibrate stores object with calibration results
    def __init__(self):
        #stores x,y,z angles (from target facing sensor)
        #for one camera, this will be 0
        self.pos=np.array([0,0,0])
        #camera x,y,z position
        self.ang=np.array([0,0,0])
        #stores MTF value in [x,y,data] where data is [spatial frequency, contrast] points
        MTF=np.array([[],[],[]])    
    
    def activeMTF():    #actively aligns to MTF target, stores results in object
        return []
    def saveresults(path):  #saves calibration results to file for retrieval, returns path
        return[]

class Process:  #stores image processing tools
    def __init__(self):
        self.x=[]
    def blur(im,sz):    #blurs with image and gaussian kernel of with sz
        #create kernel
        #need to multiply by size for magnitude
        kernel = np.ones((sz,sz),np.float32)*(1/(sz**2))    #kernel of ones
        #kernel = cv2.getGaussianKernel(1,1,1)
        image = cv2.filter2D(im,-1,kernel)  #applies filter
        return image

    def sharpen(im,times):
        #hardcoded for now
        image = im
        kernel = np.array([[-1,-1,-1], 
                        [-1, 9,-1],
                        [-1,-1,-1]])
        for x in range(1,times):   #applies the number of times of sz
            image = cv2.filter2D(im,-1,kernel)
        return image
        #create kernel

    def generatefilters(num,min,max):
        filters = np.linspace(min,max,num)
        ln = np.array(np.zeros(num))   #array the size of the # filters we need
        for x in range(min,max):
            #sz = 
            ln[x] = np.ones((sz,sz),np.float32)*(1/(sz**2))    #kernel of ones


    def varysharpness(im,ln):    #ln is linearly distributed list of kernel sizes, full diagonal use max
        image = im
        lenx, leny = len(image[1,:]),len(image[1,:])
        max = np.sqrt(lenx**2+leny**2)
        for x in range (1,len(im[1,:])):
            for y in range (1,len(im[1,:])):
                filter = ln[round(np.sqrt(x**2+y**2))/max] #finds what index of filter to use
                image += np.dot(image,filter)   #need to figure out how to do this in the right location
        return image




