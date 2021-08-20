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
    def fwfor(car,t):  #moves forward for given amount of time
        car.motor1.throttle=1
        time.sleep(t)
        car.motor1.throttle=-1
        time.sleep(0.05) #steps backwards for a sec to smooth movement
        car.motor1.throttle=0
        return car
    def bkfor(car,t):
        car.motor1.throttle=-1
        time.sleep(t)
        car.motor1.throttle=1
        time.sleep(0.05) #steps backwards for a sec to smooth movement
        car.motor1.throttle=0
        return car
    def safeleft(car):    #turns wheels left and waits a bit
        time.sleep(0.2)
        car.motor2.throttle=-1
        time.sleep(0.2)
        return car
    def safecenter(car):
        time.sleep(0.2)
        car.motor2.throttle=0
        time.sleep(0.2)
        return car
    def saferight(car):
        time.sleep(0.2)
        car.motor2.throttle=1
        time.sleep(0.2)
    def bkfor(car,t):
        car.motor1.throttle=-1
        time.sleep(t)
        car.motor1.throttle=1
        time.sleep(0.05) #steps backwards for a sec to smooth movement
        car.motor1.throttle=0
        return car
    def stepleft(car,t):    #car is a motorkit object, t is how long in each reverse
        #first: turn front wheel, reverse
        Movement.safeleft(car)
        Movement.bkfor(car,t)
        #next: straighten wheel, reverse
        Movement.safecenter(car)
        Movement.bkfor(car,t)
        #turn other wheel, reverse (straightens car)
        Movement.saferight(car)
        Movement.bkfor(car,t)
        #move forward to correct distance
        Movement.safecenter(car)
        Movement.fwfor(car,t)
        return car
    def stepright(car,t):
        Movement.saferight(car)
        Movement.bkfor(car,t)
        #next: straighten wheel, reverse
        Movement.safecenter(car)
        Movement.bkfor(car,t)
        #turn other wheel, reverse (straightens car)
        Movement.safeleft(car)
        Movement.bkfor(car,t)
        #move forward to correct distance
        Movement.safecenter(car)
        Movement.fwfor(car,t)
        return car
    def fwbcheckdecenter():
        #throttle forward some amount
        #throttle back some amount
        #check how much target point center moves left/right
        error=[]
    def fillFOV(car):  #move until target fills fov
        error = 10 #calculate error based on opencv
        max=10  #max allowable error
        t=1 #starting time for movement
        while error >= max:
            tc=t*error/10
            Movement.fwfor(car,tc)
            p1 = Calibrate.gettargetgap()
            Movement.bkfor(car,tc)
            p2 = Calibrate.gettargetgap()
            error = p2-p1
        return error
    def centertarget(car):  #move until target fills fov
        error = 10 #calculate error based on opencv
        max=10  #max allowable error
        t=1 #starting time for movement
        while error >= max:
            tc=t*error/10
            Movement.stepleft(car,tc)
            p1 = Calibrate.gettargetcenter()
            Movement.stepright(car,tc)
            p2 = Calibrate.gettargetcenter()
            error = p2-p1
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
    def gettargetcenter(): #finds center of target and returns point in [x,y]
        return [0,0]
    def gettargetgap(): #finds how much of target is missing in [x,y]
        return [0,0]
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




