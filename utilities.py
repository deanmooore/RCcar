import cv2 
import time
from adafruit_motorkit import MotorKit
import numpy as np
import matplotlib.pyplot as plt

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
        ##########think we don't need to necessarily center the car to know displacement of cams, just make sure that we know the center of each camera.############
        error=[]
    def slowforward(car,cam,t):  #moves forward based on camera output, imports car and time intended
        reference = Movement.checkpointvelocity(car,cam)    #reference for how much velocity we should expect
        before = time.time()
        frames=np.array([])
        p = 0   #how much we're powering motor
        while time.time() < (before+t):
            frames=np.array([frames,cam.read()])
            if Calibrate.velocity(car,frames) < reference*.5:   #need to calibrate this number
                p=p+0.05
                car.motor1.throttle=p
            after=time.time()
        return car
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
    def testpointvelocity(car,vid):    # quick test fw/back movement to see how much points would move, vid is cv2.VideoCapture
        frames = cv2.cvtColor(vid.read(),cv2.COLOR_RGB2GRAY)
        while Movement.bkfor(car,0.2):  #get frames while car is moving
            frames  = np.dstack(frames,vid.read())
        while Movement.fwfor(car,0.2):
            frames = np.dstack(frames, vid.read())
        return Calibrate.velocity(frames)

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
    def velocity(cam,frames): #use cv image, check scene for differences, frames is N number of frames,cam is camera checked
        v = []  #stores differences between detected image points
        for x in range(1,len(frames)):
            f1=frames[x-1]
            f2=frames[x]
            orb = cv2.ORB_create()
            # find the keypoints and descriptors with ORB
            kp1, des1 = orb.detectAndCompute(f1,None)
            kp2, des2 = orb.detectAndCompute(f2,None)
            bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
            # Match descriptors.
            matches = bf.match(des1,des2)
            # Sort them in the order of their distance.
            matches = sorted(matches, key = lambda x:x.distance)
            # Get average distance between points in matched list.
            v=np.dstack((v,np.array(matches)))    
        vel = np.average(v)
        return cam, vel
    def getframes(cam,t):
        ret, f = cam.read()
        frames = np.array(cv2.cvtColor(f,cv2.COLOR_RGB2GRAY))
        before = time.time()
        while time.time() < (before+t):
            #append list of frames with new frames
            ret, f = cam.read()
            f = cv2.cvtColor(f,cv2.COLOR_RGB2GRAY)
            frames=np.dstack((frames,f))
        return cam, frames  #returns both camera and frames
    def gettargetgap(): #finds how much of target is missing in [x,y]
        return [0,0]
    def activeMTF():    #actively aligns to MTF target, stores results in object
        return []
    def getcam(num):    #which camera number we want (1-2), in future make smart about finding these
        cam = cv2.VideoCapture(num)  #narrow fov
        if not cam.isOpened():
            print("Cannot open camera number " + str(num))
        return cam
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

def main():
    c = Calibrate.getcam(1) #get camera
    c, frames = Calibrate.getframes(c,1)    #get frames over 1 second
    c, v = Calibrate.velocity(c,frames)#calculate velocy based on frames

main()



