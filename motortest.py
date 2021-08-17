import time
from robot import *
import os

def test():
    #set permissions for pins!
    os.system('sudo busybox devmem 0x700031fc 32 0x45')
    os.system('sudo busybox devmem 0x6000d504 32 0x2')
    os.system('sudo busybox devmem 0x70003248 32 0x46')
    os.system('sudo busybox devmem 0x6000d100 32 0x00')
 
    r = Robot()
    r.left(speed=0.001)
    r.right(speed=0.1)
    time.sleep(1)
    r.stop()
test()