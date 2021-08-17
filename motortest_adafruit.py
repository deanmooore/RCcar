import time
from adafruit_motorkit import MotorKit

def main():
    kit = MotorKit(address=0x40)
    kit.motor1.throttle = 1
    time.sleep(0.5)
    kit.motor1.throttle = 0
main()