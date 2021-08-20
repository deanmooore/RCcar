import utilities
from adafruit_motorkit import MotorKit

def main():
    r=MotorKit(address=0x40)
    t=0.5
    r=utilities.Movement.stepleft(r,t)
    r=utilities.Movement.stepleft(r,t)
    r=utilities.Movement.stepleft(r,t)
    r=utilities.Movement.stepleft(r,t)
main()