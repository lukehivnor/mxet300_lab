import L1_log as log
import L1_ina
import L2_inverse_kinematics as ik
#import L2_speed_control as sc
import numpy as np
import time
from time import sleep
# import L1_motor
import L1_encoder
import L2_kinematics


def kinematic():
    arr1 = L2_kinematics.getMotion()
    return arr1
def inverse_kinematic():
    arr2 = L2_kinematics.getPdCurrent()
    return arr2

if __name__ == "__main__":
    while True:
        
        
        
        volt = L1_ina.readVolts()

        log.tmpFile(volt, "rahbit_volts.txt")

        xdot , thetadot = kinematic()
        PDL, PDR = inverse_kinematic()

        log.tmpFile(xdot, "xdot.txt")
        log.tmpFile(thetadot, "thetadot.txt")

        log.tmpFile(PDL, "phil.txt")
        log.tmpFile(PDR, "phir.txt")

        time.sleep(0.1)