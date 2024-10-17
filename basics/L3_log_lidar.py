import L1_log as log
import L1_ina
import numpy as np
import time
from time import sleep

import L2_vector






if __name__ == "__main__":
    while True:
        
        m, deg = L2_vector.getNearest()
        x, y = L2_vector.polar2cart(m, deg)
        
        volt = L1_ina.readVolts()

        log.tmpFile(volt, "rahbit_volts.txt")

        log.tmpFile(m, "m.txt")
        log.tmpFile(deg, "deg.txt")

        log.tmpFile(x, "x.txt")
        log.tmpFile(y, "y.txt")

        
        print(x,',', y, '\n')

        time.sleep(0.1)