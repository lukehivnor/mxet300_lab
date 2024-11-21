import L2_compass_heading
import L1_log
import time
import board
import digitalio
import netifaces as ni
from time import sleep
from PIL import Image, ImageDraw, ImageFont
import adafruit_ssd1306
import L1_ina
import L1_log





def get_it():
    head=L2_compass_heading.get_heading()
    L1_log.stringTmpFile(str(round(head,2)), 'direction.txt')
    # heading = str(head)
    return head 


def direct():
    direction = get_it()
    round(direction,2)
    # range if 
    d = str('0')
    if (-30< direction<30):
        d = 'north'
    if (30< direction<60):
        d = 'northeast'
    if (60< direction<90):
        d = 'east'
    if (90< direction<135):
        d = 'southeast'
    if (135< direction<-160):
        d = 'south'
    if (0< direction<-45):
        d = 'northwest'
    if (-45< direction<-135):
        d = 'west'
    if (-135< direction<-160):
        d = 'southwest'
   
    f = L1_log.stringTmpFile(d, 'cardinal.txt')
    return f



if __name__ == "__main__":
    while True:
        print(round(L2_compass_heading.get_heading(),2)) 
                  # Print the compass heading in degrees
        get_it()
        direct()
        volt = L1_ina.readVolts()

        L1_log.tmpFile(volt, "rahbit_volts.txt")
        time.sleep(0.1)
