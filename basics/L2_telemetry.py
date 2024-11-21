import board
import digitalio
import netifaces as ni
from time import sleep
from PIL import Image, ImageDraw, ImageFont
import adafruit_ssd1306
import L1_ina
import L1_log

while(1):
    volt = L1_ina.readVolts()

    L1_log.tmpFile(volt, "rahbit_volts")

    sleep(1)
