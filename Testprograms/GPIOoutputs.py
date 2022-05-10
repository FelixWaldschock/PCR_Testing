import RPi.GPIO as GPIO
from datetime import datetime, timedelta
import time
import os
import sys


#import from parent dir
current = os.path.dirname(os.path.realpath(__file__))
parent = os.path.dirname(current)
sys.path.append(parent)

import send2csv


LED1Pin = 11
LED2Pin = 13
Fan2Pin = 15
LEDstatus1Pin = 24
LEDstatus2Pin = 26
GPIOouts = [LED1Pin,LED2Pin, Fan2Pin, LEDstatus1Pin, LEDstatus2Pin]


def blinkingLED(Pin, Timestamp, freq):
    ts = 1 / freq
    GPIO.output(Pin, not GPIO.input(Pin))
    return 
    # intervall muss noch eingefügt werden

def initGPIOs():
    GPIO.setmode(GPIO.BOARD)
    #OUT------------
    for o in GPIOouts:
        GPIO.setup(o, GPIO.OUT)
        GPIO.output(o, False)
    
    GPIO.output(LEDstatus1Pin, True)

def toggleGPIO():
    global GPIOouts
    for g in GPIOouts:
        GPIO.output(g, not GPIO.input(g))
    return

def setGPIO(bool):
    global GPIOouts
    for g in GPIOouts:
        GPIO.output(g, bool)
        print("GPIO " + str(g) + " set:" + str(bool))
    return

initGPIOs()
setGPIO(False)

try:
    ts = datetime.now()
    setGPIO(True)
    time.sleep(5)
       


except KeyboardInterrupt:
    print("Ctl C pressed - ending program")
    setGPIO(False)
    GPIO.cleanup()                     # resets GPIO ports used back to input mode
    print("Cleanup done")
