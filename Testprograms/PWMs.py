import RPi.GPIO as GPIO
import sys
import os
import time



#import from parent dir
current = os.path.dirname(os.path.realpath(__file__))
parent = os.path.dirname(current)
sys.path.append(parent)

import controller as cntrl
import actuator


Peltier = actuator.Actuator("Peltierelement", 33)
FanPeltier = actuator.Actuator("FanPeltier", 32)
Heater = actuator.Actuator("Heater", 12)
actuators = [Peltier, FanPeltier, Heater]
controller = cntrl.controller(actuators)

def initPWMsignals():
    GPIO.setmode(GPIO.BOARD)
    for i in actuators:
        print(i.pin)
        GPIO.setup(i.pin, GPIO.OUT)
        pwm = GPIO.PWM(i.pin, 100)
        pwm.ChangeDutyCycle(0)
        i.pushPWM(pwm)

def stopPWMs():
    for i in actuators:
        i.pwm.stop()
        print("All PWM signals cleared")
    return

def startPWMs():
    for i in actuators:
        i.cDC(100)
        print("All PWMs set to 100")
    return

initPWMsignals()

try:
    while(True):
        controller.fan()
        

except KeyboardInterrupt:
    print("Ctl C pressed - ending program")
    stopPWMs()
    GPIO.cleanup()                     # resets GPIO ports used back to input mode
    print("Cleanup done")