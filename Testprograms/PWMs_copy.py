import RPi.GPIO as GPIO
import sys
import os
import time
import datetime
from ads1015 import ADS1015

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



channels = ['in0/in1','in0/gnd','in1/gnd','in2/gnd','in3/gnd']

f = 10
ts = 1 / f

def initADC():
    ads1015 = ADS1015()
    chip_type = ads1015.detect_chip_type()

    print("Found: {}".format(chip_type))

    ads1015.set_mode('single')
    ads1015.set_programmable_gain(1.024)
    if chip_type == 'ADS1015':
        ads1015.set_sample_rate(1600)
    else:
        ads1015.set_sample_rate(860)

    ADC = ads1015
    print("ADC initialisiert")
    return [ADC,ADC.get_reference_voltage()]

def readADC(chip, inputPort):
    v = chip[0].get_voltage(inputPort)
    #v = chip[0].get_compensated_voltage(channel=inputPort[0], vdiv_a = 8060000, vdiv_b=402000, reference_voltage=chip[1])
    return v

ADC = initADC()
tiS = datetime.datetime.now()
td = datetime.timedelta(seconds=ts)
print(ts)



initPWMsignals()





a = []

try:
    for i in range(5):
        controller.heat()
        b = []
        for i in range(500):
            r = readADC(ADC, channels[0])
            b.append(r)
            time.sleep(1/100)
        controller.hold()
        for i in range(500):
            r = readADC(ADC, channels[0])
            b.append(r)
            time.sleep(1/100)
        controller.cool()
        for i in range(500):
            r = readADC(ADC, channels[0])
            b.append(r)
            time.sleep(1/100)
        a.append(b)
    print(a)
        
except KeyboardInterrupt:
    print("Ctl C pressed - ending program")
    stopPWMs()
    GPIO.cleanup()                     # resets GPIO ports used back to input mode
    print("Cleanup done")