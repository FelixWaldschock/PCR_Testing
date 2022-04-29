from readline import set_completion_display_matches_hook
import sys
from tkinter import Button
import push2DB as p2db
import readSensor as rS
import sensor
import actuator
from datetime import datetime, timedelta
import random 
import RPi.GPIO as GPIO
import time
from ads1015 import ADS1015
import controller

# general parameters
numberOfCycles = 30

# create sensor objects
Temperature1 = sensor.Sensor("PT1000", ['in0/in1'],3.3)
Temperature2 = sensor.Sensor("PT1000", ['in2/ref'],3.3)
Photodiode = sensor.Sensor("Photodiode", ['in3/ref'],3.3)

# create actor objects
Peltier = actuator.Actuator("Peltierelement", 12)

# buttons
buttonPin = 13
buttonState = False

sensors = [Temperature1]
actuators = [Peltier]
threads = []

TempTol = 1

pwms = []

# FUNCTIONS --------------------------
def reachTemp(tT): #tT targetTemperature
    while ((abs(Temp.getvalue()-tT)<TempTol)==False):  
        if (Temp.getvalue()>tT):
            controller.cool()
        elif(Temp.getvalue()<tT):
            controller.heat()
    controller.hold()
    return True

def thermoCycling():
    cycleCounter += 1
    startTime = datetime.now()
    cycleTiming = datetime.now()
    target = 57 
    
    if (reachTemp(57)):
        if (datetime.now() < (cycleTiming + timedelta(sec=8))):
            cycleTiming = datetime.now()
            if (reachTemp(94)):
                if (datetime.now() < (cycleTiming + timedelta(sec=8))):
                    reachTemp(57)
    return True

def measureData():
    for sensor in sensors:
        sensor.readSensorValue(readADC(ADC, sensor.pin))
        sensor.mapValue()
    return
    
def createMeasurementDict():
    
    #MeasurementDict = {
    #"MeasurementNumber": #getNumberOfMeasurement,
    #"tempCase": #readTempSen1,
    #"tempProbe": #readTempSen2,
    #"CT-value": #readPhotoDiode,
    #"Fan1Speed": #getPWMSignalFan1,
    #"Fan2Speed": #getPWMSignalFan2
    #}
    
    MeasurementDict = {
    "MeasurementNumber": 1,
    "tempCase": Temperature1.getValue(),
    "tempProbe": Temperature2.getValue(),
    "CT-value": Photodiode.getValue(),
    "Peltier DutyCycle": Peltier.getDutyCycle()
    }

    #print(MeasurementDict)
    return MeasurementDict

def send2DB():
    p2db.send2DB(createMeasurementDict())
    return

def initPWMsignals():
    GPIO.setmode(GPIO.BOARD)
    for i in actuators:
        print(i.pin)
        GPIO.setup(i.pin, GPIO.OUT)
        pwm = GPIO.PWM(i.pin, 100)
        pwm.ChangeDutyCycle(100)
        i.pushPWM(pwm)

def initADC():
    ads1015 = ADS1015()
    chip_type = ads1015.detect_chip_type()
    print("Found: {}".format(chip_type))

    ads1015.set_mode('single')
    ads1015.set_programmable_gain(0.256)

    if chip_type == 'ADS1015':
        ads1015.set_sample_rate(1600)
    else:
        ads1015.set_sample_rate(860)

    ADC = ads1015
    print(ADC.get_reference_voltage())
    return [ADC,ADC.get_reference_voltage()]

def readADC(chip, inputPort):
    v = chip[0].get_voltage(channel=inputPort[0])
    print("Difference Voltage" + str(v) +" "+str(inputPort[0]))
    return v

def initButtons():
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(buttonPin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

def checkButtons():
    buttonPrev = buttonState
    buttonState = GPIO.input(buttonPin)
    if (buttonPrev != buttonState):
        return True    
    else:
        return False

def initThreads():
    # thread for thermocycling
    # thread for measuring
    # thread for sending 
    global threads
    tC = threading.Thread(target=thermoCycling)
    meas = threading.Thread(target=measureData)
    sending = threading.Thread(target=send2DB)
    threads = [tC, meas, sending]
    return

def loop():
    SysStatus = True
    while(start == 1):
        if (1):
            while(cycleCounter < 30):
                thermoCycling()
        if (timestamp + timedelta(seconds=2) < datetime.now()):
            p2db.send2DB(createMeasurementDict())
        Peltier.changeDutyCycle(100.0)
        
        time.sleep(0.5)

def startProcess():
    SysStatus = True
    #start all treads
    for t in threads:
        t.start()
    return

def stopProcess():
    #stop all threads
    for t in threads:
        t.join()
    #stop all pwm signals
    stopPWMs()
    return

def stopPWMs():
    for i in actuators:
        i.pwm.stop()
        print("All PWM signals cleared")
    return


# Initiation --------------------------

initPWMsignals()
ADC = initADC()
cycleCounter = 0
SysStatus = False

# -----------------

try:
    while(cycleCounter < numberOfCycles):
        if (checkButtons()):
            if(SysStatus == True):
                stopProcess()
            else:
                startProcess()

        
    stopProcess()
    print("Process done")

except KeyboardInterrupt:
    print("Ctl C pressed - ending program")
    stopPWMs()
    GPIO.cleanup()                     # resets GPIO ports used back to input mode
    print("Cleanup done")
