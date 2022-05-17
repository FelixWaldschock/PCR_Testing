import time
from ads1015 import ADS1015
import RPi.GPIO as GPIO

import sys
import os
#import from parent dir

current = os.path.dirname(os.path.realpath(__file__))
parent = os.path.dirname(current)
sys.path.append(parent)

import sensor

# create sensor objects
Temperature1 = sensor.Sensor("PT1000", ['in0/in1'],3.3, 20)
Photodiode1 = sensor.Sensor("Photodiode", ['in2/gnd'],3.3, 20)
Photodiode2 = sensor.Sensor("Photodiode", ['in3/gnd'],3.3, 20)
sensors = [Temperature1, Photodiode1, Photodiode2]

#GPIO IN
# buttons
buttonPin = 16
buttonState = False
EndSwitchPin = 18
GPIOins = [buttonPin, EndSwitchPin]

#GPIO OUT
LED1Pin = 11
LED2Pin = 13
Fan2Pin = 15
LEDstatus1Pin = 24
LEDstatus2Pin = 26
GPIOouts = [LED1Pin, LED2Pin, Fan2Pin, LEDstatus1Pin, LEDstatus2Pin]

def initGPIOs():
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BOARD)

    #IN-------------
    
    # Toggle switch
    GPIO.setup(buttonPin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    # end switch
    GPIO.setup(EndSwitchPin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    #OUT------------
    for o in GPIOouts:
        GPIO.setup(o, GPIO.OUT)
        GPIO.output(o, False)
    
    GPIO.output(LEDstatus1Pin, True)

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
    return [ADC,ADC.get_reference_voltage()]

def readADC(chip, inputPort):
    v = chip[0].get_voltage(channel=inputPort[0])
    return v

def PhotodiodeDiffMeasure(DiodeNr):
    global ADC
    if ((DiodeNr == 1) or (DiodeNr ==2)):

        #20 Measurements
        sensors[DiodeNr].changeAverageOf(20)
        
        #Wahrscheinlich immer noch sehr inakkurat!!!

        #Turn on LED
        GPIO.output(GPIOouts[DiodeNr-1], True)

        waitNms(20) #Wait for one 50Hz net period

        #Measure 20 times, every 21 ms, so one net period + 1 ms
        value = nMeasuresTimed(20, 21, DiodeNr)
        
        waitNms(20) #Wait for one 50Hz net period

        #Turn off LED
        GPIO.output(GPIOouts[DiodeNr-1], False)

        waitNms(20) #Wait for one 50Hz net period

        #Measure 20 times, every 21 ms, so one net period + 1 ms
        nolight = nMeasuresTimed(20, 21, DiodeNr)

        waitNms(20) #Wait for one 50Hz net period

        diff = value - nolight

        #Change back to 20
        sensors[DiodeNr].changeAverageOf(20)

        return diff

# n Measurements with a time distance of deltatms, Sensor average must be set to n
def nMeasuresTimed(n, deltatms, SensorNr):
    
    #Start the timer
    tstart = time.perf_counter_ns()

    for i in range(n):
        #Measure
        sensors[SensorNr].readSensorValue(readADC(ADC, sensors[SensorNr].pin))

        #Wait in the loop until time is elapsed
        while (True):
            tend = time.perf_counter_ns()
            deltat = tend-tstart
            if (deltat >= deltatms*1e6):
                break

        #Restart timer
        tstart = time.perf_counter_ns()

    #Get the average
    value = sensors[SensorNr].getValue()
    return value

def waitNms(N):
    tstart = time.perf_counter_ns()
    while (True):
        tend = time.perf_counter_ns()
        if(tend-tstart >= N*1000000):
            return


ADC = initADC()
initGPIOs()

messwert = PhotodiodeDiffMeasure(1)

print("Diff Messwert:", messwert)