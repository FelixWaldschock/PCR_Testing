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

def nMeasuresTimed(n, deltatms, SensorNr):
    #Start the timer
    
    tstart = time.perf_counter_ns()
    ttot = tstart

    for i in range(n):
        #Measure
        sensors[SensorNr].readSensorValue(readADC(ADC, sensors[SensorNr].pin))
        #Wait in the loop until time is elapsed
        while (True):
            tend = time.perf_counter_ns()
            deltat = tend-tstart
            if (deltat >= deltatms*10e5):
                #print("t", i, ":", deltat)
                break
            #print("tend-tstart", tend-tstart)   #Test print to see how long it takes

        #Restart timer
        tstart = time.perf_counter_ns()
        #print("break", i)  #Test print to see how long it takes
    #Get the average
    value = sensors[SensorNr].getValue()
    print("Total time:", time.perf_counter_ns()-ttot) #Test print for total
    return value

ADC = initADC()
t1 = time.perf_counter_ns()
sensors[1].readSensorValue(readADC(ADC, sensors[1].pin))
t2 = time.perf_counter_ns()

print("measure time:", t2-t1)

nMeasuresTimed(20, 21, 1)