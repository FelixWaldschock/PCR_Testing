import datetime
from ads1015 import ADS1015
import time
import sys
import os
#import from parent dir

current = os.path.dirname(os.path.realpath(__file__))
parent = os.path.dirname(current)
sys.path.append(parent)

import send2csv

#channels = ['in0/in1','in0/gnd','in1/gnd','in2/gnd','in3/gnd']
channels = ['in0/in1']

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
    print(ADC.get_reference_voltage())
    return [ADC,ADC.get_reference_voltage()]

def readADC(chip, inputPort):
    v = chip[0].get_voltage(inputPort)
    #v = chip[0].get_compensated_voltage(channel=inputPort[0], vdiv_a = 8060000, vdiv_b=402000, reference_voltage=chip[1])
    return v

ADC = initADC()
tiS = datetime.datetime.now()
td = datetime.timedelta(seconds=ts)
print(ts)
a = []
for i in range(200):
    start = time.time()
    r = (readADC(ADC, channels[0]))
    end = time.time()
    a.append(r)
    
    time.sleep(1/50)
    #for c in channels:
    
        #print("Channel " + str(c) + " "+str(readADC(ADC,c)))
print(end-start)
#print(a)
send2csv.send2csv("Measurement1.csv", a)

for i in a:
    ff = "{:.5f}".format(i)
    print(ff)
print(sum(a)/len(a))
