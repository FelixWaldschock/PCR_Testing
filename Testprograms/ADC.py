from mimetypes import init
from ads1015 import ADS1015

channels = ['in0/in1','in2/ref','in3/ref']


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
    #print(ADC.get_reference_voltage())
    return [ADC,ADC.get_reference_voltage()]

def readADC(chip, inputPort):
    v = chip[0].get_voltage(channel=inputPort[0])
    return v

ADC = initADC()

while(True):
    for c in channels:
        print("Channel " + str(c) + str(readADC(ADC[0], c)))
        