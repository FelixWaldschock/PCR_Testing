import push2DB as p2db
import readSensor as rS
import sensor
import actuator
from datetime import datetime, timedelta
import random 
import RPi.GPIO as GPIO
import time
from ads1015 import ADS1015

# create sensor objects
Temperature1 = sensor.Sensor("PT1000", ['in0/in1'],3.3)
Temperature2 = sensor.Sensor("PT1000", ['in2/ref'],3.3)
Photodiode = sensor.Sensor("Photodiode", ['in3/ref'],3.3)

# create actor objects
Peltier = actuator.Actuator("Peltierelement", 12)

sensors = [Temperature1]
actuators = [Peltier]

pwms = []

def startNewMeasurement():
    #Create new number
    #start warming up and first cycle
    return 
    
def thermoCycling():
    #hold on 57°C for 8secs
    #heat up to 94°C
    #hold for 8secs
    #cool down to 57°C
    #start over
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

start = 1
initPWMsignals()
ADC = initADC()

try:
    timestamp = datetime.now()
    while(start == 1):
        for sensor in sensors:
            sensor.readSensorValue(readADC(ADC, sensor.pin))
            sensor.mapValue()
        if (timestamp + timedelta(seconds=2) < datetime.now()):
            p2db.send2DB(createMeasurementDict())
        Peltier.changeDutyCycle(100.0)
        
        time.sleep(0.5)

except KeyboardInterrupt:
    print("Ctl C pressed - ending program")
    for i in actuators:
        i.pwm.stop()
        print("All PWM signals cleared")
    GPIO.cleanup()                     # resets GPIO ports used back to input mode
    print("Cleanup done")
