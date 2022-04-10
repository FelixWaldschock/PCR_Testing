import push2DB as p2db
import readSensor as rS
import sensor
import actuator
from datetime import datetime, timedelta
import random 
import RPi.GPIO as GPIO
import time

# create sensor objects
Temperature1 = sensor.Sensor("Temperatur", 12)
Temperature2 = sensor.Sensor("Temperatur", 13)
Photodiode = sensor.Sensor("Photodiode", 18)

# create actor objects
Peltier = actuator.Actuator("Peltierelement", 12)

sensors = [Temperature1,Temperature2,Photodiode]
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

start = 1
initPWMsignals()
try:
    timestamp = datetime.now()
    while(start == 1):
        for sensor in sensors:
            sensor.readSensorValue(random.random()*100*sensor.pin)
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
