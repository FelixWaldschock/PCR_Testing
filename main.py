import push2DB as p2db
import readSensor as rS
import sensor
from datetime import datetime
import random 

# create sensor objects
Temperature1 = sensor.Sensor("Temperatur", 12)
Temperature2 = sensor.Sensor("Temperatur", 13)
Photodiode = sensor.Sensor("Photodiode", 18)

sensors = [Temperature1,Temperature2,Photodiode]

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
    
    #MeasurementDict = {
    #"MeasurementNumber": 1,
    #"tempCase": Temperature1.getValue,
    #"tempProbe": Temperature2.getValue,
    #"CT-value": Photodiode.getValue
    #
    #}

    MeasurementDict = {
    "MeasurementNumber": 1,
    "tempCase": random.random()*100,
    "tempProbe": random.random()*100,
    "CT-value": random.random()*100
    
    }
    return MeasurementDict



start = 1

while(start == 1):
    for sensor in sensors:
        sensor.readSensorValue()
    p2db.send2DB(createMeasurementDict())