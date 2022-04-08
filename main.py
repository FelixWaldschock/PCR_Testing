import push2DB as p2db
import readSensor as rS
from datetime import datetime

#Initialise empty arrays for sensor values




def startNewMeasurement():
    #Create new number
    #start warming up and first cycle
    
def thermoCycling():
    #hold on 57°C for 8secs
    #heat up to 94°C
    #hold for 8secs
    #cool down to 57°C
    #start over

def createMeasurementDict():
    MeasurementDict = {
    "MeasurementNumber": #getNumberOfMeasurement,
    "tempCase": #readTempSen1,
    "tempProbe": #readTempSen2,
    "CT-value": #readPhotoDiode,
    "Fan1Speed": #getPWMSignalFan1,
    "Fan2Speed": #getPWMSignalFan2
    }
    return MeasurementDict


p2db.send2DB(MeasurementDict)
start = 1

while(start == 1):

