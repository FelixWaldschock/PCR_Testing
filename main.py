import push2DB as p2db
import readSensor as rS
import sensor
import actuator
from datetime import datetime, timedelta
import RPi.GPIO as GPIO
from ads1015 import ADS1015
import controller
import threading

# general parameters
numberOfCycles = 30

# create sensor objects
Temperature1 = sensor.Sensor("PT1000", ['in0/in1'],3.3)
Photodiode1 = sensor.Sensor("Photodiode1", ['in2/ref'],3.3)
Photodiode2 = sensor.Sensor("Photodiode2", ['in3/ref'],3.3)


# create actor objects
Peltier = actuator.Actuator("Peltierelement", 12)
Fan = actuator.Actuator("Fan", 32)
LED = actuator.Actuator("LED", 33)
# https://duckduckgo.com/?q=raspberry+pi+pin+layout&t=brave&iax=images&ia=images&iai=https%3A%2F%2Ffossbytes.com%2Fwp-content%2Fuploads%2F2021%2F04%2Fgpio-pins-raspberry-pi-4-e1617866771594.png


# buttons
buttonPin = 13
buttonState = False

sensors = [Temperature1, Photodiode1, Photodiode2]
actuators = [Peltier, Fan, LED]
threads = []

TempTol = 1

pwms = []

# FUNCTIONS --------------------------
def reachTemp(tT): #tT targetTemperature
    while ((abs(Temperature1.getvalue()-tT)<TempTol)==False):  
        if (Temperature1.getvalue()>tT):
            controller.cool()
        elif(Temperature1.getvalue()<tT):
            controller.heat()
    controller.hold()
    return True

def thermoCycling():
    global cycleCounter
    cycleCounter += 1
    cycleTiming = datetime.now()
    if (reachTemp(57)):
        if (datetime.now() < (cycleTiming + timedelta(sec=8))):
            cycleTiming = datetime.now()
            if (reachTemp(94)):
                if (datetime.now() < (cycleTiming + timedelta(sec=8))):
                    if(reachTemp(57)):
                        return True
    return 

def measureData():
    for sensor in sensors:
        sensor.readSensorValue(readADC(ADC, sensor.pin))
        sensor.mapValue()
    return

def measureDataLoop(mfreq,sfreq):
    mts = timedelta(sec=(1/mfreq))
    m = int(mfreq/sfreq)
    i = 0
    timestamp = datetime.now()
    while((datetime.now()+mts)<timestamp):
        i += 1
        measureData()
        timestamp = datetime.now()
        if((i%m) == 0):
            send2DB()
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
    "Measurement_Number": 1,
    "Temperature_Probe": Temperature1.getValue(),
    "CT-value_Probe1": Photodiode1.getValue(),
    "CT-value_Probe2": Photodiode2.getValue(),
    "Peltier_DutyCycle": Peltier.getDutyCycle(),
    "Fan_DutyCycle": Fan.getDutyCycle()
    }

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
    if (buttonPrev == False):
        if (buttonPrev != buttonState):
            return True    
        else:
            return False
    return False

def initThreads():
    # thread for thermocycling
    # thread for measuring
    # thread for sending 
    global threads
    tC = threading.Thread(target=thermoCycling)
    meas = threading.Thread(target=measureDataLoop(1000))
    threads = [tC, meas]
    return

def stopPWMs():
    for i in actuators:
        i.pwm.stop()
        print("All PWM signals cleared")
    return

def startProcess():
    global SysStatus
    SysStatus = True
    #start all treads
    for t in threads:
        t.start()
    return

def stopProcess():
    #stop all threads
    global SysStatus
    SysStatus = False
    for t in threads:
        t.join()
    #stop all pwm signals
    stopPWMs()
    return

# Initiation --------------------------

initPWMsignals()
ADC = initADC()
cycleCounter = 0
SysStatus = False

# -----------------

try:
    while(cycleCounter <= numberOfCycles):
        if (checkButtons()):
            if(SysStatus == True):
                stopProcess()
            else:
                startProcess()
    if (cycleCounter == numberOfCycles):
        print(str(cycleCounter)+" Cycles done! Stopping system")
    stopProcess()
    print("Process done")

except KeyboardInterrupt:
    print("Ctl C pressed - ending program")
    stopPWMs()
    GPIO.cleanup()                     # resets GPIO ports used back to input mode
    print("Cleanup done")
