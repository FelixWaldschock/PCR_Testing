import push2DB as p2db
import readSensor as rS
import sensor
import actuator
import controller as cntrl
from datetime import datetime, timedelta
import RPi.GPIO as GPIO
from ads1015 import ADS1015
import threading
import time

#PID
from simple_pid import PID
pid.sample_time = 0.01 


# general parameters
numberOfCycles = 30

# create sensor objects
Temperature1 = sensor.Sensor("PT1000", ['in0/in1'],3.3, 20)
Photodiode1 = sensor.Sensor("Photodiode", ['in2/gnd'],3.3, 20)
Photodiode2 = sensor.Sensor("Photodiode", ['in3/gnd'],3.3, 20)
sensors = [Temperature1, Photodiode1, Photodiode2]

# create actor objects
Peltier = actuator.Actuator("Peltierelement", 33)
FanPeltier = actuator.Actuator("FanPeltier", 32)
Heater = actuator.Actuator("Heater", 12)
actuators = [Peltier, FanPeltier, Heater]
controller = cntrl.controller(actuators)

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

# RPi Pin Layout
# https://duckduckgo.com/?q=raspberry+pi+pin+layout&t=brave&iax=images&ia=images&iai=https%3A%2F%2Ffossbytes.com%2Fwp-content%2Fuploads%2F2021%2F04%2Fgpio-pins-raspberry-pi-4-e1617866771594.png

threads = []

TempTol = 1

pwms = []

# FUNCTIONS --------------------------
def reachTemp(tT): #tT targetTemperature
    global controller

    while ((abs(Temperature1.getValue()-tT)<TempTol)==False):  
        if (Temperature1.getValue()>tT):
            controller.cool()
        elif(Temperature1.getValue()<tT):
            controller.heat()
    controller.hold()
    return True



def upTempPID(tT):
    global controller
    pid = PID(17.16, 0.9438,0, output_limits=(0, 100)) 
    pid.setpoint = tT
    while ((abs(Temperature1.getValue()-tT)<TempTol)==False): 
        pidValue = pid(Temperature1.getValue()) # returns DutyCycle value 0-100
        controller.heat(pidValue) 

    return True


def downTempPID(tT):
    global controller
    pid = PID(17.16, 0.9438,0, output_limits=(0, 100)) # kühler PID noch anpassen
    pid.setpoint = tT   
    while ((abs(Temperature1.getValue()-tT)<TempTol)==False):  
        pidValue = pid(Temperature1.getValue()) # returns DutyCycle value 0-100
        controller.Peltier(pidValue) 
    return True


def holdTempPID(tT):
    pid = PID(17.16, 0.9438,0, output_limits=(0, 100))
    startHold = datetime.now()
    while (datetime.now() < startHold +timedelta(seconds=8) ) :
        pidValue = pid(Temperature1.getValue()) # returns DutyCycle value 0-100
        controller.heat(pidValue)
    return   


def thermoCycling():
    global cycleCounter
    cycleCounter += 1
    print("Thermal cycling startet: index " + str(cycleCounter))
    
    cycleTiming = datetime.now()
    if (reachTemp(57)):
        if (datetime.now() < (cycleTiming + timedelta(seconds=8))):
            cycleTiming = datetime.now()
            if (reachTemp(94)):
                if (datetime.now() < (cycleTiming + timedelta(seconds=8))):
                    if(reachTemp(57)):
                        return True
    return 

def measureData():
    global ADC
    for sensor in sensors:
        sensor.readSensorValue(readADC(ADC, sensor.pin))
        sensor.mapValue()
    return

def measureDataLoop(mfreq,sfreq):
    print("MeasureDataLoop")
    mts = timedelta(seconds=(1/mfreq))
    m = int(mfreq/sfreq)
    i = 0
    timestamp = datetime.now()
    while((datetime.now()+mts)>timestamp):
        i += 1
        measureData()
        timestamp = datetime.now()
        if((i%m) == 0):
            send2DB()
    return
    
def createMeasurementDict():
    MeasurementDict = {
    "Measurement_Number": 1,
    "Temperature_Probe": Temperature1.mapValue(),
    "CT-value_Probe1": Photodiode1.mapValue(),
    #"CT-value_Probe2": Photodiode2.mapValue(),
    "Peltier_DutyCycle": Peltier.getDutyCycle(),
    "Fan_DutyCycle": FanPeltier.getDutyCycle()
    }
    print(MeasurementDict)
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
    return [ADC,ADC.get_reference_voltage()]

def readADC(chip, inputPort):
    v = chip[0].get_voltage(channel=inputPort[0])
    return v

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

def checkButtons():
    print("CheckButtons")
    global buttonState
    global buttonPin
    buttonPrev = buttonState
    buttonState = GPIO.input(buttonPin)
    if (buttonPrev == False):
        if (buttonPrev != buttonState):
            return True    
        else:
            return False
    return False

def readGPIOins():
    #if (GPIO.Input(EndSwitchPin)==False):
    #    stopProcess()
    return

def initThreads():
    # thread for thermocycling
    # thread for measuring
    # thread for sending 
    global threads
    tC = threading.Thread(target=thermoCycling)
    meas = threading.Thread(target=measureDataLoop(1000,10))
    readGPIOs = threading.Thread(target=readGPIOins)
    blinkLEDs = threading.Thread(target=blinkingLED)
    threads = [tC, meas, readGPIOs, blinkLEDs]
    print("Initiated threads: " + str(len(threads)))
    return

def stopPWMs():
    for i in actuators:
        i.pwm.stop()
        print("All PWM signals cleared")
    return

def startProcess():
    print("Starting Process")
    global threads
    global SysStatus
    toggleGPIO(True)
    SysStatus = True
    #start all treads
    print(threads)
    for t in threads:
        t.start()
        print(t)
    print("Process started")
    return

def toggleGPIO(bool):
    print("GPIOs toggled to:" + str(bool))
    GPIO.output(Fan2Pin, bool)
    GPIO.output(LED1Pin, bool)
    GPIO.output(LED2Pin, bool)
    return

def stopProcess():
    #stop all threads
    global threads
    global SysStatus
    SysStatus = False
    for t in threads:
        t.join()

    #stop all pwm signals
    stopPWMs()
    toggleGPIO(False)
    return

def blinkingLED():
    while(True):
        if (SysStatus == True):
            while(True):
                GPIO.output(LEDstatus1Pin, True)
                time.sleep(1)
                GPIO.output(LEDstatus1Pin, False)


def PhotodiodeDiffMeasure(DiodeNr):
    global ADC
    if ((DiodeNr == 1) or (DiodeNr ==2)):

        #40 Measurements
        sensors[DiodeNr].changeAverageOf(40)
        
        #Wahrscheinlich immer noch sehr inakkurat!!!

        #Turn on LED
        GPIO.output(GPIOouts[DiodeNr - 1], True)

        waitNms(20) #Wait for one 50Hz net period

        #Measure 40 times over two periods
        value = nMeasuresTimed(40, 1, DiodeNr)
        
        waitNms(20) #Wait for one 50Hz net period

        #Turn off LED
        GPIO.output(GPIOouts[DiodeNr - 1], False)

        waitNms(20) #Wait for one 50Hz net period

        #Measure 40 times over two periods
        nolight = nMeasuresTimed(40, 1, DiodeNr)

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
            if (tend-tstart >= deltatms*1e6):
                break
            print("tend-tstart", tend-tstart)   #Test print to see how long it takes

        #Restart timer
        tstart = time.perf_counter_ns()
        print("break", i)  #Test print to see how long it takes
    #Get the average
    value = sensors[SensorNr].getValue()
    return value

def waitNms(N):
    tstart = time.perf_counter_ns()
    while (True):
        tend = time.perf_counter_ns()
        if(tend-tstart >= N*1e6)
        break

        
# Initiation --------------------------

ADC = initADC()
initPWMsignals()
initGPIOs()
initThreads()
cycleCounter = 0
SysStatus = False
print("Initiation done")

# -----------------

try:
    print("Try")
    while(cycleCounter <= numberOfCycles):
        print("Loop started")
        if (checkButtons()):
            if(SysStatus == True):
                stopProcess()
            else:
                startProcess()
                print("Process Stared loop")
    if (cycleCounter == numberOfCycles):
        print(str(cycleCounter)+" Cycles done! Stopping system")
    stopProcess()
    print("Process done")

except KeyboardInterrupt:
    print("Ctl C pressed - ending program")
    stopPWMs()
    GPIO.cleanup()                     # resets GPIO ports used back to input mode
    print("Cleanup done")
