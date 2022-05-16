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
pid = PID(17.16, 0.9438,0, output_limits=(0, 100))
pid.sample_time = 0.1 

# general parameters
numberOfCycles = 30

# system variables
SysStatus = False
Running = False
ProcessDone = False

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
            controller.cool(100)
        elif(Temperature1.getValue()<tT):
            controller.heat(100)
    controller.hold()
    return True

def upTempPID(tT):
    global controller
    pid = PID(10.3, 0.331,0, output_limits=(0, 100)) 
    pid.setpoint = tT
    while ((abs(Temperature1.mapValue()-tT)<TempTol)==False): 
        Temperature1.readSensorValue(readADC(ADC, sensors[0].pin))
        pidValue = pid(Temperature1.mapValue()) # returns DutyCycle value 0-100
        print("upTempPID temp:", Temperature1.mapValue())
        controller.heat(pidValue) 

    return True

def downTempPID(tT):
    global controller
    controller.fan()
    pid = PID(17.16, 0.9438,0, output_limits=(0, 100)) # kühler PID noch anpassen
    pid.setpoint = tT   
    while ((abs(Temperature1.mapValue()-tT)<TempTol)==False):
        Temperature1.readSensorValue(readADC(ADC, sensors[0].pin))  
        pidValue = pid(Temperature1.mapValue()) # returns DutyCycle value 0-100
        print("downTempPID temp:", Temperature1.mapValue())
        controller.cool(pidValue) 
    controller.fanStop()
    return True

def holdTempPID(tT, holdtime):
    pid = PID(17.16, 0.9438,0, output_limits=(0, 100))
    startHold = datetime.now()
    while (datetime.now() < startHold +timedelta(seconds=holdtime) ) :
        Temperature1.readSensorValue(readADC(ADC, sensors[0].pin))  
        pidValue = pid(Temperature1.getValue()) # returns DutyCycle value 0-100
        print("holdTempPID temp:", Temperature1.mapValue())
        controller.heat(pidValue)
    return  

def measureData():
    global ADC
    for sensor in sensors:
        sensor.readSensorValue(readADC(ADC, sensor.pin))
        sensor.mapValue()
    return
 
def createMeasurementDict():
    measureData()
    MeasurementDict = {
    "Measurement_Number": 1,
    "Temperature_Probe": Temperature1.mapValue(),
    "CT-value_Probe1": Photodiode1.mapValue(),
    "CT-value_Probe2": Photodiode2.mapValue(),
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

def stopPWMs():
    for i in actuators:
        i.pwm.stop()
        print("All PWM signals cleared")
    return

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
    GPIO.setup(buttonPin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    # end switch
    GPIO.setup(EndSwitchPin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    #OUT------------
    for o in GPIOouts:
        GPIO.setup(o, GPIO.OUT)
        GPIO.output(o, False)
    
    GPIO.output(LEDstatus1Pin, True)
    return

def checkButtons():
    global buttonState
    global buttonPin
    global SysStatus
    buttonState = GPIO.input(buttonPin)
    if(buttonState):
        SysStatus = not SysStatus
        return
    return


#Thread Loops--------------------
def thermoCycling():
    global cycleCounter
    while(not ProcessDone):
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
    return True

def measureDataLoop():
    while(True):
        send2DB()
        time.sleep(.5)
        print("sent data")
    return True

def readGPIOins():
    while(True):
        if (GPIO.Input(EndSwitchPin)==False):
            stopProcess()
    return True

def blinkingLED():
    global SysStatus
    while(True):
        if (Running == True):
            GPIO.output(LEDstatus2Pin, False)
            GPIO.output(LEDstatus1Pin, True)
            time.sleep(1)
            GPIO.output(LEDstatus1Pin, False)
            time.sleep(1)
        else:
            GPIO.output(LEDstatus2Pin, True)
    return
#---------------------------------

def initThreads():
    # thread for thermocycling
    # thread for measuring
    # thread for sending 
    global threads
    tC = threading.Thread(target=thermoCycling)
    meas = threading.Thread(target=measureDataLoop())
    readGPIOs = threading.Thread(target=readGPIOins)
    blinkLEDs = threading.Thread(target=blinkingLED)
    threads = [tC, meas, readGPIOs, blinkLEDs]
    print("Initiated threads: " + str(len(threads)))
    return



def startProcess():
    print("Starting Process")
    global threads
    global SysStatus
    toggleGPIO(True)
    SysStatus = True
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


def PhotodiodeDiffMeasure(DiodeNr):
    global ADC
    if ((DiodeNr == 1) or (DiodeNr ==2)):

        #20 Measurements
        sensors[DiodeNr].changeAverageOf(20)
        
        #Wahrscheinlich immer noch sehr inakkurat!!!

        #Turn on LED
        GPIO.output(GPIOouts[DiodeNr - 1], True)

        waitNms(20) #Wait for one 50Hz net period

        #Measure 20 times, every 21 ms, so one net period + 1 ms
        value = nMeasuresTimed(20, 21, DiodeNr)
        
        waitNms(20) #Wait for one 50Hz net period

        #Turn off LED
        GPIO.output(GPIOouts[DiodeNr - 1], False)

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


def LODmeasurement():
    numberOfSamples = 5
    index = 0
    measurementDiode1 = []
    measurementDiode2 = []
    while(index < numberOfSamples):
        if(GPIO.input(buttonPin)):
            index += 1
            print("taking measurement")
            iterations = 10
            for i in range(iterations):
                m1 = Photodiode1.getValue()
                m2 = Photodiode2.getValue()
                time.sleep(0.1)
            m1 = m1/iterations
            m2 = m2/iterations
            measurementDiode1.append(m1)
            measurementDiode2.append(m2)
        print("Wainting for push button")
    return measurementDiode1, measurementDiode2
        
# Initiation --------------------------

ADC = initADC()
initPWMsignals()
initGPIOs()
initThreads()
cycleCounter = 0
print("Initiation done")

# -------------------------------------

# Main loop

try:
    print("Try")
    while(cycleCounter <= numberOfCycles):
        #print("Loop started")
        checkButtons()
        if((SysStatus == True) and not Running):
            startProcess()
            print("Process Stared loop")
        
        elif((SysStatus == False) and Running):
            stopProcess()
        
    ProcessDone = True 
    print(str(cycleCounter)+" Cycles done! Stopping system")

    stopProcess()
    print("Process done")

except KeyboardInterrupt:
    print("Ctl C pressed - ending program")
    stopPWMs()
    toggleGPIO(False)
    GPIO.cleanup()                     # resets GPIO ports used back to input mode
    print("Cleanup done")
