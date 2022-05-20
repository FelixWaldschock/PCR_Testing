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
import send2csv
import numpy as np

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
ThreadsRunning = False
InitCycleState = False
StopThreads = False
cycleCounter = 0
measurement1 = []
measurement2 = []

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
    print("Reaching temperature: " + str(tT))
    global controller
    state = None
    while ((abs(Temperature1.mapValue()-tT)<TempTol)==False):
        #print((abs(Temperature1.mapValue()-tT)))
        if((abs(Temperature1.mapValue()-tT)<TempTol)):
            return True
        if (Temperature1.mapValue()>tT):
            if(not(state == "cool")):
                print("cooling")
            state = "cool"
            controller.cool()
        elif(Temperature1.mapValue()<tT):
            if(not(state == "heat")):
                print("heating")
            state = "heat"
            controller.heat()

    controller.hold()
    return True

def upHoldTempPID(tT,holdtime):
    global controller
    pid = PID(6.43, 0.133,0, output_limits=(0, 100)) 
    pid.setpoint = tT
    print("Heating"+ str(tT))
    while (Temperature1.mapValue()<tT):
        if(StopThreads):
            return
        else:
            Temperature1.readSensorValue(readADC(ADC, sensors[0].pin))
            pidValue = pid(Temperature1.mapValue()) # returns DutyCycle value 0-100
            controller.heatPID(pidValue,0) 
            #print("upTempPID temp:", Temperature1.mapValue(), "  DC:", pidValue, "Target: ", tT)
        
    endHold = datetime.now()+ timedelta(seconds=holdtime)

    print("Holding " + str(tT))
    while (datetime.now() < endHold ) :
        if(StopThreads):
            return
        else:
            GPIO
            Temperature1.readSensorValue(readADC(ADC, sensors[0].pin))  
            pidValue = pid(Temperature1.mapValue()) 
            controller.heatPID(pidValue,100)
            #print("holdHTempPID temp:", Temperature1.mapValue(), "  DC:", pidValue)
    """
    enlightingTime = datetime.now()+ timedelta(seconds=1)
    print("Flashing")
    if(tT == 72):
        
        while(datetime.now() < enlightingTime):
            toggleLED(True)
        toggleLED(False)
    """
    return True



def downHoldTempPID(tT,holdtime):
    global controller
    pid = PID(0.43, 10,0, output_limits=(-100, 100)) 
    pid.setpoint = tT
    print("Cooling" + str(tT))
    while (Temperature1.mapValue()>tT):
        if(StopThreads):
            return
        Temperature1.readSensorValue(readADC(ADC, sensors[0].pin))
        pidValue = pid(Temperature1.mapValue()) # returns DutyCycle value 0-100
        controller.coolPID(abs(pidValue)) 
        #print("downTempPID temp:", Temperature1.mapValue(), "  DC:", pidValue)
    
    endHold = datetime.now()+ timedelta(seconds=holdtime)
    print("Holding " + str(tT))
    while (datetime.now() < endHold):
        if(StopThreads):
            return
        Temperature1.readSensorValue(readADC(ADC, sensors[0].pin))  
        pidValue = pid(Temperature1.mapValue()) 

        if Temperature1.mapValue()>tT:
            controller.coolPID(abs(pidValue))
            #print("holdCTempPID temp:", Temperature1.mapValue(), "  DC:", pidValue)
        
        elif Temperature1.mapValue()<tT:
            controller.heatPID(abs(pidValue),100)
            #print("holdHTempPID temp:", Temperature1.mapValue(), "  DC:", pidValue)
    
    return True

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
    "Peltier_DutyCycle": float(Peltier.getDutyCycle()),
    "Heater_DutyCycle": float(Heater.getDutyCycle()),
    "Fan_DutyCycle": float(FanPeltier.getDutyCycle())
    }
    #print(MeasurementDict)
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
    print("ADC initiation succesful")
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
    #if ((SysStatus == True) and (Running == True)):
    #    StopThreads = True
    if(buttonState):
        SysStatus = not SysStatus
        return
    return


#Thread Loops--------------------
def thermoCycling():
    global cycleCounter
    global numberOfCycles
    global measurement1
    global measurement2
    global StopThreads
    while(not(StopThreads)):
        #initiation cycle -> heat to 94 and hold 60seconds
        upHoldTempPID(94, 45) 
        print("initiation cycle done!")
        # main cycling
        for i in range(numberOfCycles):
            cycleCounter+=1
            upHoldTempPID(96,15)
            downHoldTempPID(55,15)
            upHoldTempPID(74,15)
            print("Measuring")
            measurement1.append(PhotodiodeDiffMeasure(1))
            measurement2.append(PhotodiodeDiffMeasure(2))
            print("cycle " + str(cycleCounter) + " done")
            send2csv.send2csv("Messung1.csv", measurement1)
            send2csv.send2csv("Messung2.csv", measurement1)
            
        # end cycle
        send2csv.send2csv("Messung1_end.csv", measurement1)
        send2csv.send2csv("Messung2_end.csv", measurement1)
        upHoldTempPID(72,30)
        print("All cycles completed!!!!!!!!!!!")
        StopThreads = True

    return
    """print("Thermocycle Loop started")
    ht = timedelta(seconds=8)
    global cycleCounter
    while(not ProcessDone):
        if(StopThreads == True):
            return
        cycleTiming = datetime.now()
        if (reachTemp(57)):
            cycleCounter += 1
            print("Thermal cycling startet: index " + str(cycleCounter))
            cycleTiming = datetime.now()
            controller.hold()
            print("stage1")
            print(datetime.now())
            time.sleep(8)
            print(datetime.now())
            if (reachTemp(92)):
                print("stage2")
                controller.hold()
                print(datetime.now())
                time.sleep(8)
                print(datetime.now())
                if(reachTemp(57)):
                    controller.hold()
                    print("stage3")
        return"""

def measureDataLoop():
    while(not StopThreads):
        send2DB()
        time.sleep(.5)
        #print("sent data")
    return True

def readGPIOins():
    GPIO.setmode(GPIO.BOARD)
    print("Read GPIOs started")
    while(True):
        if(StopThreads):
            return
        if (GPIO.input(EndSwitchPin)==False):
            #stopProcess()
            print("End switch false")
    return True

def blinkingLED():
    print("Blinking LEDs started")
    global SysStatus
    while(not StopThreads):
        if (Running == True):
            GPIO.output(LEDstatus1Pin, False)
            GPIO.output(LEDstatus2Pin, True)
            time.sleep(1)
            GPIO.output(LEDstatus2Pin, False)
            time.sleep(1)
        else:
            GPIO.output(LEDstatus1Pin, True)
    return
#---------------------------------

def initThreads():
    # thread for thermocycling
    # thread for measuring
    # thread for sending 
    global threads
    tC = threading.Thread(target=thermoCycling)
    meas = threading.Thread(target=measureDataLoop)
    readGPIOs = threading.Thread(target=readGPIOins)
    blinkLEDs = threading.Thread(target=blinkingLED)
    threads = [tC, meas, readGPIOs, blinkLEDs]
    
    print("Initiated threads: " + str(len(threads)))
    return

def startThreads():
    global threads
    for t in threads:
        t.start()
    print("Threads started")

def startProcess():
    print("Starting Process")
    global threads
    global SysStatus
    global ThreadsRunning
    global Running
    Running = True
    toggleGPIO(True)
    
    #print(threads)
    if(ThreadsRunning == False):
        ThreadsRunning = True
        startThreads()
        print("Process started")
    else:
        print("Threads already running")
    
    return

def toggleLED(bool):
    GPIO.output(LED1Pin, bool)
    GPIO.output(LED2Pin, bool)
    return

def toggleGPIO(bool):
    print("GPIOs toggled to:" + str(bool))
    GPIO.output(Fan2Pin, bool)
    #GPIO.output(LED1Pin, bool)
    #GPIO.output(LED2Pin, bool)
    return

def stopProcess():
    #stop all threads
    print("Stopping Process")
    global threads
    global SysStatus
    SysStatus = False
    Runnig = False
    print(threads)
    StopThreads = True
    #stop all pwm signals
    print("stopped all threads")
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
    value = sensors[SensorNr].mapValue()
    return value

def waitNms(N):
    tstart = time.perf_counter_ns()
    while (True):
        tend = time.perf_counter_ns()
        if(tend-tstart >= N*1000000):
            return


def LODmeasurement():
    state = False


    numberOfSamples = 5
    index = 0
    m = []
    vlt1 = []
    vlt2 = []
    toggleGPIO(True)
    while(index < numberOfSamples):
        if(GPIO.input(buttonPin) and (not state)):
            state = True
            index += 1
            print("taking measurement: " + str(index))
            iterations = 20
            timedelta = 0.1
            m1 = []
            m2 = []
            for i in range(iterations):
                measureData()
                m1.append(Photodiode1.mapValue())
                m2.append(Photodiode2.mapValue())
                time.sleep(timedelta)
            m1 = np.array(m1)
            m2 = np.array(m2)
            std1 = np.std(m1)
            std2 = np.std(m2)
            vlt1 = np.average(m1)
            vlt2 = np.average(m2)
            m.append([vlt1,vlt2,std1,std2])
            state = False
            print("Wainting for push button")
    print(m)
    print("measurement done!")
    send2csv.send2csv("Messungen.csv",m)
    return 
        


# Initiation --------------------------

ADC = initADC()
initGPIOs()
initPWMsignals()
initThreads()
print("Initiation done")


# -------------------------------------

# Main loop

LODtest = False

if(not (LODtest)):
    try:
        print("Waiting for button push")
        while(cycleCounter < numberOfCycles):
            checkButtons()
            if((SysStatus == True) and (Running == False)):
                startProcess()
                print("Process started loop")

            """if((SysStatus == False) and Running == True):
                print("Exit due to button press")
                stopProcess() 
                break
            """
        #stopProcess()
        ProcessDone = True 
        #print(str(cycleCounter)+" Cycles done! Stopping system")
        #print("Process done")
        exit()

    except KeyboardInterrupt:
        print("Ctl C pressed - ending program")
        stopPWMs()
        toggleGPIO(False)
        GPIO.cleanup()                     # resets GPIO ports used back to input mode
        print("Cleanup done")
        exit()

if(LODtest):
    LODmeasurement()