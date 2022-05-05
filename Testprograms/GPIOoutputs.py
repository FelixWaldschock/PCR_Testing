import RPi.GPIO as GPIO
from datetime import datetime, timedelta
import time

LED1Pin = 11
LED2Pin = 13
Fan2Pin = 15
LEDstatus1Pin = 24
LEDstatus2Pin = 26
GPIOouts = [LED1Pin, LED2Pin, Fan2Pin, LEDstatus1Pin, LEDstatus2Pin]

def blinkingLED(Pin, Timestamp, freq):
    ts = 1000/freq
    s = False
    while(not s):
        if ((datetime.now()+timedelta(milliseconds=ts))>Timestamp):
            GPIO.Output(Pin, not GPIO.input(Pin))
            Timestamp = datetime.now()
            s = not s
        return

def toggleGPIO():
    global GPIOouts
    for g in GPIOouts:
        GPIO.Output(g, not GPIO.Input(g))
    return

def setGPIO(bool):
    global GPIOouts
    for g in GPIOouts:
        GPIO.Output(g, bool)
        print("GPIO " + str(g) + " set:" + str(bool))
    return

setGPIO(False)

try:
    while(True):
        setGPIO(True)
        time.sleep(1)
        setGPIO(False)
        time.sleep(3)
        print("Test toggle")
        for i in range(10):
            toggleGPIO()
            time.sleep(1)
        print("Test blinking")
        ts = datetime.now()
        for i in range(10):
            ts = blinkingLED(LEDstatus1Pin, ts, 10)


except KeyboardInterrupt:
    print("Ctl C pressed - ending program")
    stopPWMs()
    GPIO.cleanup()                     # resets GPIO ports used back to input mode
    print("Cleanup done")