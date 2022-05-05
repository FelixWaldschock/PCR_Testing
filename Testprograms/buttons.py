import RPi.GPIO as GPIO
import time


#GPIO IN
# buttons
buttonPin = 16
buttonState = False
EndSwitchPin = 18
GPIOins = [buttonPin, EndSwitchPin]

def initGPIOs():
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

def readGPIOins():
    print("Button = :" + str(GPIO.Input(buttonPin)))
    print("EndSwitch = :" + str(GPIO.Input(EndSwitchPin)))
    return

while(True):
    readGPIOins()
    time.sleep(1)