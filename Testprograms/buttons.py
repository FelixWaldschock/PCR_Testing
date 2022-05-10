import RPi.GPIO as GPIO
import time
from datetime import datetime, timedelta


#GPIO IN
# buttons
buttonPin = 16
buttonState = False
buttonPrev = False
EndSwitchPin = 18
GPIOins = [buttonPin, EndSwitchPin]
GPIO.setmode(GPIO.BOARD)

def initGPIOs():
    GPIO.setmode(GPIO.BOARD)
    #IN-------------
    
    # Toggle switch
    GPIO.setup(buttonPin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    # end switch
    GPIO.setup(EndSwitchPin, GPIO.IN)

    

def readGPIOins():
    print("Button = :" + str(GPIO.input(buttonPin)))
    print("EndSwitch = :" + str(GPIO.input(EndSwitchPin)))
    return

def checkButtons():
    global buttonState
    global buttonPin
    global buttonPrev
    buttonState = GPIO.input(buttonPin)
    presstime = timedelta(seconds=0.1)
    ts = datetime.now()
    if (buttonPrev != buttonState):
        if(datetime.now() + presstime > ts):
            print(True)
            buttonPrev = True
            return True    
    else:
        if(datetime.now() + presstime > ts):
            print(False)
            buttonPrev = False
            return False
    return False

    

    
initGPIOs()

while(True):
    #checkButtons()
    readGPIOins()
    #print(GPIO.input(buttonPin))
    time.sleep(1)
    