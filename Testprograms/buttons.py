from tabnanny import check
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

def checkButtons():
    print("CheckButtons")
    global buttonState
    global buttonPin
    buttonPrev = buttonState
    buttonState = GPIO.input(buttonPin)
    if (buttonPrev == False):
        if (buttonPrev != buttonState):
            print(True)
            return True    
        else:
            print(False)
            return False
    return False

while(True):
    checkButtons()
    #readGPIOins()
    #time.sleep(1)
