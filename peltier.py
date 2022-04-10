class Peltier(object):
    import RPi.GPIO as GPIO   # Import the GPIO library.
    import time               # Import time library
    GPIO.setmode(GPIO.BOARD)  # Set Pi to use pin number when referencing GPIO pins.

    def __init__(self, pin, dutycycle):
        self.pin = pin
        self.dutycycle = dutycycle
        

    def initPWM(self):
        self.pwm = GPIO.setup(self.pin, 1000)
        return


    def updateDutyCycle(self, dutycycle):
        self.pwm.ChangeDutyCycle(dutycycle)
        return

    def stopPWM(self):
        self.pwm.stop()
        return