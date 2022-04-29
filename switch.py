import RPi.GPIO as GPIO

class Switch(object):
    def __init__(self, pin):
        self.pin = pin
        self.state = None
        self.setup()
        self.GPIO = None

    def setup(self):
        GPIO.setwarnings(False)
        GPIO.setup(self.pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

    def getValue(self):
        self.state = 
