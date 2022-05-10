class Actuator(object):
    
    def __init__(self, type, pin):
        self.type = type
        self.pin = pin
        self.dutyCycle = None

    def pushPWM(self, pwm):
        self.pwm = pwm
        self.pwm.start(0)
        return

    def cDC(self, dc):
        self.pwm.ChangeDutyCycle(dc)
        self.dutyCycle = dc
        return

    def getDutyCycle(self):
        return self.dutyCycle
