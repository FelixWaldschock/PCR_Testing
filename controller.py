class controller(Object):
    def __init__(self, Ob):
        self.Peltier = Ob[0]
        self.Fan = Ob[1]
        self.Heater = Ob[2]

    def cool(self):
        self.Peltier.ChangeDutyCycle(100)
        self.Fan.ChangeDutyCycle(100)
        self.Heater.ChangeDutyCycle(0)
        return

    def heat(self):
        self.Peltier.ChangeDutyCycle(0)
        self.Fan.ChangeDutyCycle(0)
        self.Heater.ChangeDutyCycle(100)
        return

    def stop(self):
        self.Peltier.ChangeDutyCycle(0)
        self.Fan.ChangeDutyCycle(0)
        self.Heater.ChangeDutyCycle(0)
        return

    def hold(self):
        self.Peltier.ChangeDutyCycle(0)
        self.Fan.ChangeDutyCycle(0)
        self.Heater.ChangeDutyCycle(3)
        return

