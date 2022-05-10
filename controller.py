class controller(object):
    def __init__(self, Ob):
        self.Peltier = Ob[0]
        self.Fan = Ob[1]
        self.Heater = Ob[2]

    def cool(self):
        print("cooling")
        self.Peltier.cDC(100)
        self.Fan.cDC(100)
        self.Heater.cDC(0)
        return

    def heat(self):
        print("heating")
        self.Peltier.cDC(0)
        self.Fan.cDC(100)
        self.Heater.cDC(100)
        return

    def stop(self):
        print("stop")
        self.Peltier.cDC(0)
        self.Fan.cDC(0)
        self.Heater.cDC(0)
        return

    def hold(self):
        print("holding")
        self.Peltier.cDC(0)
        self.Fan.cDC(100)
        self.Heater.cDC(3)
        return

    def fan(self):
        self.Fan.cDC(100)
        return

    def all(self):
        print("all")
        self.Peltier.cDC(100)
        self.Fan.cDC(100)
        self.Heater.cDC(100)