class Sensor(object):
    def __init__(self, type, pin):
        self.type = type
        self.pin = pin
        self.values = []

    def readSensorValue(self, rvalue):
        self.values.insert(0,rvalue)
        if (len(self.values)>=10):
            self.values.pop()
   
    def getValueAveraged(self):
        r = 0.0
        for i in self.values:
            r += i
        r = r / len(self.values)
        return float(r)

    def getValue(self):
        r = self.getValueAveraged()
        return r
    