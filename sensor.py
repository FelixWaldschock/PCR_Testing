class Sensor(object):
    global values
    values = []
    

    def __init__(self, type, pin):
        self.type = type
        self.pin = pin

    def readSensorValue(self, rvalue):
        values.insert(0,rvalue)
        if (len(values)>=10):
            values.pop()

    def getValueAveraged(self):
        r = 0.0
        for i in values:
            r += i
        r = r / len(values)
        return float(r)

    def getValue(self):
        r = self.getValueAveraged()
        return r
    