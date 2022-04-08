class Sensor(object):
    global values
    values = []
    global value

    def __init__(self, type, pin):
        self.type = type
        self.pin = pin


    def readSensorValue(rvalue):
        if (len(values)>=60):
            values.pop()
        values.append(rvalue)

    def getValueAveraged():
        r = 0
        for i in values:
            r += i
        r = r / len(values)
        value = r    

    def getValue():
        return value