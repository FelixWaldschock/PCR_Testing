from optparse import Values


class Sensor(object):
    values = []

    def __init__(self, type):
        self.type = type
        
    def getValueAveraged():
        r = 0
        for i in values:
            r += i
        r = r / len(values)
        return r    