class Sensor(object):
    def __init__(self, type, pin, vcc):
        self.type = type
        self.pin = pin
        self.values = []
        self.vcc = vcc

    def readSensorValue(self, rvalue):
        self.values.insert(0,rvalue)
        if (len(self.values)>=20):
            self.values.pop()
   
    def getValueAveraged(self):
        r = 0.0
        t = sum(self.values)
        l = len(self.values)
        if (l>0):
            r = t/l
        else:
            r = t
        return float(r)

    def getValue(self):
        r = self.getValueAveraged()
        return float(r)
    
    def mapValue(self):
        if (self.type == "PT1000"):
           
            """
            Mapping voltage difference to temperature
            per 1 degree increasement resistance increases by 4.32 Ohms
            if PT1000 is @ 0°C there is no voltage difference 
            """

            t0 = 1000 #1000Ohm @0°C
            dR_proGrad = 4.32
            dV_dT = 1.7844141 * 0.001

            r = round(abs(self.getValue())/dV_dT,2)
            r += 10
            return r

        elif(self.type == "Photodiode"):
            r = abs(self.getValue())
            return r
        else:
            print("Type not defined")
        