def cool():
    Peltier.ChangeDutyCycle(100)
    Fan.ChangeDutyCycle(100)
    Heater.ChangeDutyCycle(0)
    return

def heat():
    Peltier.ChangeDutyCycle(0)
    Fan.ChangeDutyCycle(0)
    Heater.ChangeDutyCycle(100)
    return

def hold():
    Peltier.ChangeDutyCycle(0)
    Fan.ChangeDutyCycle(0)
    Heater.ChangeDutyCycle(0)
    return