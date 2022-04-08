from select import POLLWRNORM
import RPi.GPIO as GPIO
import time

listOfDevices = ["Fan1", "Fan2", "LED", "Peltier"]

pins = [12,13,18,19]
pin = pins[0]

pwms = [pwm0, pwm1, pwm2, pwm3]


def setup(p):
    GPIO.setmode(GPIO.BOARD)
    for i in range(len(pwms)):
        GPIO.setup(pins[i], GPIO.OUT)
        GPIO.output(pins[i], GPIO.LOW)
        pwms[i] = GPIO.PWM(pins[i], 1000)
        pwms[i].start(0)

def destroy(pwmS):
    pwmS.stop()
    GPIO.cleanup()