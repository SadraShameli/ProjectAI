import math
from time import sleep
import RPi.GPIO as GPIO


# Helper Functions
def Clamp(n, smallest, largest):
    return max(smallest, min(n, largest))


GPIO.setmode(GPIO.BCM)


class Pin:

    # Initializer
    def __init__(self, pin, freq):
        GPIO.setup(pin, GPIO.OUT)
        self.pwm = GPIO.PWM(pin, freq)
        self.pwm.start(0)

    # Deinitializer
    def __del__(self):
        self.pwm.stop()

    def SetDuty(self, duty):
        self.pwm.ChangeDutyCycle(duty)


steeringPWM = Pin(18, 50)
motor1PWM = Pin(24, 20000)
motor2PWM = Pin(23, 20000)


def MoveForward(speed):
    motor1PWM.SetDuty(speed)
    motor2PWM.SetDuty(0)


def MoveBackward(speed):
    motor1PWM.SetDuty(0)
    motor2PWM.SetDuty(speed)


def Stop():
    motor1PWM.SetDuty(0)
    motor2PWM.SetDuty(0)


def TurnDegree(angle):
    angle = angle + 360.0 if angle < 0 else angle
    angle = angle * 0.0277778 + 3.0
    print(angle)
    steeringPWM.SetDuty(Clamp(angle, 5.0, 10.0))


def TurnRadian(angle):
    angle = angle + math.pi * 2.0 if angle < 0 else angle
    angle = 1.59155 * angle + 3.0
    steeringPWM.SetDuty(Clamp(angle, 5.0, 10.0))


try:
    while True:
        var = float(input())

        TurnDegree(var)


finally:
    GPIO.cleanup()
