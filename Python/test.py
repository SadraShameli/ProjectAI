from gpiozero import Motor
from gpiozero import AngularServo as Servo
from gpiozero.pins.pigpio import PiGPIOFactory


def MapRange(x,  in_min,  in_max,  out_min,  out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


motor = Motor(24, 23)
steering = Servo(18, pin_factory=PiGPIOFactory())


while True:
    x = float(input("Enter:"))
    print(MapRange(x, -32510, 32767, 0, 1))
