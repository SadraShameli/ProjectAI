import math
import RPi.GPIO as GPIO

# Helper Functions
def Clamp(n, smallest, largest):
    return max(smallest, min(n, largest))
  
GPIO.setmode(GPIO.BCM)

class Pin:
  
  # Initializer
  def __init__(self, pin, freq): 
    GPIO.setup(pin, GPIO.OUT)
    self.pwm=GPIO.PWM(pin, freq)
    self.pwm.start(0)
    
  # Deinitializer
  def __del__(self):
    self.pwm.stop()
    
  def SetDuty(self, duty):
    self.pwm.ChangeDutyCycle(duty)


steeringPWM = Pin(18, 50)

def TurnDegree(angle):
  duty = 0.01389 * -angle + 5
  steeringPWM.SetDuty(Clamp(duty, 5, 10))
    
def TurnRadian(angle):
  TurnDegree(math.degrees(float(angle)))
      
try:
  while True:    
    var = input()
    TurnDegree(var)    
        
finally:    
    GPIO.cleanup()