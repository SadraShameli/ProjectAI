import math
from time import sleep

# External Libraries
import ydlidar
import RPi.GPIO as GPIO

# Helper Functions
def Clamp(n, smallest, largest):
    return max(smallest, min(n, largest))

# Pin Manager
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


# Main Project
class ProjectAI:

    # Initializer
    def __init__(self):
        print('Welcome to ProjectAI')

        # GPIO Setup
        GPIO.setmode(GPIO.BCM)
        self.steeringPWM = Pin(18, 50)
        self.motor1PWM = Pin(24, 20000)
        self.motor2PWM = Pin(25, 20000)

        # YDLidar Setup
        self.lidar = ydlidar.CYdLidar()
        self.lidar.setlidaropt(ydlidar.LidarPropSerialBaudrate, 128000)
        self.lidar.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TRIANGLE)
        self.lidar.setlidaropt(ydlidar.LidarPropDeviceType, ydlidar.YDLIDAR_TYPE_SERIAL)
        self.lidar.setlidaropt(ydlidar.LidarPropAutoReconnect, True)
        self.lidar.setlidaropt(ydlidar.LidarPropAbnormalCheckCount, 4)
        self.lidar.setlidaropt(ydlidar.LidarPropScanFrequency, 12.0)
        self.lidar.setlidaropt(ydlidar.LidarPropSampleRate, 5)
        self.lidar.setlidaropt(ydlidar.LidarPropMaxRange, 10.0)
        self.lidar.setlidaropt(ydlidar.LidarPropMinRange, 0.12)

        # Running Application Loop
        try:
            self.Runtime()
        except KeyboardInterrupt:
            print('\nTerminating Application')

    # Search for YDLidar devices
    def ConnectLidar(self):
        for version, port in ydlidar.lidarPortList().items():
            print(f'YDLidar device has been found - [Port: {port}, Version: {version}]')
            
            # Try to Initialize YDLidar SDK
            self.lidar.setlidaropt(ydlidar.LidarPropSerialPort, port)
            if self.lidar.initialize() and self.lidar.turnOn():
                return True

    # Moving Robot
    def MoveForward(self, speed):
        self.motor1PWM.SetDuty(speed)
        self.motor2PWM.SetDuty(0)

    def MoveBackward(self, speed):
        self.motor1PWM.SetDuty(0)
        self.motor2PWM.SetDuty(speed)

    def Stop(self):
        self.steeringPWM.SetDuty(5)
        self.motor1PWM.SetDuty(0)
        self.motor2PWM.SetDuty(0)

    def TurnDegree(self, angle):
        duty = 0.01389 * -angle + 5
        self.steeringPWM.SetDuty(Clamp(duty, 5, 10))

    def TurnRadian(self, angle):
        duty = 10 - (1.59156 * -angle)
        self.steeringPWM.SetDuty(Clamp(duty, 5, 10))
        
    # Runtime
    def Runtime(self):
        # Loop indefinitely until connected
        while not self.ConnectLidar():
            sleep(1)

        scan = ydlidar.LaserScan()
        while True:
            if self.lidar.doProcessSimple(scan):
                # print(f'Scan received: {scan.points.size()} ranges at {1.0 / scan.config.scan_time} Hz')
                    
                maxRange = max(scan.points, key=lambda point: point.range)
                print(f'{math.degrees(maxRange.angle)} : {maxRange.range}\n')
                
                self.TurnRadian(maxRange.angle)
                self.MoveForward(15)

            else:
                print('Failed to get Lidar Data')
                self.Stop()

    # Deinitializer
    def __del__(self):
        print('Releasing resources')
        
        # Cleaning up YDLidar SDK
        self.lidar.turnOff()
        self.lidar.disconnecting()
        
        # Cleaning up GPIO
        GPIO.cleanup()


# Instantiating Main Application
project = ProjectAI()
