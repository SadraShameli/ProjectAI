import os
import time
import math
import threading
import functools
import json
import mimetypes


# External Libraries
import ydlidar
import RPi.GPIO as GPIO
import netifaces as ni
from http.server import BaseHTTPRequestHandler, HTTPServer
from pyPS4Controller.controller import Controller

# Constants
PI_2 = math.pi / 2 
RAD_90 = math.degrees(90)
HEADING_LEFT = 90
HEADING_AHEAD = 270
HEADING_RIGHT = 360


# Helper Functions
def Clamp(n, smallest, largest):
    return max(smallest, min(n, largest))

def MapRange( x,  in_min,  in_max,  out_min,  out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def GetIpAddress():
    return ni.ifaddresses('wlan0')[ni.AF_INET][0]['addr']


# Helper Classes
# Pin Manager
class Pin:
    # Initializer
    def __init__(self, pin, freq):        
        GPIO.setup(pin, GPIO.OUT)
        self.pwm = GPIO.PWM(pin, freq)
        self.pwm.start(0)
        self.pin = pin
        self.log(f'Initialized PWM Pin {pin} with Frequency {freq}')

    # Deinitializer
    def __del__(self): 
        self.pwm.stop()
        self.log(f'Deinitialized Pin {self.pin}')

    def SetDuty(self, duty):
        self.pwm.ChangeDutyCycle(duty)
    
    @staticmethod    
    def log(*args):
        print(f'[Pin] {args[0]}')
        
        
# HTTP Helper
class HTTPHelper(BaseHTTPRequestHandler):
    
    # Overwriting initializer to accept a callback
    def __init__(self, *args, callback=None, **kwargs):
        self.callback = callback
        super().__init__(*args, **kwargs)
        
        
    def do_GET(self):  
        try:
            if self.path == '/': 
                self.path = '/index.html'   
                    
            if '.' in self.path:             
                with open(f'../WebServer{self.path}', 'rb') as f:
                    mimeType = mimetypes.guess_type(self.path)[0]

                    self.send_response(200)   
                    self.send_header("Content-type", mimeType)
                    self.end_headers()  
                    self.wfile.write(f.read())    
                              
                    self.log(f'File served: {self.path}')             
            else:
                self.send_response(404) 
                self.end_headers()  
                self.log(f'URL {self.path} is not implemented')
                
        except FileNotFoundError as e:
            self.send_response(404) 
            self.end_headers()  
            self.log(f'File or directory not found: {e.filename}')
            
        except Exception as e:
            self.send_response(500) 
            self.end_headers()  
            self.log(f'Exception thrown: {e}')  
    
    
    def do_POST(self):
        try:
            content_length = int(self.headers['Content-Length'])
            post_data = json.loads(self.rfile.read(content_length))
            
            directionProperty = ['Direction', 'Speed']
            if all(dir in post_data for dir in directionProperty):
                dir = post_data.get('Direction')
                self.callback.speed = post_data.get('Speed')    
                
                # Parse direction from JSON
                if 'forward' in dir: self.callback.MoveForward(self.callback.speed)
                elif 'backward' in dir: self.callback.MoveBackward(self.callback.speed)
                elif 'left' in dir: self.callback.TurnLeft()
                elif 'right' in dir: self.callback.TurnRight()
                elif 'stop' in dir: self.callback.Stop()
                    
                self.send_response(200)  
                self.end_headers()              
            else:
                self.send_response(404) 
                self.end_headers()  
                self.log(f'URL {self.path} is not implemented')
        
        except Exception as e:
            self.send_response(500) 
            self.end_headers()  
            self.log(f'Exception thrown: {e}')  
        
    def log_message(self, format, *args):
        return
    
    def log(*args):
        print(f'[HTTPServer] {args[1]}')


# ConsoleController Helper
class ConsoleController(Controller):
    
    # Initializer
    def __init__(self, callback=None, **kwargs):
        self.callback = callback
        Controller.__init__(self, **kwargs)
        
    def map_speed(self, speed): 
        return MapRange(speed, -32510, 32767, 0, 100)
    def map_heading(self, heading) : 
        return MapRange(heading, -32510, 32767, HEADING_LEFT, HEADING_RIGHT)
    
    def on_up_arrow_press(self):
        self.callback.MoveForward(self.callback.speed)
    def on_down_arrow_press(self):
        self.callback.MoveBackward(self.callback.speed)
    def on_up_down_arrow_release(self):
        self.callback.Stop()
    def on_R2_press(self, speed):
        self.callback.MoveForward(self.map_speed(speed))
    def on_L2_press(self, speed):
        self.callback.MoveBackward(self.map_speed(speed))
    def on_R2_release(self):
        self.callback.Stop()
    def on_L2_release(self):
        self.callback.Stop()
    def on_R3_right(self, degree):
        self.callback.TurnDegree(self.map_heading(degree))
    def on_R3_left(self, degree):
        self.callback.TurnDegree(self.map_heading(degree))

        
# ProjectAI 
class ProjectAI:    
    
    # Initializer
    def __init__(self):                
        # GPIO Setup
        self.log('Initializing GPIO Pins')
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)    
        self.steeringPWM = Pin(18, 50)
        self.motor1PWM = Pin(24, 20000)
        self.motor2PWM = Pin(23, 20000)        
        self.speed = 0
        
        # YDLidar Setup
        self.log('Initializing YDLidar SDK')
        self.lidar = ydlidar.CYdLidar()
        self.lidar.setlidaropt(ydlidar.LidarPropSerialBaudrate, 128000)
        self.lidar.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TRIANGLE)
        self.lidar.setlidaropt(ydlidar.LidarPropDeviceType, ydlidar.YDLIDAR_TYPE_SERIAL)
        self.lidar.setlidaropt(ydlidar.LidarPropAutoReconnect, True)
        self.lidar.setlidaropt(ydlidar.LidarPropScanFrequency, 12.0)
        self.lidar.setlidaropt(ydlidar.LidarPropSampleRate, 5)
        self.lidar.setlidaropt(ydlidar.LidarPropMaxRange, 10.0)
        self.lidar.setlidaropt(ydlidar.LidarPropMinRange, 0.12)

        # Webserver Setup
        self.log('Initializing Webserver')      
        handler_partial = functools.partial(HTTPHelper, callback=self)      
        self.webserver = HTTPServer((GetIpAddress(), 8080), handler_partial)
        threading.Thread(target=self.webserver.serve_forever, daemon=True).start()
        self.log(f'WebServer started at {self.webserver.server_address[0]}:{self.webserver.server_address[1]}') 
        
        # Console Controller Setup  
        # threading.Thread(target=self.ConnectToDevices, daemon=True).start()    
        while not self.ConnectLidar():
            time.sleep(0.25)        
            
        while not self.ConnectController():
            time.sleep(0.25)        
        
        self.log(f'''Please control the robot via the URL {self.webserver.server_address[0]}:{self.webserver.server_address[1]} or a PS4 or PS5 controller''')
        
        
    # Deinitializer
    def __del__(self):
        self.log('Deinitializing')
        
        # Cleaning up GPIO
        GPIO.cleanup()
        
        # Cleaning up YDLidar SDK
        if hasattr(self, 'lidar'):
            self.lidar.turnOff()
            self.lidar.disconnecting()     
            
        # Cleaning up Webserver
        if hasattr(self, 'webserver'):
          self.webserver.server_close()   
        
        
    # Search for YDLidar devices
    def ConnectLidar(self):
        portList = ydlidar.lidarPortList().items()
        if len(portList) == 0:
            self.log('No YDLidar device could be found')
            return False
        
        # Iterate through all YDLidar devices
        for version, port in portList:
            # YDLidar Found
            self.log(f'YDLidar device has been found - Port: {port}, Version: {version}')
            
            # Try to Initialize YDLidar SDK
            self.lidar.setlidaropt(ydlidar.LidarPropSerialPort, port)
            if self.lidar.initialize() and self.lidar.turnOn(): 
                return True   
            else: 
                self.log(f'Failed to connect with YDLidar at {port}') 
                return False 
    
    
    # Search for Console Controller
    def ConnectController(self):
        # Iterate through all joystick interfaces
        for input in range(0, 8):
            address = f'/dev/input/js{input}'            
            if os.path.exists(address):                
                self.controller = ConsoleController(interface=address, connecting_using_ds4drv=False, callback=self)        
                threading.Thread(target=self.controller.listen, daemon=True).start()
                time.sleep(0.25)
                if self.controller.is_connected: 
                    self.log(f'Connected to controller at {address}')
                    return True
        
        self.log('No console controller could be found')
        return False
            
            
    # Moving Robot
    def MoveForward(self, speed):
        self.motor1PWM.SetDuty(Clamp(speed, 0, 100))
        self.motor2PWM.SetDuty(0)

    def MoveBackward(self, speed):
        self.motor1PWM.SetDuty(0)
        self.motor2PWM.SetDuty(Clamp(speed, 0, 100))

    def Stop(self):
        self.motor1PWM.SetDuty(0)
        self.motor2PWM.SetDuty(0)
        
    def TurnLeft(self): self.TurnDegree(HEADING_LEFT)
    def TurnAhead(self): self.TurnDegree(HEADING_AHEAD)
    def TurnRight(self): self.TurnDegree(HEADING_RIGHT)        

    def TurnDegree(self, angle):        
        if angle < 0: angle = 360.0 - abs(angle)
        angle = (angle - 90) * 0.0277778 + 2.15
        self.steeringPWM.SetDuty(Clamp(angle, 5.0, 10.0))

    def TurnRadian(self, angle):   
        if angle < 0: angle = angle + math.tau
        angle = 1.59155 * (angle - RAD_90) + 2.15
        self.steeringPWM.SetDuty(Clamp(angle, 5.0, 10.0))
     
     
    # Runtime
    def Runtime(self):                

        scan = ydlidar.LaserScan()                
        if self.lidar.doProcessSimple(scan):
            # print(f'Scan received: {scan.points.size()} ranges at {1.0 / scan.config.scan_time} Hz')                
            
            distancesMin90 = list(filter(lambda point: round(math.degrees(point.angle)) == -90, scan.points))
            distances120 = list(filter(lambda point: round(math.degrees(point.angle)) == 120, scan.points))
            distances45 = list(filter(lambda point: round(math.degrees(point.angle)) == 45, scan.points))
            
            # temp = [[], [], []]
            # for point in distancesMin90:
            #     temp[0].append(point.range)
            #     
            # for point in distances120:
            #     temp[1].append(point.range)
            #     
            # for point in distances45:
            #     temp[2].append(point.range)
                 
            # result = run_robot(temp, 'winner.pkl', 'config.txt')
             
            maxRange = max(scan.points, key=lambda point: point.range)
            # self.log(f'{math.degrees(maxRange.angle)} : {maxRange.range}\n')
             
            # self.TurnRadian(maxRange.angle)
            # self.MoveForward(15)

        else:
            self.log('Failed to get Lidar Data')
            self.Stop()
    
                
    @staticmethod
    def log(*args):
        print(f'[ProjectAI] {args[0]}')
        
        
# Main Entrance
# Catching errors
try:    
    print('''
----------------------------------------------------
Welcome to
  ____            _           _        _      ___    
 |  _ \ _ __ ___ (_) ___  ___| |_     / \    |_ _|   
 | |_) | '__/ _ \| |/ _ \/ __| __|   / _ \    | |    
 |  __/| | | (_) | |  __/ (__| |_   / ___ \ _ | | _  
 |_|   |_|  \___// |\___|\___|\__| /_/   \_(_)___(_) 

Made with \u2665 By Sadra Shameli
----------------------------------------------------
''')
    
    # Instantiating Main Application
    s_Robot = ProjectAI()         

    # Loop indefinitely until Lidar is connected
    while True:
        s_Robot.Runtime()
    
  
except KeyboardInterrupt:
    print('Terminating Application')
    quit()
    
except Exception as e:
    print(f'Exception thrown: {e}') 