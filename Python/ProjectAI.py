import os
import sys
import time
import math
import functools
import json
import numpy
from threading import Thread

# External Libraries
import ydlidar
import netifaces as ni
import tflite_runtime.interpreter as tf
from gpiozero import Motor
from gpiozero import AngularServo as Servo
from gpiozero.pins.pigpio import PiGPIOFactory
from http.server import BaseHTTPRequestHandler, HTTPServer
from pyPS4Controller.controller import Controller

# Constants
CURRENT_FOLDER = os.path.dirname(os.path.realpath(__file__))
PROJECT_FOLDER = os.path.dirname(CURRENT_FOLDER)
CONTROLLER_ADDRESS = '/dev/input/js'
HEADING_LEFT = 0.0
HEADING_TOPLEFT = math.pi * 0.25
HEADING_AHEAD = math.pi * 0.5
HEADING_TOPRIGHT = math.pi * 0.75
HEADING_RIGHT = math.pi * 1.0


# A.I.
AI_FOLDER = f'{CURRENT_FOLDER}/AI'
AI_MODEL_NAME = f'{AI_FOLDER}/model.tflite'
DEFAULT_APERTURE = math.radians(8)


# Helper Functions
def ProgramRestart():
    Thread(target=os.execv, args=[sys.executable, ['python'] + sys.argv]).start()
    os._exit()


def Clamp(n, smallest, largest):
    return max(smallest, min(n, largest))


def MapRange(x,  in_min,  in_max,  out_min,  out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


def Average(lst):
    length = len(lst)
    if not length:
        return 0
    return sum(lst) / len(lst)


def GetIpAddress():
    return ni.ifaddresses('wlan0')[ni.AF_INET][0]['addr']


# Helper Classes
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
                with open(f'{PROJECT_FOLDER}{self.path}', 'rb') as f:

                    self.send_response(200)
                    self.end_headers()
                    self.wfile.write(f.read())

                    self.log(f'File served: {self.path}')

            if 'restart' in self.path:
                self.send_response(200)
                self.end_headers()
                ProgramRestart()

            if 'reboot' in self.path:
                self.send_response(200)
                self.end_headers()
                os.system('sudo reboot')

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
                self.callback.ParseRobotControl(post_data)

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
    def __init__(self, callback, **kwargs):
        self.callback = callback
        Controller.__init__(self, **kwargs)

    def map_speed(self, value):
        return MapRange(value, -32767, 32767, 0.0, 1.0)

    def map_heading(self, value):
        return MapRange(value, -32767, 32767, -180, 180)

    def on_up_arrow_press(self):
        self.callback.MoveForward(self.callback.movingSpeed)

    def on_down_arrow_press(self):
        self.callback.MoveBackward(self.callback.movingSpeed)

    def on_up_down_arrow_release(self):
        self.callback.Stop()

    def on_R2_press(self, value):
        self.callback.MoveForward(self.map_speed(value))

    def on_L2_press(self, value):
        self.callback.MoveBackward(self.map_speed(value))

    def on_R2_release(self):
        self.callback.Stop()

    def on_L2_release(self):
        self.callback.Stop()

    def on_R3_up(self, value): return
    def on_R3_down(self, value): return
    def on_R3_left(self, value): return
    def on_R3_right(self, value): return
    def on_L3_up(self, value): return
    def on_L3_down(self, value): return
    def on_triangle_press(self): return
    def on_triangle_release(self): return
    def on_square_press(self): return
    def on_square_release(self): return
    def on_right_arrow_press(self): return
    def on_left_right_arrow_release(self): return
    def on_left_arrow_press(self): return
    def on_left_right_arrow_release(self): return
    def on_R1_press(self): return
    def on_R1_release(self): return
    def on_L1_press(self): return
    def on_L1_release(self): return

    def on_L3_right(self, value):
        self.callback.TurnDegree(self.map_heading(value))

    def on_L3_left(self, value):
        self.callback.TurnDegree(self.map_heading(value))

    def on_L3_x_at_rest(self):
        self.callback.Stop()

    def on_L3_y_at_rest(self):
        self.callback.TurnAhead()

    def on_R3_x_at_rest(self):
        self.callback.Stop()

    def on_R3_y_at_rest(self):
        self.callback.TurnAhead()

    def on_x_press(self):
        self.callback.recordNodes = True

    def on_x_release(self):
        self.callback.recordNodes = False
        self.callback.SaveNodes()

    def on_circle_press(self):
        self.callback.followAI = True

    def on_circle_release(self):
        self.callback.followAI = False


# ProjectAI
class ProjectAI:

    # Initializer
    def __init__(self):
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
        try:
            self.movingSpeed = 0.0
            self.movingHeading = 0.0
            self.InputNodes = []
            self.DistanceNodes = []
            self.recordNodes = False
            self.followAI = False

            # Setup GPIO
            self.log('Initializing GPIO Pins')
            self.gpioFactory = PiGPIOFactory()
            self.Motor = Motor(24, 23, pin_factory=self.gpioFactory)
            self.Steering = Servo(18, min_angle=-180, max_angle=180, pin_factory=self.gpioFactory)

            # Setup YDLidar
            self.log('Initializing YDLidar SDK')
            self.lidar = ydlidar.CYdLidar()
            self.lidar.setlidaropt(ydlidar.LidarPropSerialBaudrate, 128000)
            self.lidar.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TRIANGLE)
            self.lidar.setlidaropt(ydlidar.LidarPropDeviceType, ydlidar.YDLIDAR_TYPE_SERIAL)
            self.lidar.setlidaropt(ydlidar.LidarPropSupportMotorDtrCtrl, True)
            self.lidar.setlidaropt(ydlidar.LidarPropSingleChannel, False)
            self.lidar.setlidaropt(ydlidar.LidarPropAutoReconnect, True)
            self.lidar.setlidaropt(ydlidar.LidarPropIntenstiy, False)
            self.lidar.setlidaropt(ydlidar.LidarPropReversion, True)
            self.lidar.setlidaropt(ydlidar.LidarPropScanFrequency, 12.0)
            self.lidar.setlidaropt(ydlidar.LidarPropSampleRate, 5)
            self.lidar.setlidaropt(ydlidar.LidarPropMaxRange, 10.0)
            self.lidar.setlidaropt(ydlidar.LidarPropMinRange, 0.12)
            self.lidar.setlidaropt(ydlidar.LidarPropMaxAngle, 180.0)
            self.lidar.setlidaropt(ydlidar.LidarPropMinAngle, -180.0)
            self.scan = ydlidar.LaserScan()

            # Setup Webserver
            self.log('Initializing Webserver')
            handler_partial = functools.partial(HTTPHelper, callback=self)
            self.webserver = HTTPServer((GetIpAddress(), 8080), handler_partial)
            Thread(target=self.webserver.serve_forever, daemon=True).start()
            self.log(f'WebServer started at {self.webserver.server_address[0]}:{self.webserver.server_address[1]}')
            Thread(target=os.system, args=[f'python3 -m http.server -b {GetIpAddress()}'], daemon=True).start()

            # Setup Console Controller
            self.ConnectController()

            # Connect YDLidar
            if not self.ConnectLidar():
                ProgramRestart()

            self.log(f'''Please control the robot via the URL {self.webserver.server_address[0]}:{self.webserver.server_address[1]} or a PS4 or PS5 controller''')
            self.log('Initialization done!')

        except Exception as e:
            self.log(f'Exception thrown: {e}')

    # Deinitializer
    def __del__(self):
        self.log('Deinitializing')

        # Cleaning up YDLidar SDK
        if hasattr(self, 'lidar'):
            self.lidar.turnOff()
            self.lidar.disconnecting()
            self.log('Deinitialized YDLidar SDK')

        # Cleaning up Webserver
        if hasattr(self, 'webserver'):
            self.webserver.server_close()
            self.log('Deinitialized Webserver')

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
        address = f'{CONTROLLER_ADDRESS}{0}'
        for idx in range(0, 8):
            if os.path.exists(f'{CONTROLLER_ADDRESS}{idx}'):
                address = f'{CONTROLLER_ADDRESS}{idx}'
                break

        # Controller found
        self.controller = ConsoleController(interface=address, connecting_using_ds4drv=False, callback=self)
        Thread(target=self.controller.listen, args=[sys.maxsize], daemon=True).start()

        time.sleep(1)
        if not self.controller.is_connected:
            self.log('No console controller could be found')
            return False
        self.log(f'Connected to controller at {address}')
        return True

    # Moving Robot
    def MoveForward(self, speed):
        self.movingSpeed = Clamp(speed, 0.0, 1.0)
        self.Motor.forward(self.movingSpeed)

    def MoveBackward(self, speed):
        self.movingSpeed = Clamp(speed, 0.0, 1.0)
        self.Motor.backward(self.movingSpeed)

    def Stop(self):
        self.Motor.stop()

    def TurnLeft(self): self.Steering.min()
    def TurnAhead(self): self.Steering.mid()
    def TurnRight(self): self.Steering.max()

    def TurnDegree(self, angle):
        self.Steering.angle = angle = Clamp(angle, -180, 180)
        self.movingHeading = math.radians(angle)

    def TurnRadian(self, angle):
        return self.TurnDegree(math.degrees(angle))

    # Extract robot control data from POST body
    def ParseRobotControl(self, post_data):
        dir = post_data.get('Direction')
        speed = post_data.get('Speed')

        if 'forward' in dir:
            self.MoveForward(speed)
        elif 'backward' in dir:
            self.MoveBackward(speed)
        elif 'left' in dir:
            self.TurnLeft()
        elif 'right' in dir:
            self.TurnRight()
        elif 'stop' in dir:
            self.Stop()
            self.TurnAhead()

    def FilterScansAngle(self, angle):
        aperture = DEFAULT_APERTURE / 2
        results = [scan.range for scan in self.scan.points if math.isclose(angle, scan.angle, rel_tol=aperture)]
        return Average(results)

    # Save Nodes for the A.I. to file
    def SaveNodes(self):
        numpy.save(f'{AI_FOLDER}/DistanceNodes.npy', numpy.asarray(self.DistanceNodes))
        numpy.save(f'{AI_FOLDER}/InputNodes.npy', numpy.asarray(self.InputNodes))
        self.DistanceNodes.clear()
        self.InputNodes.clear()

    # Setup TensorFlow

    def ReadTensors(self):
        self.tfInterpreter = tf.Interpreter(AI_MODEL_NAME)
        self.tfInterpreter.allocate_tensors()

        # Get input and output tensors.
        self.input_details = self.tfInterpreter.get_input_details()
        self.output_details = self.tfInterpreter.get_output_details()
        self.input_shape = self.input_details[0]['shape']

    # Runtime
    def Runtime(self):
        self.ReadTensors()
        while True:
            try:
                if self.lidar.doProcessSimple(self.scan):
                    if self.recordNodes:
                        self.DistanceNodes.append([
                            self.FilterScansAngle(HEADING_LEFT),
                            self.FilterScansAngle(HEADING_TOPLEFT),
                            self.FilterScansAngle(HEADING_AHEAD),
                            self.FilterScansAngle(HEADING_TOPRIGHT),
                            self.FilterScansAngle(HEADING_RIGHT)
                        ])

                        self.InputNodes.append([
                            self.movingSpeed,
                            self.movingHeading / math.pi
                        ])

                    if self.followAI:
                        distanceNodes = numpy.array([
                            [
                                # self.FilterScansAngle(HEADING_LEFT),
                                self.FilterScansAngle(HEADING_TOPLEFT),
                                self.FilterScansAngle(HEADING_AHEAD),
                                self.FilterScansAngle(HEADING_TOPRIGHT),
                                self.FilterScansAngle(HEADING_RIGHT)
                            ]
                        ], dtype=numpy.float32)

                        self.tfInterpreter.set_tensor(self.input_details[0]['index'], distanceNodes)
                        self.tfInterpreter.invoke()
                        tensor = self.tfInterpreter.get_tensor(self.output_details[0]['index'])

                        self.TurnRadian(tensor[0][1])
                        self.MoveForward(tensor[0][0])

                        print(tensor)

                    time.sleep(0.05)

                else:
                    self.log('Failed to get Lidar Data')
                    self.Stop()
                    time.sleep(1)

            except Exception as e:
                self.log(f'Exception thrown: {e}')

    @ staticmethod
    def log(*args):
        print(f'[ProjectAI] {args[0]}')


# Main Entrance
# Catching errors
try:
    # Instantiating Main Application
    s_Robot = ProjectAI()
    # Loop indefinitely
    s_Robot.Runtime()

except KeyboardInterrupt:
    print('Terminating Application')

except Exception as e:
    print(f'Exception thrown: {e}')
