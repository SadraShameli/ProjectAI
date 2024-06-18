import os
import sys
import cv2
import glob
import time
import math
import json
import numpy
import signal
import functools
from urllib import parse
from datetime import datetime
from threading import Thread
from multiprocessing import Process

# External Libraries
import ydlidar
import netifaces as ni
import tflite_runtime.interpreter as tf
from picamera2 import Picamera2
from gpiozero import Motor
from gpiozero import AngularServo as Servo
from gpiozero.pins.pigpio import PiGPIOFactory
from http.server import BaseHTTPRequestHandler, HTTPServer
from pyPS4Controller.controller import Controller

# Constants
CURRENT_FOLDER = os.path.dirname(os.path.realpath(__file__))
PROJECT_FOLDER = os.path.dirname(CURRENT_FOLDER)
WEBSERVER_FOLDER = f"{PROJECT_FOLDER}/WebServer"
IMAGES_FOLDER = f"{PROJECT_FOLDER}/Captured Images"
CONTROLLER_ADDRESS = "/dev/input/js"
HTTP_SERVER_PORT = 8080
DATETIME_FORMATE = "%B %d, %Y - %H·%M·%S"

# Movement
HEADING_LEFT = math.pi * 0.0
HEADING_HIGHERLEFT = math.pi * 0.2
HEADING_TOPLEFT = math.pi * 0.4
HEADING_AHEAD = math.pi * 0.5
HEADING_TOPRIGHT = math.pi * 0.6
HEADING_HIGHERRIGHT = math.pi * 0.8
HEADING_RIGHT = math.pi * 1.0
SPEED_MULTIPLIER = 0.05

# Camera resolution - 3280 * 2464
CAMERA_WIDTH = 3280
CAMERA_HEIGHT = 2464

# A.I.
AI_FOLDER = f"{CURRENT_FOLDER}/AI"
AI_FOLDER_IMAGES = f"{AI_FOLDER}/Images"
AI_MODEL_CAMERA_FILE = f"{AI_FOLDER}/Camera.tflite"
AI_MODEL_LIDAR_FILE = f"{AI_FOLDER}/Lidar.tflite"
AI_IMAGE_NODES_FILE = f"{AI_FOLDER}/ImageNodes.csv"
AI_DISTANCE_NODES_FILE = f"{AI_FOLDER}/DistanceNodes.npy"
AI_INPUT_NODES_FILE = f"{AI_FOLDER}/InputNodes.npy"
DEFAULT_APERTURE = math.radians(5)
LIDAR_RANGE_MIN = 0.12
LIDAR_RANGE_MAX = 10.0
LIDAR_ANGLE_MIN = -180.0
LIDAR_ANGLE_MAX = 180.0
USE_CAMERA = False


# Helper Functions
# Low Level
def SignalHandler(sig, frame):
    ProjectAI._Running = False


def ProgramRestart():
    Process(target=os.execv, args=[sys.executable, ["python"] + sys.argv]).start()
    os._exit(0)


def KillProcesses(cmd):
    listProcesses = os.popen(cmd).read()
    for process in listProcesses.splitlines():
        os.system(f"kill -9 {process}")


def DeleteFile(path):
    if os.path.exists(path):
        os.remove(path)


def GetIpAddress():
    return ni.ifaddresses("wlan0")[ni.AF_INET][0]["addr"]


def DeleteAllFilesInFolder(folder):
    files = glob.glob(f"{folder}/*")
    for file in files:
        os.remove(file)


# Math
def Clamp(n, smallest, largest):
    return max(smallest, min(n, largest))


def MapRange(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


# Helper Classes
class ScopedTimer:

    # Initializer
    def __init__(self, name):
        self.Name = name
        self.StartTime = time.perf_counter()

    # Deinitializer
    def __del__(self):
        elapsedTime = time.perf_counter() - self.StartTime
        self.log(f"{self.Name} took {elapsedTime * 1000.0:0.4f} ms")

    @staticmethod
    def log(*args):
        print(f"[ScopedTimer] {args[0]}")


# HTTP Helper
class HTTPHelper(BaseHTTPRequestHandler):

    # Overwriting initializer to accept a callback
    def __init__(self, *args, callback=None, **kwargs):
        self.callback = callback
        super().__init__(*args, **kwargs)

    # Handle GET requests
    def do_GET(self):
        try:
            queryParameters = dict(parse.parse_qsl(parse.urlsplit(self.path).query))

            if not queryParameters:
                if self.path == "/":
                    self.path = "/index.html"

                if "." in self.path:
                    with open(f"{WEBSERVER_FOLDER}{self.path}", "rb") as f:
                        self.send_200()
                        self.wfile.write(f.read())

                        self.log(f"File served: {self.path}")

                else:
                    self.send_404()
                    return

            else:
                command = queryParameters.get("command")

                if command == "Terminate":
                    self.send_200()
                    ProjectAI._Running = False

                elif command == "Restart":
                    self.send_200()
                    ProgramRestart()

                elif command == "Reboot":
                    self.send_200()
                    os.system("sudo reboot")

                elif command == "CaptureImage":
                    self.callback.CaptureImage()

                elif command == "FollowAI":
                    self.callback.FollowAI = True

                elif command == "FollowMaxDistance":
                    self.callback.FollowMaxDistance = True

                elif command == "RecordNodes":
                    self.callback.RecordNodes = True

                elif command == "StopRecordingNodes":
                    self.callback.RecordNodes = False
                    self.callback.SaveNodes()

                elif command == "Stop":
                    self.callback.RecordNodes = False
                    self.callback.FollowAI = False
                    self.callback.FollowMaxDistance = False
                    self.callback.Stop()
                    self.callback.ClearNodes()

                else:
                    self.send_404()
                    return

                self.send_200()

        except FileNotFoundError as e:
            self.send_response(404)
            self.end_headers()
            self.log(f"File or directory not found: {e.filename}")

        except Exception as e:
            self.send_response(500)
            self.end_headers()
            self.log(f"Exception thrown: {e}")

    # Handle POST requests
    def do_POST(self):
        try:
            content_length = int(self.headers["Content-Length"])
            post_data = json.loads(self.rfile.read(content_length))
            queryParameters = dict(parse.parse_qsl(parse.urlsplit(self.path).query))

            command = queryParameters.get("command")
            if command == "MoveRobot":
                movementProperty = ["Direction", "Speed"]
                if all(dir in post_data for dir in movementProperty):
                    self.callback.ParseRobotControl(post_data)

            else:
                self.send_404()
                return

            self.send_response(200)
            self.end_headers()

        except Exception as e:
            self.send_response(500)
            self.end_headers()
            self.log(f"Exception thrown: {e}")

    def send_200(self):
        self.send_response(200)
        self.end_headers()

    def send_404(self):
        self.send_response(404)
        self.end_headers()
        self.log(f"URL {self.path} is not implemented")

    def log_message(self, format, *args):
        return

    @staticmethod
    def log(*args):
        print(f"[HTTPServer] {args[0]}")


# ConsoleController Helper
class ConsoleController(Controller):

    # Initializer
    def __init__(self, callback, **kwargs):
        Controller.__init__(self, **kwargs)
        self.callback = callback

    def map_speed(self, value):
        return MapRange(value, -32767, 32767, 0.0, 1.0)

    def map_heading(self, value):
        return MapRange(value, -32767, 32767, 0, 180)

    def on_up_arrow_press(self):
        if self.callback.MovingSpeed < 1.0:
            self.callback.MovingSpeed += SPEED_MULTIPLIER

    def on_down_arrow_press(self):
        if self.callback.MovingSpeed > 0.0:
            self.callback.MovingSpeed -= SPEED_MULTIPLIER

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
        self.callback.RecordNodes = True

    def on_x_release(self):
        self.callback.SaveNodes()
        self.callback.RecordNodes = False

    def on_circle_press(self):
        self.callback.FollowAI = True

    def on_circle_release(self):
        self.callback.FollowAI = False

    def on_square_press(self):
        self.callback.FollowMaxDistance = True

    def on_square_release(self):
        self.callback.FollowMaxDistance = False

    def on_triangle_press(self):
        self.callback.FollowAI = False
        self.callback.FollowMaxDistance = False
        self.callback.RecordNodes = False
        self.callback.Stop()
        self.callback.ClearNodes()

    # These are neither required nor handled
    def on_R1_press(self):
        return

    def on_R1_release(self):
        return

    def on_L1_press(self):
        return

    def on_L1_release(self):
        return

    def on_R3_up(self, value):
        return

    def on_R3_down(self, value):
        return

    def on_R3_left(self, value):
        return

    def on_R3_right(self, value):
        return

    def on_L3_up(self, value):
        return

    def on_L3_down(self, value):
        return

    def on_R3_press(self):
        return

    def on_R3_release(self):
        return

    def on_L3_press(self):
        return

    def on_L3_release(self):
        return

    def on_triangle_release(self):
        return

    def on_right_arrow_press(self):
        return

    def on_left_arrow_press(self):
        return

    def on_left_right_arrow_release(self):
        return

    def on_playstation_button_press(self):
        return

    def on_playstation_button_release(self):
        return

    def on_options_press(self):
        return

    def on_options_release(self):
        return

    def on_share_press(self):
        return

    def on_share_release(self):
        return


# ProjectAI
class ProjectAI:
    _Running = True

    # Initializer
    def __init__(self):
        print(
            """
----------------------------------------------------
Welcome to
 ____            _           _        _      ___
|  _ \ _ __ ___ (_) ___  ___| |_     / \    |_ _|
| |_) | '__/ _ \| |/ _ \/ __| __|   / _ \    | |
|  __/| | | (_) | |  __/ (__| |_   / ___ \ _ | | _
|_|   |_|  \___// |\___|\___|\__| /_/   \_(_)___(_)

Made with \u2665 By Sadra Shameli
----------------------------------------------------
"""
        )
        try:
            self.MovingSpeed = 0.0
            self.MovingHeading = HEADING_AHEAD
            self.InputNodes = []
            self.DistanceNodes = []
            self.RecordNodes = False
            self.FollowAI = False
            self.FollowMaxDistance = False

            # Setup GPIO
            self.log("Initializing GPIO Pins")
            self.gpioFactory = PiGPIOFactory()
            self.Motor = Motor(24, 23, pin_factory=self.gpioFactory)
            self.Steering = Servo(18, pin_factory=self.gpioFactory)

            # Setup Signal Handler to catch events
            signal.signal(signal.SIGINT, SignalHandler)

            # Setup YDLidar
            self.log("Initializing YDLidar SDK")
            self.lidar = ydlidar.CYdLidar()
            self.lidar.setlidaropt(ydlidar.LidarPropSerialBaudrate, 128000)
            self.lidar.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TRIANGLE)
            self.lidar.setlidaropt(
                ydlidar.LidarPropDeviceType, ydlidar.YDLIDAR_TYPE_SERIAL
            )
            self.lidar.setlidaropt(ydlidar.LidarPropIgnoreArray, "")
            self.lidar.setlidaropt(ydlidar.LidarPropSupportMotorDtrCtrl, True)
            self.lidar.setlidaropt(ydlidar.LidarPropFixedResolution, True)
            self.lidar.setlidaropt(ydlidar.LidarPropSingleChannel, True)
            self.lidar.setlidaropt(ydlidar.LidarPropAutoReconnect, True)
            self.lidar.setlidaropt(ydlidar.LidarPropIntenstiy, False)
            self.lidar.setlidaropt(ydlidar.LidarPropReversion, False)
            self.lidar.setlidaropt(ydlidar.LidarPropInverted, False)
            self.lidar.setlidaropt(ydlidar.LidarPropAbnormalCheckCount, 4)
            self.lidar.setlidaropt(ydlidar.LidarPropScanFrequency, 12.0)
            self.lidar.setlidaropt(ydlidar.LidarPropIntenstiyBit, 0)
            self.lidar.setlidaropt(ydlidar.LidarPropSampleRate, 5)
            self.lidar.setlidaropt(ydlidar.LidarPropMinRange, LIDAR_RANGE_MIN)
            self.lidar.setlidaropt(ydlidar.LidarPropMaxRange, LIDAR_RANGE_MAX)
            self.lidar.setlidaropt(ydlidar.LidarPropMinAngle, LIDAR_ANGLE_MIN)
            self.lidar.setlidaropt(ydlidar.LidarPropMaxAngle, LIDAR_ANGLE_MAX)
            self.scan = ydlidar.LaserScan()

            # Setup camera
            self.log(
                f"Initializing Camera with resolution {CAMERA_WIDTH} * {CAMERA_HEIGHT}"
            )
            KillProcesses("lsof -t /dev/video0")
            self.camera = Picamera2()
            self.camera_config = self.camera.create_still_configuration(
                main={"size": (CAMERA_WIDTH, CAMERA_HEIGHT)}
            )
            self.camera.configure(self.camera_config)
            self.camera.start()
            self.InputImage = self.camera.capture_array()
            self.ImageNodes = []
            self.ImageIndex = 0

            # Connect YDLidar
            if not self.ConnectLidar():
                sys.exit()

            # Setup Console Controller
            Thread(target=self.ConnectController, daemon=True).start()

            # Setup numpy
            numpy.set_printoptions(formatter={"float": lambda x: "{0:0.3f}".format(x)})

            # Setup TensorFlow tensors, model and neural network
            self.log("Initializing TensorFlow")
            self.SetTensorFlow()

            # Setup web server
            self.log("Initializing web server")
            self.StartWebServer()

            # Initialization Finished
            self.log(
                "Please control the robot via the created web server or a PS4 or PS5 controller"
            )
            self.log("Initialization done!")

            self.Runtime()
            self.log("Runtime finished")

        except Exception as e:
            self.log(f"Exception thrown: {e}")

        finally:
            self.log("Terminating Application")
            self.Terminate()

    # Destructor
    def __del__(self):
        self.log("Destructing ProjectAI")

    # Called once application is terminated
    def Terminate(self):

        # Cleaning up YDLidar SDK
        if hasattr(self, "lidar"):
            self.lidar.turnOff()
            self.lidar.disconnecting()
            self.log("Destructor YDLidar SDK")

        # Cleaning up Webserver
        if hasattr(self, "webserver"):
            self.webserver.shutdown()
            self.webserver.server_close()
            self.log("Destructor web server")

        # Cleaning up camera
        if hasattr(self, "camera"):
            self.camera.stop()
            self.log("Destructor camera")

    # Search for YDLidar devices
    def ConnectLidar(self):

        portList = ydlidar.lidarPortList().items()
        if len(portList) == 0:
            self.log("No YDLidar device could be found")
            return False

        # Iterate through all YDLidar devices
        for version, port in portList:
            # YDLidar Found
            self.log(
                f"YDLidar device has been found - Port: {port}, Version: {version}"
            )

            # Try to Initialize YDLidar SDK
            self.lidar.setlidaropt(ydlidar.LidarPropSerialPort, port)
            if self.lidar.initialize() and self.lidar.turnOn():
                return True
            else:
                self.log(f"Failed to connect to YDLidar at {port}")
                return False

    # Search for Console Controller
    def ConnectController(self):
        # Iterate through all joystick interfaces
        address = f"{CONTROLLER_ADDRESS}{0}"
        for idx in range(0, 8):
            if os.path.exists(f"{CONTROLLER_ADDRESS}{idx}"):
                address = f"{CONTROLLER_ADDRESS}{idx}"
                break

        # Controller found
        self.controller = ConsoleController(
            interface=address, connecting_using_ds4drv=False, callback=self
        )
        Thread(target=self.controller.listen, args=[sys.maxsize], daemon=True).start()

        time.sleep(0.25)
        if self.controller.is_connected:
            self.log(f"Connected to controller at {address}")
            return True

        self.log("No console controller could be found")
        return False

    # Start Webserver
    def StartWebServer(self):

        # Check whether the ports are already open
        KillProcesses(f"lsof -t -i:{HTTP_SERVER_PORT};lsof -t -i:8000")

        self.webserver = HTTPServer(
            (GetIpAddress(), HTTP_SERVER_PORT),
            functools.partial(HTTPHelper, callback=self),
        )
        Thread(target=self.webserver.serve_forever, daemon=True).start()
        Thread(
            target=os.system,
            args=[f"python3 -m http.server -b {GetIpAddress()}"],
            daemon=True,
        ).start()
        self.log(
            f"WebServer started at {GetIpAddress()}:{self.webserver.server_address[1]} and {GetIpAddress()}:8000"
        )

    # Moving Robot
    def MoveForward(self, speed):
        self.MovingSpeed = Clamp(speed, 0.0, 1.0)
        self.Motor.forward(self.MovingSpeed)

    def MoveBackward(self, speed):
        self.MovingSpeed = Clamp(speed, 0.0, 1.0)
        self.Motor.backward(self.MovingSpeed)

    def Stop(self):
        self.Motor.stop()
        self.MovingSpeed = 0

    # Giving Directions
    def TurnLeft(self):
        self.Steering.min()

    def TurnAhead(self):
        self.Steering.mid()

    def TurnRight(self):
        self.Steering.max()

    # 0.0 to 180.0 (full left : full right) -> mapped to -90.0 : 90.0
    def TurnDegree(self, angle):
        self.Steering.angle = angle = Clamp(angle - 90.0, -90.0, 90.0)
        self.MovingHeading = math.radians(angle)

    # 0.0 to pi (full left : full right)
    def TurnRadian(self, angle):
        return self.TurnDegree(math.degrees(angle))

    # Extract robot control data from POST body
    def ParseRobotControl(self, post_data):
        dir = post_data.get("Direction")
        self.MovingSpeed = MapRange(post_data.get("Speed"), 0, 100, 0, 1)

        if "forward" in dir:
            self.MoveForward(self.MovingSpeed)
        elif "backward" in dir:
            self.MoveBackward(self.MovingSpeed)
        elif "left" in dir:
            self.TurnLeft()
        elif "right" in dir:
            self.TurnRight()
        elif "stop" in dir:
            self.Stop()
            self.TurnAhead()

    # Generates an image
    def CaptureImage(self):
        fileName = datetime.now().strftime(DATETIME_FORMATE)
        self.camera.capture_file(f"{IMAGES_FOLDER}/{fileName}.jpg")

    # Correct scan angle based on sensor rotation
    def CorrectScanAngle(self, angle):
        return angle - HEADING_AHEAD

    # Filter scans based on angle, returns a distance
    def FilterScansAngle(self, angle):
        results = [
            MapRange(scan.range, LIDAR_RANGE_MIN, LIDAR_RANGE_MAX, 0.0, 1.0)
            for scan in self.scan.points
            if math.isclose(scan.angle, angle, rel_tol=DEFAULT_APERTURE)
        ]
        if len(results):
            return numpy.mean(results)
        return 0.0

    # Filter scans based on maximum distance, returns a scan
    def FilterScansMaxDistance(self):
        results = [
            scan
            for scan in self.scan.points
            if HEADING_LEFT <= scan.angle <= HEADING_RIGHT
        ]
        return max(results, key=lambda scan: scan.range)

    # Save Nodes for the A.I. to file
    def SaveNodes(self):
        if USE_CAMERA:
            with open(AI_IMAGE_NODES_FILE, "w") as file:
                for node in self.ImageNodes:
                    file.write(f"{node}\n")

        else:
            numpy.save(AI_DISTANCE_NODES_FILE, numpy.asarray(self.DistanceNodes))
            numpy.save(AI_INPUT_NODES_FILE, numpy.asarray(self.InputNodes))

    def ClearNodes(self):
        if USE_CAMERA:
            self.ImageIndex = 0
            self.ImageNodes.clear()
            self.InputNodes.clear()
            DeleteFile(AI_IMAGE_NODES_FILE)
            DeleteAllFilesInFolder(AI_FOLDER_IMAGES)

        else:
            self.DistanceNodes.clear()
            DeleteFile(AI_DISTANCE_NODES_FILE)
            DeleteFile(AI_INPUT_NODES_FILE)

    # Setup TensorFlow
    def SetTensorFlow(self):
        if USE_CAMERA:
            self.tfInterpreter = tf.Interpreter(AI_MODEL_CAMERA_FILE)
        else:
            self.tfInterpreter = tf.Interpreter(AI_MODEL_LIDAR_FILE)
        self.tfInterpreter.allocate_tensors()

        # Get input and output tensors.
        self.input_details = self.tfInterpreter.get_input_details()
        self.output_details = self.tfInterpreter.get_output_details()
        self.input_shape = self.input_details[0]["shape"]

    def ProcessImage(self, image):
        image = cv2.resize(image, (250, 250))
        height, _, _ = image.shape
        image = image[int(height / 2) :, :, :]
        image = cv2.GaussianBlur(image, (5, 5), 0)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        image = cv2.Canny(image, 50, 100)
        image = cv2.resize(image, (250, 250))
        image = image.reshape(250, 250, 1)
        image = image / 255  # normalizing
        return image

    # Runtime
    def Runtime(self):
        while self._Running:
            if not USE_CAMERA and not self.lidar.doProcessSimple(self.scan):
                self.Stop()
                time.sleep(0.5)
                continue

            if self.FollowAI:
                if USE_CAMERA:
                    self.InputImage = self.camera.capture_array()
                    self.InputImage = self.ProcessImage(self.InputImage)

                    self.InputImage = numpy.array(
                        [self.InputImage], dtype=numpy.float32
                    )
                    self.tfInterpreter.set_tensor(
                        self.input_details[0]["index"], self.InputImage
                    )
                    self.tfInterpreter.invoke()
                    tensor = self.tfInterpreter.get_tensor(
                        self.output_details[0]["index"]
                    )
                    node = tensor[0]
                    # print(node)

                    heading = MapRange(node[0], -1.0, 1.0, HEADING_LEFT, HEADING_RIGHT)
                    self.TurnRadian(heading)
                    # self.MoveForward(node[1])

                else:
                    distanceNodes = numpy.array(
                        [
                            [
                                self.FilterScansAngle(HEADING_LEFT),
                                self.FilterScansAngle(HEADING_HIGHERLEFT),
                                self.FilterScansAngle(HEADING_TOPLEFT),
                                self.FilterScansAngle(HEADING_TOPRIGHT),
                                self.FilterScansAngle(HEADING_HIGHERRIGHT),
                                self.FilterScansAngle(HEADING_RIGHT),
                            ]
                        ],
                        dtype=numpy.float32,
                    )

                    self.tfInterpreter.set_tensor(
                        self.input_details[0]["index"], distanceNodes
                    )
                    self.tfInterpreter.invoke()
                    tensor = self.tfInterpreter.get_tensor(
                        self.output_details[0]["index"]
                    )
                    node = tensor[0]

                    heading = MapRange(node[1], -1.0, 1.0, HEADING_LEFT, HEADING_RIGHT)
                    self.TurnRadian(heading)
                    # self.MoveForward(node[0])

            if self.RecordNodes:
                if USE_CAMERA:
                    self.camera.capture_file(
                        f"{AI_FOLDER_IMAGES}/{self.ImageIndex}.jpg"
                    )
                    self.ImageNodes.append(
                        f"Images/img{self.ImageIndex}.jpg, {self.MovingHeading}, {self.MovingSpeed}"
                    )
                    self.ImageIndex += 1
                else:
                    self.DistanceNodes.append(
                        [
                            self.FilterScansAngle(HEADING_LEFT),
                            self.FilterScansAngle(HEADING_HIGHERLEFT),
                            self.FilterScansAngle(HEADING_TOPLEFT),
                            self.FilterScansAngle(HEADING_TOPRIGHT),
                            self.FilterScansAngle(HEADING_HIGHERRIGHT),
                            self.FilterScansAngle(HEADING_RIGHT),
                        ]
                    )
                    self.InputNodes.append(
                        [self.MovingSpeed, self.MovingHeading / math.pi]
                    )

            if self.FollowMaxDistance:
                scan = self.FilterScansMaxDistance()
                self.TurnRadian(self.CorrectScanAngle(scan.angle))

            time.sleep(0.75)

    @staticmethod
    def log(*args):
        print(f"[ProjectAI] {args[0]}")


# Main Entrance
robot = ProjectAI()
