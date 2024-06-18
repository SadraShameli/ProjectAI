import os
import cv2
import time
import signal

# External Libraries
from picamera2 import Picamera2

# Camera resolution - 3280 * 2464
CAMERA_WIDTH = 1280
CAMERA_HEIGHT = 720


# Helper Functions
# Low Level
def SignalHandler(sig, frame):
    ProjectAI._Running = False


def KillProcesses(cmd):
    listProcesses = os.popen(cmd).read()
    for process in listProcesses.splitlines():
        os.system(f"kill -9 {process}")


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
            # Setup Signal Handler to catch events
            signal.signal(signal.SIGINT, SignalHandler)

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

            # Initialization Finished
            self.log("Initialization done!")

            self.Runtime()
            self.log("Runtime finished")

        except Exception as e:
            self.log(f"Exception thrown: {e}")

        finally:
            self.log("Terminating Application")
            self.Terminate()

    # Deinitializer
    def __del__(self):
        self.log("Deinitializing")

    # Called once application is terminated
    def Terminate(self):
        # Cleaning up camera
        if hasattr(self, "camera"):
            self.camera.stop()
            self.log("Deinitialized camera")

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
            image = self.camera.capture_array()

            cv2.imshow("Project A.I. Main - Press Q to exit", image)
            image = self.ProcessImage(image)
            cv2.imshow("Project A.I. Processed Image", image)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                self._Running = False

    @staticmethod
    def log(*args):
        print(f"[ProjectAI] {args[0]}")


# Main Entrance
robot = ProjectAI()
