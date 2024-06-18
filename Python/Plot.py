import sys
import time
import signal

import ydlidar
import matplotlib.pyplot as plt
import matplotlib.animation as animation


# Helper Functions
# Low Level
def SignalHandler(sig, frame):
    ProjectAI._Running = False


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
            self.lidar.setlidaropt(ydlidar.LidarPropMaxRange, 10.0)
            self.lidar.setlidaropt(ydlidar.LidarPropMinRange, 0.12)
            self.lidar.setlidaropt(ydlidar.LidarPropMaxAngle, 180.0)
            self.lidar.setlidaropt(ydlidar.LidarPropMinAngle, -180.0)
            self.scan = ydlidar.LaserScan()

            # Connect YDLidar
            if not self.ConnectLidar():
                sys.exit()

            # Animation setup
            self.figure = plt.figure()
            self.lidar_polar = plt.subplot(polar=True)
            self.lidar_polar.autoscale_view(True, True, True)

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

        # Cleaning up Animation
        plt.close()
        self.log("Deinitialized Animation")

        # Cleaning up YDLidar SDK
        if hasattr(self, "lidar"):
            self.lidar.turnOff()
            self.lidar.disconnecting()
            self.log("Deinitialized YDLidar SDK")

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

    # Plot the lidar output
    def Animate(self, num):
        if self.lidar.doProcessSimple(self.scan):
            angle = []
            range = []
            intensity = []
            for point in self.scan.points:
                angle.append(point.angle)
                range.append(point.range)
                intensity.append(point.intensity)
            self.lidar_polar.clear()
            self.lidar_polar.scatter(angle, range, c=intensity, cmap="hsv", alpha=0.95)

            # Runtime

    def Runtime(self):
        while self._Running:
            anim = animation.FuncAnimation(self.figure, self.Animate, interval=50)
            plt.show()
            time.sleep(sys.maxsize)

    @staticmethod
    def log(*args):
        print(f"[ProjectAI] {args[0]}")


# Main Entrance
robot = ProjectAI()
