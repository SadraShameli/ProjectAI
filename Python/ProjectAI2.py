import os
import sys
import time
import math
import json
import numpy
import signal
import functools
from threading import Thread

# External Libraries
import ydlidar

# Constants
CURRENT_FOLDER = os.path.dirname(os.path.realpath(__file__))
PROJECT_FOLDER = os.path.dirname(CURRENT_FOLDER)
HEADING_LEFT = math.pi * -1.0
HEADING_TOPLEFT = math.pi * -0.75
HEADING_AHEAD = math.pi * -0.5
HEADING_TOPRIGHT = math.pi * -0.25
HEADING_RIGHT = math.pi * 0.0

# A.I.
AI_FOLDER = f'{CURRENT_FOLDER}/AI'
AI_MODEL_NAME = f'{AI_FOLDER}/model.tflite'
DEFAULT_APERTURE = math.radians(1)


def Clamp(n, smallest, largest):
    return max(smallest, min(n, largest))

def MapRange(x,  in_min,  in_max,  out_min,  out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

# ProjectAI
class ProjectAI:   
    _Running = True
    
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
            self.MovingSpeed = 0.0
            self.MovingHeading = 0.0
            self.InputNodes = []
            self.DistanceNodes = []
            self.RecordNodes = False
            self.FollowAI = True
            self.FollowMaxDistance = False    

            # Setup YDLidar
            self.log('Initializing YDLidar SDK')
            self.lidar = ydlidar.CYdLidar()
            self.lidar.setlidaropt(ydlidar.LidarPropSerialBaudrate, 128000)        
            self.lidar.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TRIANGLE)
            self.lidar.setlidaropt(ydlidar.LidarPropDeviceType, ydlidar.YDLIDAR_TYPE_SERIAL)        
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

            while not self.ConnectLidar():
                pass

            numpy.set_printoptions(formatter={'float': lambda x: "{0:0.3f}".format(x)})

            self.log('Initialization done!')          
            self.Runtime()            
            self.log('Runtime finished')
            
            
        except KeyboardInterrupt:
            self.log('Terminating Application')
            ProjectAI._Running = False

        except Exception as e:
            self.log(f'Exception thrown: {e}')
        
        finally:
            self.Terminate()
            

    # Deinitializer
    def __del__(self):
        self.log('Deinitializing')

                   
    # Called once application is terminated
    def Terminate(self) :
        # Cleaning up YDLidar SDK
        if hasattr(self, 'lidar'):
            self.lidar.turnOff()
            self.lidar.disconnecting()
            self.log('Deinitialized YDLidar SDK')


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


    # Filter scans based on angle, returns a distance
    def FilterScansAngle(self, angle):
        with open("scan.txt", "w") as f:
            f.writelines([str(math.degrees(scan.angle)) + '\n' for scan in self.scan.points])
        results = [scan.range for scan in self.scan.points if scan.angle - DEFAULT_APERTURE <= angle <= scan.angle + DEFAULT_APERTURE]
        if len(results):
            return numpy.mean(results)                
        return 0.0
    
    
    # Filter scans based on maximum distance, returns a scan
    def FilterScansMaxDistance(self):
        return max(self.scan.points, key=lambda scan: scan.range)


    # Save Nodes for the A.I. to file
    def SaveNodes(self):
        numpy.save(f'{AI_FOLDER}/DistanceNodes.npy', numpy.asarray(self.DistanceNodes))
        numpy.save(f'{AI_FOLDER}/InputNodes.npy', numpy.asarray(self.InputNodes))
        self.DistanceNodes.clear()
        self.InputNodes.clear()


    # Runtime
    def Runtime(self):
        while ProjectAI._Running:            
            if self.lidar.doProcessSimple(self.scan):
                
                if self.FollowMaxDistance:
                    scan = self.FilterScansMaxDistance()
                    angle = math.degrees(scan.angle)
                    self.log(f'{scan.range} | {angle}')
                    self.TurnDegree(angle)
                    self.MoveForward(self.MovingSpeed)                                       
                    
                if self.RecordNodes:
                    self.DistanceNodes.append([
                        self.FilterScansAngle(HEADING_LEFT),
                        self.FilterScansAngle(HEADING_TOPLEFT),
                        self.FilterScansAngle(HEADING_AHEAD),
                        self.FilterScansAngle(HEADING_TOPRIGHT),
                        self.FilterScansAngle(HEADING_RIGHT)
                    ])

                    self.InputNodes.append([
                        self.MovingSpeed,
                        self.MovingHeading / math.pi
                    ])

                if self.FollowAI:
                    distanceNodes = numpy.array([
                        [
                            self.FilterScansAngle(HEADING_LEFT),
                            self.FilterScansAngle(HEADING_TOPLEFT),
                            self.FilterScansAngle(HEADING_AHEAD),
                            self.FilterScansAngle(HEADING_TOPRIGHT),
                            self.FilterScansAngle(HEADING_RIGHT)
                        ]
                    ], dtype=numpy.float32)
                    
                    print(distanceNodes)
                    
            else:
                self.log('Lidar failed')
                time.sleep(1)


    @staticmethod
    def log(*args):
        print(f'[ProjectAI] {args[0]}')



# Main Entrance
robot = ProjectAI()    