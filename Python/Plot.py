import math
from time import sleep

import ydlidar
import matplotlib.pyplot as plt
import matplotlib.animation as animation

RMAX = 32.0

fig = plt.figure()
fig.canvas.manager.set_window_title('YDLidar LIDAR Monitor')
lidar_polar = plt.subplot(polar=True)
lidar_polar.autoscale_view(True, True, True)
lidar_polar.set_rmax(RMAX)
lidar_polar.grid(True)

lidar = ydlidar.CYdLidar()
lidar.setlidaropt(ydlidar.LidarPropSerialBaudrate, 128000)
lidar.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TRIANGLE)
lidar.setlidaropt(ydlidar.LidarPropDeviceType, ydlidar.YDLIDAR_TYPE_SERIAL)
lidar.setlidaropt(ydlidar.LidarPropAutoReconnect, True)
lidar.setlidaropt(ydlidar.LidarPropAbnormalCheckCount, 4)
lidar.setlidaropt(ydlidar.LidarPropScanFrequency, 12.0)
lidar.setlidaropt(ydlidar.LidarPropSampleRate, 5)
lidar.setlidaropt(ydlidar.LidarPropMaxRange, 10.0)
lidar.setlidaropt(ydlidar.LidarPropMinRange, 0.12)
scan = ydlidar.LaserScan()


def animate(num):
    r = lidar.doProcessSimple(scan)
    if r:
        angle = []
        ran = []
        intensity = []
        for point in scan.points:
            angle.append(point.angle)
            ran.append(point.range)
            intensity.append(point.intensity)
        lidar_polar.clear()
        lidar_polar.scatter(angle, ran, alpha=0.95)


def ConnectLidar():
    for version, port in ydlidar.lidarPortList().items():
        print(f'YDLidar device has been found - [Port: {port}, Version: {version}]')
        
        # Try to initialize SDK and LiDAR
        lidar.setlidaropt(ydlidar.LidarPropSerialPort, port)
        if lidar.initialize() and lidar.turnOn():
            return True


def Runtime():
    while True:
        ani = animation.FuncAnimation(fig, animate, interval=5)
        plt.show()


try:
    while not ConnectLidar():
        sleep(1)
    Runtime()

except KeyboardInterrupt:
    print('Terminating Application')

finally:
    lidar.turnOff()
    lidar.disconnecting()
    plt.close()
