#include "Lidar.h"
#include "Log.h"

#include <glm/glm.hpp>

namespace ProjectAI
{
    Lidar::Lidar()
    {
        ydlidar::os_init();

        // Baudrate
        int i_optval = 128000;
        m_Lidar.setlidaropt(LidarPropSerialBaudrate, &i_optval, sizeof(int));

        // Tof lidar
        i_optval = TYPE_TRIANGLE;
        m_Lidar.setlidaropt(LidarPropLidarType, &i_optval, sizeof(int));

        // Device type
        i_optval = YDLIDAR_TYPE_SERIAL;
        m_Lidar.setlidaropt(LidarPropDeviceType, &i_optval, sizeof(int));

        // Sample rate
        i_optval = 3;
        m_Lidar.setlidaropt(LidarPropSampleRate, &i_optval, sizeof(int));

        // Abnormal count
        i_optval = 4;
        m_Lidar.setlidaropt(LidarPropAbnormalCheckCount, &i_optval, sizeof(int));

        // Intenstiy bit count
        i_optval = 8;
        m_Lidar.setlidaropt(LidarPropIntenstiyBit, &i_optval, sizeof(int));

        // Fixed-angle resolution
        bool b_optvalue = false;
        m_Lidar.setlidaropt(LidarPropFixedResolution, &b_optvalue, sizeof(bool));

        // Rotate 180
        b_optvalue = false;
        m_Lidar.setlidaropt(LidarPropReversion, &b_optvalue, sizeof(bool));

        // Counterclockwise
        b_optvalue = false;
        m_Lidar.setlidaropt(LidarPropInverted, &b_optvalue, sizeof(bool));

        // Auto-reconnect
        b_optvalue = true;
        m_Lidar.setlidaropt(LidarPropAutoReconnect, &b_optvalue, sizeof(bool));

        // One-way communication
        b_optvalue = true;
        m_Lidar.setlidaropt(LidarPropSingleChannel, &b_optvalue, sizeof(bool));

        // Intensity
        b_optvalue = false;
        m_Lidar.setlidaropt(LidarPropIntenstiy, &b_optvalue, sizeof(bool));

        // Motor DTR
        b_optvalue = true;
        m_Lidar.setlidaropt(LidarPropSupportMotorDtrCtrl, &b_optvalue, sizeof(bool));

        // HeartBeat
        b_optvalue = false;
        m_Lidar.setlidaropt(LidarPropSupportHeartBeat, &b_optvalue, sizeof(bool));

        // Unit: Degrees
        float f_optvalue = 180.0f;
        m_Lidar.setlidaropt(LidarPropMaxAngle, &f_optvalue, sizeof(float));
        f_optvalue = -180.0f;
        m_Lidar.setlidaropt(LidarPropMinAngle, &f_optvalue, sizeof(float));

        // Unit: Meters
        f_optvalue = 64.f;
        m_Lidar.setlidaropt(LidarPropMaxRange, &f_optvalue, sizeof(float));
        f_optvalue = 0.05f;
        m_Lidar.setlidaropt(LidarPropMinRange, &f_optvalue, sizeof(float));

        // Unit: Hz
        f_optvalue = 12.0f;
        m_Lidar.setlidaropt(LidarPropScanFrequency, &f_optvalue, sizeof(float));

        m_Lidar.enableGlassNoise(false);
        m_Lidar.enableSunNoise(false);
    }

    Lidar::~Lidar()
    {
        m_Lidar.turnOff();
        m_Lidar.disconnecting();
        ydlidar::os_shutdown();
    }

    bool Lidar::Connect()
    {
        // Search for Lidar devices
        auto lidarPortList = ydlidar::lidarPortList();
        for (auto &port : lidarPortList)
        {
            CORE_INFO("YDLidar device has been found - [Port: {0}, Version: {1}", port.second, port.first);
            m_Lidar.setlidaropt(LidarPropSerialPort, port.second.c_str(), port.second.size());

            // Try to initialize SDK and LiDAR
            if (m_Lidar.initialize() && m_Lidar.turnOn())
                return true;
            else
                CORE_ERROR(m_Lidar.DescribeError());
        }

        return false;
    }

    bool Lidar::Scan()
    {
        if (ydlidar::os_isOk())
        {
            LaserScan scan;
            if (m_Lidar.doProcessSimple(scan))
            {
                CORE_INFO("Scan received: {0} ranges at {1}Hz", scan.points.size(), 1.0 / scan.config.scan_time);

                auto max = std::begin(scan.points);
                for (auto it = scan.points.begin(); it != scan.points.end(); ++it)
                {
                    // CORE_INFO("{0} : {1} ", point.angle * (180 / M_PI), point.range);
                    if (it->range > max->range)
                        max = it;
                }

                CORE_INFO("{0} : {1}", glm::degrees(max->angle), max->range);

                // std::map<float, int> map;
                // map[]

                return true;
            }
            else
                CORE_ERROR("Failed to get Lidar Data");
        }

        return false;
    }
}