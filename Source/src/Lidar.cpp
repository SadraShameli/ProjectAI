#include "Lidar.h"
#include "Log.h"
#include "Driver.h"

namespace ProjectAI
{
    void Lidar::Init()
    {
        // Baudrate
        int i_optval = 128000;
        m_Lidar.setlidaropt(LidarPropSerialBaudrate, &i_optval, sizeof(int));

        // Triangle Lidar
        i_optval = TYPE_TRIANGLE;
        m_Lidar.setlidaropt(LidarPropLidarType, &i_optval, sizeof(int));

        // Device Type
        i_optval = YDLIDAR_TYPE_SERIAL;
        m_Lidar.setlidaropt(LidarPropDeviceType, &i_optval, sizeof(int));

        // Sample Rate
        i_optval = 5;
        m_Lidar.setlidaropt(LidarPropSampleRate, &i_optval, sizeof(int));

        // Auto Reconnect
        bool b_optvalue = true;
        m_Lidar.setlidaropt(LidarPropAutoReconnect, &b_optvalue, sizeof(bool));

        // One-way Communication
        b_optvalue = true;
        m_Lidar.setlidaropt(LidarPropSingleChannel, &b_optvalue, sizeof(bool));

        // Unit: Meters
        float f_optvalue = 10.0f;
        m_Lidar.setlidaropt(LidarPropMaxRange, &f_optvalue, sizeof(float));
        f_optvalue = 0.12f;
        m_Lidar.setlidaropt(LidarPropMinRange, &f_optvalue, sizeof(float));

        // Unit: Hz
        f_optvalue = 12.0f;
        m_Lidar.setlidaropt(LidarPropScanFrequency, &f_optvalue, sizeof(float));
    }

    void Lidar::Destroy()
    {
        m_Lidar.turnOff();
        m_Lidar.disconnecting();
    }

    bool Lidar::Connect()
    {
        // Search for YDLidar devices
        auto lidarPortList = ydlidar::lidarPortList();
        for (auto &port : lidarPortList)
        {
            CORE_INFO("YDLidar device has been found - [Port: {0}, Version: {1}]", port.second, port.first);
            m_Lidar.setlidaropt(LidarPropSerialPort, port.second.c_str(), port.second.size());

            // Try to Initialize YDLidar SDK
            if (m_Lidar.initialize() && m_Lidar.turnOn())
                return true;
            else
                CORE_ERROR(m_Lidar.DescribeError());
        }

        return false;
    }

    bool Lidar::Scan()
    {
        if (m_Lidar.doProcessSimple(m_Laser))
        {
            CORE_INFO("Scan received: {0} ranges at {1} Hz", m_Laser.points.size(), 1.0f / m_Laser.config.scan_time);

            auto maxRange = std::max_element(m_Laser.points.begin(), m_Laser.points.end(), [](const LaserPoint &a, const LaserPoint &b)
                                             { return a.range < b.range; });

            CORE_INFO("{0} : {1}", glm::degrees(maxRange->angle) + 90, maxRange->range);

            Driver::MoveForward(100);
            Driver::TurnDegree(glm::degrees(maxRange->angle));

            return true;
        }
        else
            CORE_ERROR("Failed to get Lidar Data");

        return false;
    }
}