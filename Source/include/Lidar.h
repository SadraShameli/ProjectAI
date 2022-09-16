#pragma once

#include <CYdLidar.h>

namespace ProjectAI
{
    class Lidar
    {
    public:
        Lidar();
        ~Lidar();

        bool Connect();
        bool Scan();

    private:
        CYdLidar m_Lidar;
    };
}