#pragma once

#include <CYdLidar.h>

namespace ProjectAI
{
    class Lidar
    {
    public:
        static void Init();
        static void Destroy();

        static bool Connect();
        static bool Scan();

    private:
        static inline CYdLidar m_Lidar;
    };
}