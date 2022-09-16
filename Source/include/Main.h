#pragma once

#include "Lidar.h"

namespace ProjectAI
{
    class Application
    {
    public:
        Application();
        ~Application();

        Application &Get();

        void Run();

    private:
        static inline Application *m_Instance = nullptr;
        Lidar m_Lidar;

        bool m_Running;
    };
}
