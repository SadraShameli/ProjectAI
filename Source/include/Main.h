#pragma once

#include "Lidar.h"

namespace ProjectAI
{
    class Application
    {
    public:
        Application();
        ~Application();
        static Application &Get();

        void Run();
        void Close();

    private:
        static inline Application *m_Instance = nullptr;

        bool m_Running;
    };
}
