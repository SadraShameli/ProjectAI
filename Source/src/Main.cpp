#include "Main.h"

namespace ProjectAI
{
    Application::Application()
    {
        m_Instance = this;
        m_Running = true;
    }

    Application::~Application()
    {
    }

    Application &Application::Get()
    {
        return *m_Instance;
    }

    void Application::Run()
    {
        m_Lidar.Connect();

        while (m_Running)
        {
            m_Lidar.Scan();
        }
    }

}

int main()
{
    std::unique_ptr<ProjectAI::Application> app = std::make_unique<ProjectAI::Application>();
    app->Run();

    return 0;
}