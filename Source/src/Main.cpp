#include "Main.h"
#include "Timer.h"
#include "Log.h"

namespace ProjectAI
{
    static void signal_callback_handler(int signum)
    {
        CORE_ERROR("Terminating");
        Application::Get().Close();
    }

    Application::Application()
    {
        m_Instance = this;
        m_Running = true;

        signal(SIGINT, signal_callback_handler);
        Log::Init();

        CORE_INFO("Welcome to ProjectA.I.");
    }

    Application::~Application()
    {
        Close();
    }

    Application &Application::Get()
    {
        return *m_Instance;
    }

    void Application::Run()
    {
        if (!m_Lidar.Connect())
        {
            CORE_ERROR("Couldn't find a Lidar device, Waiting for device!");
            while (m_Running && !m_Lidar.Connect())
                sleep(1);
        }

        while (m_Running)
        {
            Timer timer("Runtime");
            m_Lidar.Scan();
        }
    }

    void Application::Close()
    {
        m_Running = false;
    }
}

int main()
{
    auto app = std::make_unique<ProjectAI::Application>();
    app->Run();

    return 0;
}