#include "Main.h"
#include "Timer.h"
#include "Log.h"
#include "GPIO.h"

namespace ProjectAI
{
    static void signal_callback_handler(int signum)
    {
        switch (signum)
        {
        case SIGINT:
            CORE_WARN("Terminating");
            Application::Get().Close();
            break;

        default:
            break;
        }
    }

    Application::Application()
    {
        m_Instance = this;

        // Initializing
        ScopedTimer timer("Initialization");

        // Logging system
        Log::Init();
        CORE_INFO("Welcome to ProjectA.I.");

        // Terminal Signal event listener
        signal(SIGINT, signal_callback_handler);

        // Initializing GPIO
        GPIO::Init();

        // Lidar SDK
        Lidar::Init();
        if (!Lidar::Connect())
        {
            CORE_ERROR("Couldn't find a Lidar device, Waiting for device!");
            while (m_Running && !Lidar::Connect())
                sleep(1);
        }

        // Initialization done! Running application
        m_Running = true;
    }

    Application::~Application()
    {
        ScopedTimer timer("Releasing resources");

        if (m_Running)
            Close();

        Lidar::Destroy();
        GPIO::Destroy();
    }

    void Application::Close()
    {
        CORE_INFO("Closing ProjectA.I.");
        m_Running = false;
    }

    Application &Application::Get()
    {
        return *m_Instance;
    }

    void Application::Run()
    {
        GPIO::Toggle(GPIO::Motor1, true);
        GPIO::Toggle(GPIO::Motor2, false);

        while (m_Running)
        {
            ScopedTimer timer("Runtime");

            // Runtime
            GPIO::Update();
            Lidar::Scan();
        }
    }
}

int main()
{
    auto app = std::make_unique<ProjectAI::Application>();
    app->Run();

    return 0;
}