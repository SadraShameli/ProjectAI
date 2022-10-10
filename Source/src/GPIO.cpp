#include "GPIO.h"
#include <pigpio.h>

namespace ProjectAI
{
    struct Pin
    {
        int tag;
        clock_t updateTime;
        int interval;
        bool continuous;
        bool state = false;
        Pin(GPIO::Outputs _tag) : tag((int)_tag) {}
    };

    static Pin s_Pins[]{GPIO::Outputs::Motor1, GPIO::Outputs::Motor2, GPIO::Outputs::SteeringPWM};

    void GPIO::Init()
    {
        gpioInitialise();
        for (auto &pin : s_Pins)
            gpioSetMode(pin.tag, PI_OUTPUT);
    }

    void GPIO::Destroy()
    {
        gpioTerminate();
    }

    void GPIO::Toggle(Outputs pinTag, bool targetState)
    {
        for (auto &pin : s_Pins)
        {
            if ((Outputs)pin.tag == pinTag)
            {
                gpioWrite(pin.tag, targetState);
                pin.interval = 999999;
                return;
            }
        }
    }

    void GPIO::Blink(Outputs pinTag, int blinkTime, bool continuousBlinking)
    {
        for (auto &pin : s_Pins)
        {
            if ((Outputs)pin.tag == pinTag)
            {
                pin.state = true;
                pin.interval = blinkTime;
                pin.continuous = continuousBlinking;
                pin.updateTime = 0;
                Update();

                return;
            }
        }
    }

    void GPIO::SetContinuity(Outputs pinTag, bool continuousBlinking)
    {
        for (auto &pin : s_Pins)
        {
            if ((Outputs)pin.tag == pinTag)
            {
                pin.continuous = continuousBlinking;
                return;
            }
        }
    }

    void GPIO::Update()
    {
        clock_t currentTime = clock();
        for (auto &pin : s_Pins)
        {
            if ((currentTime - pin.updateTime) > pin.interval)
            {
                pin.updateTime = currentTime;
                if (pin.continuous)
                {
                    pin.state = !pin.state;
                    gpioWrite(pin.tag, pin.state);
                }
                else
                {
                    if (pin.state)
                    {
                        gpioWrite(pin.tag, 1);
                        pin.state = false;
                    }
                    else
                        gpioWrite(pin.tag, 0);
                }
            }
        }
    }
}