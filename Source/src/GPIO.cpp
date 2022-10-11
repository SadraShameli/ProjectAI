#include "GPIO.h"
#include <pigpio.h>

namespace ProjectAI
{
    struct Pin
    {
        unsigned int Port;
        bool State, IsContinuous;
        clock_t PreviousTime, Interval;

        Pin(GPIO::Outputs port) : Port(port), State(false), IsContinuous(false), PreviousTime(0), Interval(0) {}
    };

    struct PWM
    {
        unsigned int Port, Range, DutyCycle;
        PWM(GPIO::Outputs port, unsigned int range, unsigned int duty) : Port(port), Range(range), DutyCycle(duty) {}
    };

    static Pin s_Pins[] = {};
    static PWM s_PinsPWM[] = {PWM(GPIO::Outputs::Motor1, 255, 20000), PWM(GPIO::Outputs::Motor2, 255, 20000), PWM(GPIO::Outputs::steeringPin, 1100, 50)};

    void GPIO::Init()
    {
        gpioInitialise();
        for (auto &pin : s_Pins)
            gpioSetMode(pin.Port, PI_OUTPUT);

        for (auto &pin : s_PinsPWM)
        {
            gpioSetPWMrange(pin.Port, pin.Range);
            gpioSetPWMfrequency(pin.Port, pin.DutyCycle);
        }
    }

    void GPIO::Destroy()
    {
        for (auto &pin : s_Pins)
            gpioWrite(pin.Port, false);

        for (auto &pin : s_PinsPWM)
            gpioPWM(pin.Port, 0);

        gpioTerminate();
    }

    void GPIO::Toggle(Outputs port, bool targetState)
    {
        for (auto &pin : s_Pins)
        {
            if (pin.Port == port)
            {
                pin.State = targetState;
                pin.Interval = 0;
                Update();

                return;
            }
        }
    }

    void GPIO::PWM(Outputs port, float dutyCycle)
    {
        for (auto &pin : s_PinsPWM)
        {
            if (pin.Port == port)
            {
                gpioPWM(port, std::min(dutyCycle, (float)pin.Range));

                return;
            }
        }
    }

    void GPIO::Blink(Outputs port, int blinkTime, bool continuousBlinking)
    {
        for (auto &pin : s_Pins)
        {
            if (pin.Port == port)
            {
                pin.State = true;
                pin.Interval = blinkTime;
                pin.IsContinuous = continuousBlinking;
                pin.PreviousTime = 0;
                Update();

                return;
            }
        }
    }

    void GPIO::SetContinuity(Outputs port, bool continuousBlinking)
    {
        for (auto &pin : s_Pins)
        {
            if (pin.Port == port)
            {
                pin.IsContinuous = continuousBlinking;
                return;
            }
        }
    }

    void GPIO::Update()
    {
        clock_t currentTime = clock();
        for (auto &pin : s_Pins)
        {
            if (pin.Interval)
            {
                if (currentTime - pin.PreviousTime > pin.Interval)
                {
                    pin.PreviousTime = currentTime;
                    gpioWrite(pin.Port, pin.State);

                    if (pin.IsContinuous)
                        pin.State = !pin.State;
                }
            }
            else
                gpioWrite(pin.Port, pin.State);
        }
    }
}