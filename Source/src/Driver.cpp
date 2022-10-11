#include "Driver.h"
#include "GPIO.h"

namespace ProjectAI
{
    void Driver::Init()
    {
    }

    void Driver::MoveForward(float speed)
    {
        GPIO::PWM(GPIO::Motor1, speed);
        GPIO::PWM(GPIO::Motor2, 0);
    }

    void Driver::MoveBackward(float speed)
    {
        GPIO::PWM(GPIO::Motor1, 0);
        GPIO::PWM(GPIO::Motor2, speed);
    }

    void Driver::MoveLeft()
    {
        TurnDegree(-90);
    }

    void Driver::MoveRight()
    {
        TurnDegree(90);
    }

    void Driver::Stop()
    {
        GPIO::PWM(GPIO::Motor1, 0);
        GPIO::PWM(GPIO::Motor2, 0);
    }

    void Driver::TurnDegree(float angle)
    {
        float dutyCycle = (angle + 90) / 3 + 90;
        GPIO::PWM(GPIO::steeringPin, std::clamp(dutyCycle, 60.0f, 120.0f));
    }

    void Driver::TurnRadian(float angle)
    {
        TurnDegree(glm::degrees(angle));
    }
}