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
        TurnDegree(Left);
    }

    void Driver::MoveRight()
    {
        TurnDegree(Right);
    }

    void Driver::Stop()
    {
        GPIO::PWM(GPIO::Motor1, 0);
        GPIO::PWM(GPIO::Motor2, 0);
    }

    void Driver::TurnDegree(float angle)
    {
        GPIO::PWM(GPIO::steeringPin, std::clamp(angle < 0 ? angle + 360 : angle, 0.0f, 360.0f));
    }

    void Driver::TurnRadian(float angle)
    {
        TurnDegree(glm::degrees(angle));
    }
}