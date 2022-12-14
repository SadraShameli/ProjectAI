#pragma once
#include <chrono>

namespace ProjectAI
{
    class GPIO
    {
    public:
        enum Outputs
        {
            Motor1 = 24,
            Motor2 = 23,
            steeringPin = 18
        };

    public:
        static void Init();
        static void Destroy();

        static void Update();
        static void Toggle(Outputs, bool);
        static void PWM(Outputs, float);
        static void Blink(Outputs, int, bool = false);
        static void SetContinuity(Outputs, bool);
    };
}