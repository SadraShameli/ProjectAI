#pragma once
#include <numbers>

namespace ProjectAI
{
    class Driver
    {
    public:        
        static void Init();

        static void MoveForward(float);
        static void MoveBackward(float);
        static void MoveLeft();
        static void MoveRight();
        static void Stop();

        static void TurnDegree(float);
        static void TurnRadian(float);
    
    public:    
        static constexpr float Left = 0.0f;
        static constexpr float TopLeft = std::numbers::pi * 0.25f;
        static constexpr float Ahead = std::numbers::pi * 0.5f;
        static constexpr float TopRight = std::numbers::pi * 0.75f;
        static constexpr float Right = std::numbers::pi * 1.0;
    };
}