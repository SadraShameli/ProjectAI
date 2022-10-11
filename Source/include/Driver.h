#pragma once

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
    };
}