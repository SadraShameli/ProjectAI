#pragma once

namespace ProjectAI
{
    class Driver
    {
    public:
        static void Init();

        static void MoveForward();
        static void MoveBackward();
        static void MoveLeft();
        static void MoveRight();

        static void Turn(float);
        static void Stop();

    private:
    };
}