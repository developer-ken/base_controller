#include "basectl.h"
#include <time.h>

long long ltime()
{
    struct timespec uptime;
    clock_gettime(CLOCK_MONOTONIC, &uptime);
    return uptime.tv_sec * 1000LL + uptime.tv_nsec / 1000000LL;
}

BaseController::BaseController(Serial *port, PIDController *linearX, PIDController *linearY, PIDController *angular)
{
    this->port = port;
    this->linearX = linearX;
    this->linearY = linearY;
    this->angular = angular;
    linearTarget = {0, 0};
    angularTarget = 0;
    MotorLock(true);
}

void BaseController::Close()
{
    port->Close();
}

void BaseController::Move(Vector2 linear, double angular)
{
    linearTarget = linear;
    angularTarget = angular;
}

void BaseController::MoveRaw(Vector2 linear, double angular)
{
    int16_t Lf, Lb, Rf, Rb;
    Lf = linear.X - linear.Y - angular;
    Lb = linear.X + linear.Y - angular;
    Rf = linear.X + linear.Y + angular;
    Rb = linear.X - linear.Y + angular;

    transbuffer.data.HEADER = 0xAF;
    transbuffer.data.type = MOTOR_CMD;
    transbuffer.data.data[0] = Lf;
    transbuffer.data.data[1] = Rf;
    transbuffer.data.data[2] = Lb;
    transbuffer.data.data[3] = Rb;
    port->Write((char *)&transbuffer.bytes, 10);
}

void BaseController::MotorLock(bool lock)
{
    transbuffer.data.HEADER = 0xAF;
    transbuffer.data.type = MOTOR_LOCK_UNLOCK;
    transbuffer.data.data[0] = !lock;
    port->Write((char *)&transbuffer.bytes, 10);
    IsMotorLocked = lock;
}

void BaseController::Beep(bool on)
{
    transbuffer.data.HEADER = 0xAF;
    transbuffer.data.type = BEEP_CMD;
    transbuffer.data.data[0] = on;
    port->Write((char *)&transbuffer.bytes, 10);
}

void BaseController::LightControl(Color *Left, Color *Right)
{
    transbuffer.data.HEADER = 0xAF;
    transbuffer.data.type = LED_COLOR_CHANGE;
    if (Left != nullptr)
    {
        transbuffer.data.data[0] = 1; // 1 = Left
        transbuffer.data.data[1] = Left->R;
        transbuffer.data.data[2] = Left->G;
        transbuffer.data.data[3] = Left->B;
        port->Write((char *)&transbuffer.bytes, 10);
    }
    if (Right != nullptr)
    {
        transbuffer.data.data[0] = 0; // 0 = Right
        transbuffer.data.data[1] = Right->R;
        transbuffer.data.data[2] = Right->G;
        transbuffer.data.data[3] = Right->B;
        port->Write((char *)&transbuffer.bytes, 10);
    }
}

void BaseController::__PID(Vector2 Currentlinear, double Currentangular)
{
    long long currentTime = ltime();
    Vector2 linearOutput = {
        linearX->Update(linearTarget.X, Currentlinear.X, currentTime),
        linearY->Update(linearTarget.Y, Currentlinear.Y, currentTime)};
    double angularOutput = angular->Update(angularTarget, Currentangular, currentTime);

    // 如果目标速度为0，小车需要停下，将PID控制器重置为零状态，同时所有电机刹车
    // 这是为了加速小车停下，不等待PID控制器下降阶跃响应的拟合。
    /*if (linearTarget.X == 0 && linearTarget.Y == 0 && angularTarget == 0)
    {
        linearX->SetZeroState();
        linearY->SetZeroState();
        angular->SetZeroState();
        linearOutput.X = 0;
        linearOutput.Y = 0;
        angularOutput = 0;
    }*/
    MoveRaw(linearOutput, angularOutput);
}