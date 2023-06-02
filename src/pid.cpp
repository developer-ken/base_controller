#include "pid.h"

PIDController::PIDController(double kp, double ki, double kd, double maxintegral)
    : kp(kp), ki(ki), kd(kd), prev_error(0), integral(0), maxintegral(maxintegral)
{
}

double PIDController::Update(double setpoint, double currentval, long time)
{
    if (time_last == 0)
    {
        time_last = time;
        return 0;
    }
    long dt = time - time_last;
    if (dt == 0) // dt为0，时间没有变化，系统应当保持输出值不变
    {
        return outlast;
    }
    double kp_new = kp * (dt / TIME_IMTERVAL);
    double ki_new = ki * (TIME_IMTERVAL / dt);
    double kd_new = kd / (dt / TIME_IMTERVAL);
    double error = setpoint - currentval;
    integral += error * dt;

    if (integral > maxintegral)
        integral = maxintegral;
    else if (integral < -maxintegral)
        integral = -maxintegral;

    double derivative = (error - prev_error) / dt;
    double output = kp_new * error + ki_new * integral + kd_new * derivative;
    prev_error = error;
    time_last = time;
    outlast = output;
    return output;
}