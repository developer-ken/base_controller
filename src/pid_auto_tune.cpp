#include <math.h>
#include <time.h>
#include <chrono> // for std::chrono::duration, std::chrono::duration_cast
#include <thread> // for std::this_thread::sleep_for
#include "pid.h"

typedef struct
{
    double kp;
    double ki;
    double kd;
} PIDGains;

long long get_time()
{
    struct timespec uptime;
    clock_gettime(CLOCK_MONOTONIC, &uptime);
    return uptime.tv_sec * 1000LL + uptime.tv_nsec / 1000000LL;
}

/// @brief PID自整定函数，用于自动计算PID参数。
/// @param process_variable 函数指针，参数为控制器的输出值，返回值为当前测量值
/// @param setpoint         调定点
/// @return PID自整定结果
PIDGains autotune_pid(double (*process_variable)(double), double setpoint)
{
    const double Ku = 0.6;
    const double Tu = 1.0;
    const double Kp = 0.6 * Ku;
    const double Ki = 1.2 * Ku / Tu;
    const double Kd = 0.075 * Ku * Tu;
    PIDController pid(Kp, Ki, Kd, 100);
    double output = 0.0;
    double currentval = 0.0;
    double dt = 0.02;
    double t = 0.0;
    double max_output = 100.0;
    double min_output = -100.0;
    double max_error = setpoint * 0.1;
    double error = 0.0;
    double prev_error = 0.0;
    int state = 0;
    int count = 0;
    while (true)
    {
        switch (state)
        {
        case 0: // Step up
            output = max_output;
            currentval = process_variable(output);
            error = setpoint - currentval;
            if (error > max_error)
            {
                state = 1;
                prev_error = error;
                count = 0;
            }
            break;
        case 1: // Step down
            output = min_output;
            currentval = process_variable(output);
            error = setpoint - currentval;
            if (error < -max_error)
            {
                state = 2;
            }
            break;
        case 2: // Calculate gains
            double ku = 4.0 * Kp / (3.14159 * sqrt(fabs(prev_error)));
            double tu = 3.14159 * fabs(currentval - setpoint) / (2.0 * fabs(output));
            double kp = 0.6 * ku;
            double ki = 1.2 * ku / tu;
            double kd = 0.075 * ku * tu;
            return {kp, ki, kd};
        }
        output += pid.Update(setpoint, currentval, dt);
        if (output > max_output)
        {
            output = max_output;
        }
        else if (output < min_output)
        {
            output = min_output;
        }
        currentval = process_variable(output);
        error = setpoint - currentval;
        if (fabs(error) > max_error)
        {
            state = 1;
            prev_error = error;
            count = 0;
        }
        count++;
        if (count > 10000)
        {
            break;
        }
        std::this_thread::sleep_for(std::chrono::duration<double>(dt));
        t += dt;
    }
    return {0.0, 0.0, 0.0};
}

int main(int argc, char ** argv)
{
  if(argc!=2){
    
  }
  return 0;
}
