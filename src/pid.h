#ifndef _PID_CONTROLLER_
#define _PID_CONTROLLER_

#define SAMPLE_RATE 50.0F

#define TIME_IMTERVAL (1000.0F / SAMPLE_RATE)

class PIDController
{
public:
    /// @brief 初始化PID控制器。参数应于SAMPLE_RATE(默认50Hz)整定。
    /// @param kp
    /// @param ki
    /// @param kd
    /// @param maxintegral 积分项钳制
    PIDController(double kp, double ki, double kd, double maxintegral);

    /// @brief 刷新PID控制器，向后推导一步。
    /// @param setpoint   目标值
    /// @param currentval 当前测量值
    /// @param time       当前毫秒时间戳，用于补偿非实时系统带来的周期误差
    /// @return 控制器输出值
    double Update(double setpoint, double currentval, long time);

private:
    double kp, ki, kd;
    double prev_error, integral, maxintegral, outlast;
    long time_last;
};

#endif