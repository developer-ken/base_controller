#ifndef _H_BASECTL_
#define _H_BASECTL_

#include "serial.h"
#include "pid.h"
#include <sys/types.h>

struct Vector2
{
    double X, Y;
};

struct TransData
{
    u_int8_t HEADER;
    u_int8_t type;
    int16_t data[4];
};

struct Color
{
    u_int8_t R, G, B;
};

union TransDataBuf
{
    TransData data;
    u_int8_t bytes[10];
};

enum PackType
{
    MOTOR_CMD = 0xF1,
    LED_COLOR_CHANGE = 0xF2,
    BEEP_CMD = 0xF3,
    MOTOR_LOCK_UNLOCK = 0xF4,
};

class BaseController
{
public:
    bool IsMotorLocked = true;

    /// @brief 初始化底盘控制器。底盘本身是开环的，PID依赖外部里程计输入。
    /// @param port     用于连接MCU的串口
    /// @param linearX  X速度环PID控制器
    /// @param linearY  Y速度环PID控制器
    /// @param angular  角速度环PID控制器
    BaseController(Serial *port, PIDController *linearX, PIDController *linearY, PIDController *angular);
    
    /// @brief 关闭串口
    void Close();

    /// @brief 设置目标速度，单位为m/s和rad/s。__PID函数会计算PID输出并发送至底盘。
    /// @param linear  目标线速度(X/Y)
    /// @param angular 目标角速度
    void Move(Vector2 linear, double angular);

    /// @brief 绕过PID直接发送速度指令至底盘。速度值无物理单位，为直接指令值。
    /// @param linear  目标线速度指令
    /// @param angular 目标角速度指令
    void MoveRaw(Vector2 linear, double angular);

    /// @brief 锁定或解除电机安全锁。被锁定的电机无法被指令旋转，但也不会处于刹车状态。
    /// @param lock true=锁,false=解锁
    void MotorLock(bool lock);
    void Beep(bool on);

    /// @brief 改变左右车灯颜色，nullptr为不改变。
    /// @param Left  左车灯颜色
    /// @param Right 右车灯颜色
    void LightControl(Color *Left, Color *Right);

    /// @brief PID控制器周期计算函数，在获得里程计数据更新时调用。如果里程计低于50Hz每秒，则必须对数据进行插值，然后以50Hz调用本函数。
    /// @param Currentlinear   里程计获取的线速度
    /// @param Currentangular  里程计获取的角速度
    void __PID(Vector2 Currentlinear, double Currentangular);

private:
    Serial *port;
    Vector2 linearTarget;
    PIDController *linearX, *linearY, *angular;
    double angularTarget;
    double prev_err, integral;
    TransDataBuf transbuffer;
};

#endif