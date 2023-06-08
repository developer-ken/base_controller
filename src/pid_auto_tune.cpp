#include "pid.h"
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iomanip>
#include <ctime>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "basectl.h"

using namespace std::chrono_literals;

#define SAMPLE_INTERVAL 0.02;

typedef struct
{
    double kp;
    double ki;
    double kd;
} PIDGains;

typedef struct
{
    double linearX, linearY, angular;
} Velocity;

class PIDAutotuneRos : public rclcpp::Node
{
public:
    std::chrono::steady_clock::time_point lasttime = std::chrono::steady_clock::now();
    geometry_msgs::msg::TransformStamped lasttransform;

    rclcpp::TimerBase::SharedPtr timer_{nullptr};
    std::string base_ctl_uart_device;
    BaseController *base_ctl;

    PIDAutotuneRos()
        : Node("base_pid_autotune")
    {
        base_ctl_uart_device = this->declare_parameter<std::string>("base_uart_device", "/dev/ttyUSB0");
        timer_ = this->create_wall_timer(
            1s, std::bind(&PIDAutotuneRos::on_timer, this));

        tf_buffer_ =
            std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ =
            std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    void on_timer()
    {
        
        PIDController pid_controller_x(200, 1.1, 2, 255);
        PIDController pid_controller_y(200, 1.1, 2, 255);
        PIDController pid_controller_r(5, 0.5, 2, 100);
        base_ctl = new BaseController(new Serial(base_ctl_uart_device.c_str(), 115200), &pid_controller_x, &pid_controller_y, &pid_controller_r);
        timer_->cancel();
        RCLCPP_INFO(this->get_logger(), "PID Auto tune start");
        RCLCPP_INFO(this->get_logger(), "Unlock motor in 3s...");
        Color yellow = {255, 255, 0};
        Color white = {255, 255, 255};
        base_ctl->LightControl(&yellow, &yellow);
        // base_ctl->Beep(true);

        base_ctl->MoveRaw({0, 0}, 0);
        rclcpp::sleep_for(3s);
        base_ctl->MotorLock(false);
        RCLCPP_INFO(this->get_logger(), "Motor unlocked!");
        rclcpp::sleep_for(1s);
        rclcpp::Rate loop_rate(50);
        base_ctl->Move({0, 0.05}, 0);
        for (int i = 0; i < 200; i++)
        {
            auto v = GetVelocity();
            RCLCPP_INFO(this->get_logger(), "speed: \t%f\t%f\t%f", v.linearX, v.linearY, v.angular);
            base_ctl->__PID({v.linearX, v.linearY}, v.angular);
            loop_rate.sleep();
        }
        // RCLCPP_INFO(this->get_logger(), "Tunning linear Y velocity...");
        // pid_x = autotune_pid(1, 0.1);
        // RCLCPP_INFO(this->get_logger(), "Result: %f %f %f", pid_x.kp, pid_x.ki, pid_x.kd);

        // RCLCPP_INFO(this->get_logger(), "Tunning angular velocity...");
        // pid_x = autotune_pid(2, 0.3);
        // RCLCPP_INFO(this->get_logger(), "Result: %f %f %f", pid_x.kp, pid_x.ki, pid_x.kd);
        base_ctl->Move({0, 0}, 0);
        base_ctl->MotorLock(true);
        base_ctl->LightControl(&white, &white);
    }

    Velocity GetVelocity()
    {
        geometry_msgs::msg::TransformStamped t;
        try
        {
            t = tf_buffer_->lookupTransform(
                "odom_frame", "base_link",
                tf2::TimePointZero);
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_INFO(
                this->get_logger(), "Could not transform %s to %s: %s",
                "odom_frame", "base_link", ex.what());
            return {0, 0, 0};
        }

        // 四元数转欧拉角
        tf2::Quaternion quat1(
            t.transform.rotation.x,
            t.transform.rotation.y,
            t.transform.rotation.z,
            t.transform.rotation.w);
        double roll, pitch, yaw; // 定义存储r\p\y的容器
        tf2::Matrix3x3 m(quat1);
        m.getRPY(roll, pitch, yaw); // 进行转换
                                    // 四元数转欧拉角
        tf2::Quaternion quat2(
            lasttransform.transform.rotation.x,
            lasttransform.transform.rotation.y,
            lasttransform.transform.rotation.z,
            lasttransform.transform.rotation.w);
        double lyaw; // 定义存储r\p\y的容器
        tf2::Matrix3x3 n(quat2);
        n.getRPY(roll, pitch, lyaw); // 进行转换

        auto now = std::chrono::steady_clock::now();
        double dt = std::chrono::duration_cast<std::chrono::milliseconds>(now - lasttime).count();
        lasttime = now;

        auto drot = yaw - lyaw;
        if (drot > M_PI)
            drot -= 2 * M_PI;
        else if (drot < -M_PI)
            drot += 2 * M_PI;
        // RCLCPP_INFO(this->get_logger(), "transrot %f -> %f = %f", lyaw, yaw, drot);

        Velocity v_odom = {
            (t.transform.translation.x - lasttransform.transform.translation.x) * 1000 / dt,
            (t.transform.translation.y - lasttransform.transform.translation.y) * 1000 / dt,
            (drot)*1000 / dt};

        lasttransform = t;

        Velocity v_base_link = {
            v_odom.linearX * cos(yaw) + v_odom.linearY * sin(yaw),
            -v_odom.linearX * sin(yaw) + v_odom.linearY * cos(yaw),
            v_odom.angular};

        return v_base_link;
    }

    double lx(double exec)
    {
        auto v = GetVelocity();
        base_ctl->MoveRaw({exec, 0}, 0);
        return v.linearX;
    }

    double ly(double exec)
    {
        auto v = GetVelocity();
        base_ctl->MoveRaw({0, exec}, 0);
        return v.linearY;
    }

    double ag(double exec)
    {
        auto v = GetVelocity();
        base_ctl->MoveRaw({0, 0}, exec);
        return -v.angular;
    }

    /// @brief PID自整定函数，用于自动计算PID参数。
    /// @param process_variable 函数指针，参数为控制器的输出值，返回值为当前测量值
    /// @param setpoint         调定点
    /// @return PID自整定结果
    PIDGains autotune_pid(int stt, double setpoint)
    {
        const double Ku = 0.6;
        const double Tu = 1.0;
        const double Kp = 0.6 * Ku;
        const double Ki = 1.2 * Ku / Tu;
        const double Kd = 0.075 * Ku * Tu;
        PIDController pid(Kp, Ki, Kd, 100);
        double output = 0.0;
        double currentval = 0.0;
        double dt = SAMPLE_INTERVAL;
        double t = 0.0;
        double max_output = 255.0;
        double min_output = -255.0;
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
                RCLCPP_INFO(this->get_logger(), "Stepping up...");
                RCLCPP_INFO(this->get_logger(), "pid-out:%f", output);

                if (stt == 0)
                    currentval = lx(output);
                else if (stt == 1)
                    currentval = ly(output);
                else if (stt == 2)
                    currentval = ag(output);

                RCLCPP_INFO(this->get_logger(), "measure:%f", currentval);

                error = setpoint - currentval;
                RCLCPP_INFO(this->get_logger(), "err:%f", error);
                if (error > max_error)
                {
                    //    state = 1;
                    //    prev_error = error;
                    //    count = 0;
                }
                break;
            case 1: // Step down
                output = min_output;

                RCLCPP_INFO(this->get_logger(), "Stepping down...");
                RCLCPP_INFO(this->get_logger(), "pid-out:%f", output);

                if (stt == 0)
                    currentval = lx(output);
                else if (stt == 1)
                    currentval = ly(output);
                else if (stt == 2)
                    currentval = ag(output);

                RCLCPP_INFO(this->get_logger(), "measure:%f", currentval);

                error = setpoint - currentval;
                RCLCPP_INFO(this->get_logger(), "err:%f", error);
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

            if (stt == 0)
                currentval = lx(output);
            else if (stt == 1)
                currentval = ly(output);
            else if (stt == 2)
                currentval = ag(output);

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
};

void mySigintHandler(int sig)
{
    // Do some custom action.
    // For example, publish a stop message to some other nodes.

    // All the default sigint handler does is call shutdown()
    rclcpp::shutdown();
}

int main(int argc, char **argv)
{
    signal(SIGINT, mySigintHandler);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PIDAutotuneRos>());
    rclcpp::shutdown();
    return 0;
}