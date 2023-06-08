#include "pid.h"
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iomanip>
#include <ctime>
#include <signal.h>

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>

#include "basectl.h"

using namespace ros;
using namespace std;

void sigintHandler(int sig)
{
  printf("Caught SIGINT, exiting...\n");
  ros::shutdown();
}

typedef struct
{
  double linearX, linearY, angular;
} Velocity;

// Ros2 ver.
/*
class BaseControllerNode : public rclcpp::Node
{
public:
  BaseControllerNode() : Node("base_controller")
  {
    base_ctl_uart_device = this->declare_parameter<std::string>("base_uart_device", "/dev/ttyUSB0");
    pid_controller_x = new PIDController(200, 1.1, 5, 255); // X方向PID参数
    pid_controller_y = new PIDController(200, 1.1, 5, 255); // Y方向PID参数
    pid_controller_r = new PIDController(5, 0.5, 5, 100);   // 旋转PID参数
    base_ctl = new BaseController(new Serial(base_ctl_uart_device.c_str(), 115200), pid_controller_x, pid_controller_y, pid_controller_r);
    timer_ = this->create_wall_timer(rclcpp::Rate(50).period(), std::bind(&BaseControllerNode::on_timer, this));

    tf_buffer_ =
        std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ =
        std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10, std::bind(&BaseControllerNode::on_vel_cmd, this, std::placeholders::_1));

    Color white = {255, 255, 255};
    base_ctl->LightControl(&white, &white);
    base_ctl->MoveRaw({0, 0}, 0);
    base_ctl->MotorLock(false);
    RCLCPP_INFO(this->get_logger(), "Motor unlocked!");
  }

private:
  std::chrono::steady_clock::time_point lasttime = std::chrono::steady_clock::now();
  geometry_msgs::TransformStamped lasttransform;
  ros::Subscription<geometry_msgs::Twist>::SharedPtr subscription_;

  rclcpp::TimerBase::SharedPtr timer_{nullptr};
  std::string base_ctl_uart_device;

  PIDController *pid_controller_x, *pid_controller_y, *pid_controller_r;
  BaseController *base_ctl;
  void on_timer()
  {
    auto v = GetVelocity();
    base_ctl->__PID({v.linearX, v.linearY}, v.angular);
  }
  void on_vel_cmd(const geometry_msgs::msg::Twist::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "Move command: \t%f\t%f\t%f", msg->linear.x, msg->linear.y, msg->angular.z);
    base_ctl->Move({msg->linear.x, msg->linear.y}, msg->angular.z);
  }
};
*/

tf2_ros::Buffer *tfBuffer;
tf2_ros::TransformListener *tfListener;
std::chrono::steady_clock::time_point lasttime = std::chrono::steady_clock::now();
PIDController *pid_controller_x, *pid_controller_y, *pid_controller_r;
BaseController *base_ctl;
geometry_msgs::TransformStamped lasttransform;
std::string base_ctl_uart_device, odom_frame, base_link;
Velocity vlast;

void OnNewCmdVel(const geometry_msgs::TransformStamped::ConstPtr &msg)
{
  base_ctl->Move({msg->transform.translation.x, msg->transform.translation.y}, msg->transform.rotation.z);
}

Velocity GetVelocity()
{
  geometry_msgs::TransformStamped t;
  try
  {
    t = tfBuffer->lookupTransform(odom_frame, base_link, ros::Time(0));
  }
  catch (const tf2::TransformException &ex)
  {
    ROS_INFO("Transform error when getting current speed: %s", ex.what());
    throw;
  }

  // 四元数转欧拉角
  tf::Quaternion quat1(
      t.transform.rotation.x,
      t.transform.rotation.y,
      t.transform.rotation.z,
      t.transform.rotation.w);
  double roll, pitch, yaw; // 定义存储r\p\y的容器
  tf::Matrix3x3 m(quat1);
  m.getRPY(roll, pitch, yaw); // 进行转换
                              // 四元数转欧拉角
  tf::Quaternion quat2(
      lasttransform.transform.rotation.x,
      lasttransform.transform.rotation.y,
      lasttransform.transform.rotation.z,
      lasttransform.transform.rotation.w);
  double lyaw; // 定义存储r\p\y的容器
  tf::Matrix3x3 n(quat2);
  n.getRPY(roll, pitch, lyaw); // 进行转换

  auto now = std::chrono::steady_clock::now();
  double dt = std::chrono::duration_cast<std::chrono::milliseconds>(now - lasttime).count();
  lasttime = now;
  if (dt == 0)
    return vlast;

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
  vlast = v_base_link;
  return v_base_link;
}

void PID(const ros::TimerEvent &e)
{
  try
  {
    auto v = GetVelocity();
    ROS_INFO("Speed=%f,%f,%f", v.linearX, v.linearY, v.angular);
    if (base_ctl->IsMotorLocked)
    {
      ROS_WARN("Motor is locked, will do MOTOR_UNLOCK.");
      base_ctl->MotorLock(false);
    }
    base_ctl->__PID({v.linearX, v.linearY}, v.angular);
  }
  catch (const tf2::TransformException &ex)
  {
    ROS_WARN("Error getting velocity, closed-loop unreliable!");
    base_ctl->Move({0, 0}, 0); // 将目标移动指令置为0，这也会将PID控制器置零
    base_ctl->__PID({0, 0}, 0);
  }
}

int main(int argc, char **argv)
{
  signal(SIGINT, sigintHandler);
  ros::init(argc, argv, "base_controller");
  ros::NodeHandle n;
  tfBuffer = new tf2_ros::Buffer();
  tfListener = new tf2_ros::TransformListener(*tfBuffer);
  base_ctl_uart_device = n.param<std::string>("base_uart_device", "/dev/ttyUSB0");
  odom_frame = n.param<std::string>("odom_frame", "t265_odom_frame");
  base_link = n.param<std::string>("base_link", "base_link");
  pid_controller_x = new PIDController(200, 1.1, 5, 255); // X方向PID参数
  pid_controller_y = new PIDController(200, 1.1, 5, 255); // Y方向PID参数
  pid_controller_r = new PIDController(5, 0.5, 5, 100);   // 旋转PID参数
  base_ctl = new BaseController(new Serial(base_ctl_uart_device.c_str(), 115200), pid_controller_x, pid_controller_y, pid_controller_r);

  Subscriber cv = n.subscribe("/cmd_vel", 5, OnNewCmdVel);
  Timer tm = n.createTimer(ros::Rate(50).expectedCycleTime(), PID, false, true);
  // tm.start();
  Color white = {255, 255, 255};
  Color red = {255, 0, 0};
  base_ctl->LightControl(&white, &white);
  ROS_INFO("Safety: Beep for 3 sec before motor unlock...");
  base_ctl->Beep(true);
  base_ctl->LightControl(&red, &red);
  ros::Duration(0.5).sleep();
  base_ctl->Beep(false);
  base_ctl->LightControl(&white, &white);
  ros::Duration(0.5).sleep();
  base_ctl->Beep(true);
  base_ctl->LightControl(&red, &red);
  ros::Duration(0.5).sleep();
  base_ctl->Beep(false);
  base_ctl->LightControl(&white, &white);
  ros::Duration(0.5).sleep();
  base_ctl->Beep(true);
  base_ctl->LightControl(&red, &red);
  ros::Duration(0.5).sleep();
  base_ctl->Beep(false);
  base_ctl->LightControl(&white, &white);
  ros::Duration(0.5).sleep();
  base_ctl->MoveRaw({0, 0}, 0);
  base_ctl->MotorLock(false);
  ROS_INFO("Motor unlocked!");

  ros::spin();
  return 0;
}