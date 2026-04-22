#ifndef __DRV_UNITTREE_MOTOR_HPP
#define __DRV_UNITTREE_MOTOR_HPP


#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <vector>


// 引入第三方库的头文件
#include "serialPort/SerialPort.h" 

#define MOTOR_COUNT 12

//#define TEST
#define DEMO

typedef struct 
{
    double target_position;  // 目标位置 ±411774 (rad)
    double target_velocity;  // 目标速度 ±804.00 （rad/s）

    double actual_position;  // 实际位置 ±411774 (rad)
    double actual_velocity;  // 实际速度 ±804.00 （rad/s）
    double actual_effort;    // 实际力矩 ±127.99 （N.m）

    MotorData   data;        // 反馈数据结构体
}unittree_motor_data_t;





class MotorControllerNode : public rclcpp::Node 
{
public:
    MotorControllerNode();
    ~MotorControllerNode();

private:
    // ROS2 通信接口
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_sub_;

    //  核心回调与大循环
    void Cmd_Topic_Callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void TIM_PeriodElapsedCallback();


    void exchange_motor_data();
    void Motor_Init();
    #ifdef TEST
    void exchange_motor_data_test();
    #endif


    // 数据缓存 (Data Buffers)
    // 保存12个电机的目标状态（来自上层）
    std::vector<unittree_motor_data_t> unittree_motor_data_vector_;

    //第三方库串口对象
    SerialPort serial_; // 保持串口连接的生命周期


    int test = 0;
    // 电机数据
    float K_P = 0.02;// 关节刚度系数   0~25.599
    float K_W = 0.01;// 关节速度系数   0~25.599
    //限位数据
};



#endif