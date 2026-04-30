#ifndef __DRV_UNITTREE_MOTOR_HPP
#define __DRV_UNITTREE_MOTOR_HPP

//引入系统头文件
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <vector>

// 引入第三方库的头文件
#include "serialPort/SerialPort.h" 


//引入自己编写各种头文件
#include "wheeled_dog/algorithm/alg_slope.hpp"


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

    MotorCmd cmd; 
    MotorData data; 

    float K_P; // 关节刚度系数 0~25.599
    float K_W; // 关节速度系数 0~25.599

    algorithm::Class_Slope_Filter slope_filter; // 斜坡滤波器对象

}unittree_motor_data_t;

class MotorControllerNode : public rclcpp::Node 
{
public:
    MotorControllerNode();
    ~MotorControllerNode();

private:
    //  函数声明
    //  核心回调与大循环
    void Cmd_Topic_Callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void TIM_PeriodElapsedCallback();


    void exchange_motor_data();
    void Motor_Init();
    #ifdef TEST
    void exchange_motor_data_test();
    #endif


  
    // 变量声明
    // ROS2 通信接口
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_sub_;

    // 数据缓存 (Data Buffers)
    // 保存12个电机的目标状态（来自上层）
    std::vector<unittree_motor_data_t> unittree_motor_data_vector_;
    // 专门用于给 sendRecv 喂数据的连续内存容器，提前分配好 12 个空间
    std::vector<MotorCmd>  send_cmds_vec_{MOTOR_COUNT};
    std::vector<MotorData> recv_datas_vec_{MOTOR_COUNT};
    //第三方库串口对象
    SerialPort serial_; // 保持串口连接的生命周期


    //算法类声明，用以组合底层和算法




    int test = 0;
    bool feedback_flag = false; 
    bool static_or_dynamic_flag = false; // false:动态，true:静态
    // 电机数据
    float K_P = 0.4;// 关节刚度系数   0~25.599
    float K_W = 0.25;// 轮足动态速度系数   0~25.599
    float K_W_Static = 0.01;// 轮足静态速度系数   0~25.599
    //限位数据
};



#endif
