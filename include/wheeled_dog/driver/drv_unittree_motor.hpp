#ifndef __DRV_UNITTREE_MOTOR_HPP
#define __DRV_UNITTREE_MOTOR_HPP

//引入系统头文件
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <vector>
#include "geometry_msgs/msg/twist.hpp" 
// 引入第三方库的头文件
#include "serialPort/SerialPort.h" 


//引入自己编写各种头文件
#include "wheeled_dog/algorithm/alg_slope.hpp"
#include "wheeled_dog/algorithm/alg_fsm.hpp"


#define MOTOR_COUNT 12
#define MOTOR_REDUCTION 6.33


#define JUST_SEND
//#define FEEDBACK

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
    void Cmd_Topic_Callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void TIM_PeriodElapsedCallback();


    // 电机初始化
    void Motor_Init();

    // 最终给电机发数据的函数，腿部和轮子
    void Inverse_Kinematics_Calculation();    
    void Update_Leg_Data();
    void Update_Wheel_Data();
    void Rs485_Send_Data();
    void Just_Send(int i);

    // 为状态机预留的接口函数
    void Update_Fsm_State(int cmd_msg);
    void Judge_Alive();
  

    double Torque[2]={0.4,1.0};
    // 变量声明
    // ROS2 通信接口
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;

    // 数据缓存 (Data Buffers)
    // 保存12个电机的目标状态（来自上层）
    std::vector<unittree_motor_data_t> unittree_motor_data_vector_;
    // 专门用于给 sendRecv 喂数据的连续内存容器，提前分配好 12 个空间
    std::vector<MotorCmd>  send_cmds_vec_{MOTOR_COUNT};
    std::vector<MotorData> recv_datas_vec_{MOTOR_COUNT};
    //第三方库串口对象
    SerialPort serial_; // 保持串口连接的生命周期


    //算法类声明，用以组合底层和算法
    Class_FSM class_fsm_controller; // FSM状态机对象



    int test = 0;
    bool feedback_flag = false; 
    long long Alive_Flag = 0; //记录上次成功通信的时间戳，用于监测断联和重连逻辑
    long long Pre_Alive_Flag = 0;
    // 电机数据
    float K_P = 0.4;// 关节刚度系数   0~25.599
    float K_W = 0.25;// 轮足动态速度系数   0~25.599

    double track_width = 0.6;      // 左右轮距 (单位：米)
    double wheel_radius = 0.1;     // 车轮半径 (单位：米)
    double max_wheel_speed = 0.2;  // 车轮最大允许线速度 (单位：m/s)，用于限幅保护
    double max_angular_speed = 3.14; // 机器人最大允许角速度 (单位：rad/s)，用于限幅保护

    double x_velocity_command = 0.0; // 来自上层的线速度指令 (单位：m/s)
    double z_angular_command = 0.0; // 来自上层的角速度指令 (单位：rad/s)
};



#endif
