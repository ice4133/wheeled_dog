#ifndef __DRV_UNITTREE_MOTOR_H
#define __DRV_UNITTREE_MOTOR_H


#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <vector>


// 引入第三方库的头文件
#include "serialPort/SerialPort.h" 

class MotorControllerNode : public rclcpp::Node {
public:
    MotorControllerNode();
    ~MotorControllerNode();

private:
    // 1. ROS2 通信接口
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_sub_;

    // 2. 核心回调与大循环
    void cmd_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void TIM_PeriodElapsedCallback();

    // 3. 电机交互数据
    void exchange_motor_data();

    // 4. 数据缓存 (Data Buffers)
    // 保存12个电机的目标状态（来自上层）
    std::vector<double> target_positions_;
    // 保存12个电机的实际状态（来自底层硬件）
    std::vector<double> actual_positions_;
    std::vector<double> actual_velocities_;
    std::vector<double> actual_efforts_; // 电流或力矩

    //5.第三方库串口对象
    SerialPort serial_; // 保持串口连接的生命周期
};



#endif