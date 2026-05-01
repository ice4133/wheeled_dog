#ifndef ITA_CONTROLLER_BRIDGE_HPP_
#define ITA_CONTROLLER_BRIDGE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include "geometry_msgs/msg/twist.hpp" 
class ControllerBridgeNode : public rclcpp::Node 
{
public:
    ControllerBridgeNode();
    ~ControllerBridgeNode() = default;

private:
    // 回调函数声明：处理电机回传数据
    void feedback_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
    //处理上层发来的速度指令
    void command_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

    // ROS 2 通信接口声明
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_feedback_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr motor_cmd_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_subscriber_;

    // 用于降低终端打印频率的计数器
    int print_counter_;
};

#endif // WHEELED_DOG__BRAIN_NODE_HPP_