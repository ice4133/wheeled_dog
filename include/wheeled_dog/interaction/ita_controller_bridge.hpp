#ifndef ITA_CONTROLLER_BRIDGE_HPP_
#define ITA_CONTROLLER_BRIDGE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

class ControllerBridgeNode : public rclcpp::Node 
{
public:
    ControllerBridgeNode();
    ~ControllerBridgeNode() = default;

private:
    // 回调函数声明：处理电机回传数据
    void feedback_callback(const sensor_msgs::msg::JointState::SharedPtr msg);

    // ROS 2 通信接口声明
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_feedback_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr motor_cmd_pub_;

    // 用于降低终端打印频率的计数器
    int print_counter_;
};

#endif // WHEELED_DOG__BRAIN_NODE_HPP_