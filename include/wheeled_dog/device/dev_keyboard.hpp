#ifndef __DEV_KEYBOARD_HPP
#define __DEV_KEYBOARD_HPP


#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <map>
#include <utility>

// 逻辑归类：键盘控制节点类
class KeyboardController : public rclcpp::Node 
{
public:
    // 构造函数与析构函数
    KeyboardController();
    ~KeyboardController();

    // 核心运行逻辑的接口声明
    void run();

private:
    // 数据发布器成员变量
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    
    // 指令映射字典：静态常量声明
    static const std::map<char, std::pair<double, double>> MOVE_BINDINGS;
};


#endif
