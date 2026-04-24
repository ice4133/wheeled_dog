#include "wheeled_dog/device/dev_keyboard.hpp"
#include <termios.h>
#include <unistd.h>
#include <iostream>

// 1. 初始化常量字典（实现具体映射规则）
const std::map<char, std::pair<double, double>> KeyboardController::MOVE_BINDINGS = 
{
    {'w', { 1.0,  0.0}},
    {'s', {-1.0,  0.0}},
    {'a', { 0.0,  1.0}},
    {'d', { 0.0, -1.0}},
    {' ', { 0.0,  0.0}}
};

// 2. 构造函数实现（实例化发布器）
KeyboardController::KeyboardController() : Node("keyboard_controller_node")
{
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    RCLCPP_INFO(this->get_logger(), "机器狗键盘控制已启动！\n使用 W/A/S/D 控制，空格停止，Ctrl+C 退出。");
}

// 3. 析构函数实现（确保安全退出）
KeyboardController::~KeyboardController() 
{
    RCLCPP_INFO(this->get_logger(), "键盘控制节点正在关闭...");
}

// 4. 核心逻辑实现（捕获按键并发布）
void KeyboardController::run() 
{
    struct termios original_tty, raw_tty;
    
    // 获取当前终端底层设置
    tcgetattr(STDIN_FILENO, &original_tty);
    raw_tty = original_tty;
    raw_tty.c_lflag &= ~(ICANON | ECHO); // 关闭缓冲与回显
    raw_tty.c_cc[VMIN] = 0;
    raw_tty.c_cc[VTIME] = 1;
    tcsetattr(STDIN_FILENO, TCSANOW, &raw_tty);

    char key;
    auto twist_msg = geometry_msgs::msg::Twist();

    while (rclcpp::ok()) 
    {
        if (read(STDIN_FILENO, &key, 1) > 0) 
        {
            if (key == '\x03') 
            { // Ctrl+C
                break;
            }
            auto it = MOVE_BINDINGS.find(key);
            if (it != MOVE_BINDINGS.end()) 
            {
                twist_msg.linear.x = it->second.first;
                twist_msg.angular.z = it->second.second;
                publisher_->publish(twist_msg);
            }
        }
        usleep(10000); // 10ms 休眠，防止死循环跑满 CPU 核心
    }

    // 退出前安全机制：发送 0 速度，并恢复终端环境
    twist_msg.linear.x = 0.0;
    twist_msg.angular.z = 0.0;
    publisher_->publish(twist_msg);
    tcsetattr(STDIN_FILENO, TCSANOW, &original_tty);
}


int main(int argc, char **argv) 
{
    // 1. 初始化 ROS 2 运行环境
    rclcpp::init(argc, argv);
    
    // 2. 实例化键盘控制节点
    auto node = std::make_shared<KeyboardController>();
    
    // 3. 阻塞执行核心循环
    node->run();
    
    // 4. 退出清理
    rclcpp::shutdown();
    return 0;
}