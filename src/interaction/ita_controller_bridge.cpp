#include "wheeled_dog/interaction/ita_controller_bridge.hpp"


// 构造函数实现
ControllerBridgeNode::ControllerBridgeNode() : Node("controller_bridge_node"), print_counter_(0) 
{
    // 1. 初始化订阅者：接收底层电机节点的反馈数据
    joint_feedback_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", 10, std::bind(&ControllerBridgeNode::feedback_callback, this, std::placeholders::_1));

    // 2. 初始化发布者：预留，用于之后给底层电机发送控制指令
    motor_cmd_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("joint_cmds", 10);

    velocity_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 
            10, 
            std::bind(&ControllerBridgeNode::command_callback, this, std::placeholders::_1)
        );


    RCLCPP_INFO(this->get_logger(), "控制器桥接节点已启动，正在等待电机反馈和键盘数据...");
}

// 回调函数实现
void ControllerBridgeNode::feedback_callback(const sensor_msgs::msg::JointState::SharedPtr msg) 
{
    // 降低打印频率（每10次回调打印一次）
    if (++print_counter_ % 10 != 0) {
        return;
    }

    // 提取关节数据
    RCLCPP_INFO(this->get_logger(), "收到关节反馈数据");
    
    // 遍历所有关节
    for (size_t i = 0; i < msg->name.size(); ++i) 
    {
        RCLCPP_INFO(this->get_logger(), "关节 %s: 位置=%.3f, 速度=%.3f, 力矩=%.3f",
                    msg->name[i].c_str(),
                    msg->position[i],
                    msg->velocity[i],
                    msg->effort[i]);
    }
    
    // 如果需要，可以在这里处理数据后发送控制命令
    // 例如：生成要发送给电机的指令
    // auto cmd_msg = std_msgs::msg::Float64MultiArray();
    // cmd_msg.data = { ... };
    // motor_cmd_pub_->publish(cmd_msg);
}

void ControllerBridgeNode::command_callback(const geometry_msgs::msg::Twist::SharedPtr msg) 
{
    // 提取键盘发来的数据
    double linear_x = msg->linear.x;
    double angular_z = msg->angular.z;

    // 逻辑分类处理示例
    RCLCPP_INFO(this->get_logger(), "收到指令: 线速度=%.2f, 角速度=%.2f", linear_x, angular_z);

    // 编写后续解算逻辑 ---
    // 例如：计算左右轮速度，或者判断是否需要切换步态
    if (linear_x > 0) 
    {
        // 执行前进相关的算法逻辑
    }
}


// 主函数
int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControllerBridgeNode>());
    rclcpp::shutdown();
    return 0;
}