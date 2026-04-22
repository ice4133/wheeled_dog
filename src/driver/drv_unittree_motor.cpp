/* 头文件*/
#include "wheeled_dog/driver/drv_unittree_motor.hpp"
#include <chrono>



/*  命名空间 */
using namespace std::chrono_literals;
using std::placeholders::_1;


/*
*
* @brief 电机控制器节点构造函数：节点启动
*
*/
MotorControllerNode::MotorControllerNode(): Node("motor_controller_node"),serial_("/dev/ttyUSB0")
{
  //初始化数组
  target_positions_.resize(12, 0.0);
  actual_positions_.resize(12, 0.0);
  actual_velocities_.resize(12, 0.0);
  actual_efforts_.resize(12, 0.0);

  // 2. 初始化发布者：向上层发送机器人的实际关节状态 (标准ROS2消息)
    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

  // 3. 初始化订阅者：接收上层的控制指令 (这里用Float64数组简化，实际可用更复杂的消息)
    cmd_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "joint_cmds", 10, std::bind(&MotorControllerNode::cmd_callback, this, _1));

  // 4. 初始化控制循环定时器：500Hz (2ms)
    timer_ = this->create_wall_timer(2ms, std::bind(&MotorControllerNode::TIM_PeriodElapsedCallback, this));
    
  RCLCPP_INFO(this->get_logger(), "四轮足电机控制节点已启动，运行频率: 500Hz");    


}

/*
*
* @brief 电机控制器节点析构函数：节点关闭
*
*/
MotorControllerNode::~MotorControllerNode()
{

  RCLCPP_INFO(this->get_logger(), "节点关闭，执行安全停机...");
}



/*
*
* @brief 电机控制器节点订阅回调函数：当收到上层计算好的动作指令时触发
*
*/
void MotorControllerNode::cmd_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) 
{
    // 校验数据长度是否为12
    if (msg->data.size() == 12)
    {
        // 只做一件事：更新内存中的目标值
        for (size_t i = 0; i < 12; ++i) {
            target_positions_[i] = msg->data[i];
        }
    } 
    else 
    {
        RCLCPP_WARN(this->get_logger(), "收到错误的控制指令长度!");
    }
}



/*
*
* @brief 电机控制器节点控制循环函数：持续执行控制逻辑
*
*/
void MotorControllerNode::TIM_PeriodElapsedCallback() 
{
  //发送指令并获取数据
    exchange_motor_data();

    // 第二步：将刚刚拿到的实际硬件状态发布给 ROS2 的上层算法
    auto state_msg = sensor_msgs::msg::JointState();
    state_msg.header.stamp = this->now();
    
    for (int i = 0; i < 12; ++i) {
        state_msg.name.push_back("motor_" + std::to_string(i));
        state_msg.position.push_back(actual_positions_[i]);
        state_msg.velocity.push_back(actual_velocities_[i]);
        state_msg.effort.push_back(actual_efforts_[i]);
    }
    joint_state_pub_->publish(state_msg);
}

/*
*
* @brief 电机控制器节点与硬件交互函数：发送控制指令并接收状态反馈
*
*/
void MotorControllerNode::exchange_motor_data() 
{
    // 构造发送数据包 (提取自 main.cpp)
    MotorCmd    cmd;
    MotorData   data;

    cmd.motorType = MotorType::GO_M8010_6;
    cmd.id    = 0;
    cmd.mode  = 1;    // 1代表运行
    cmd.K_P   = 0.02;
    cmd.K_W   = 0.0;
    cmd.Pos   = target_positions_[0]; // 这里只控制第一个电机，实际可扩展到12个
    cmd.W     = 0.0;
    cmd.T     = 0.0;

    // 与硬件进行通信，发送指令并接收状态
    serial_.sendRecv(&cmd, &data);

    // 更新内存中的实际状态 (提取自 main.cpp)
    actual_positions_[0] = data.Pos; // 这里只更新第一个电机，实际可扩展到12个
    actual_velocities_[0] = data.W;
    actual_efforts_[0] = data.T;
}

//主函数
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MotorControllerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}