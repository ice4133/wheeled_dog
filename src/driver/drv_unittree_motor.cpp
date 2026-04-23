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
  // 初始化电机
    Motor_Init();


  // 初始化发布者：向上层发送机器人的实际关节状态 (标准ROS2消息,先不自己写)
    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

  // 初始化订阅者：接收上层的控制指令 (这里用Float64数组简化，实际可用更复杂的消息)
    cmd_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "joint_cmds", 10, std::bind(&MotorControllerNode::Cmd_Topic_Callback, this, _1));

  // 初始化控制循环定时器
    timer_ = this->create_wall_timer(5ms, std::bind(&MotorControllerNode::TIM_PeriodElapsedCallback, this));
    
  RCLCPP_INFO(this->get_logger(), "四轮足电机控制节点已启动，运行频率: 500Hz");    


}

/*
*
* @brief 电机控制器节点析构函数：节点关闭
*
*/
MotorControllerNode::~MotorControllerNode()
{
  MotorCmd    cmd;
  cmd.motorType = MotorType::GO_M8010_6;
  for(int i = 0; i < MOTOR_COUNT; ++i)
  {
    cmd.id = i;
    cmd.mode = 0;
    cmd.K_P = 0.0;
    cmd.K_W = 0.0;
    cmd.Pos = 0.0;
    cmd.W = 0.0;
    cmd.T = 0.0;
    serial_.sendRecv(&cmd, &unittree_motor_data_vector_[i].data);
  }
  RCLCPP_INFO(this->get_logger(), "节点关闭，执行安全停机...");
}

/*
*
* @brief 电机角度初始化函数：在节点启动时设置初始状态
*
*/
void MotorControllerNode::Motor_Init()
{
  unittree_motor_data_vector_.resize(MOTOR_COUNT); // 预分配12个电机的数据结构

  unittree_motor_data_vector_[0].target_position = 0.0; // 初始位置
  unittree_motor_data_vector_[1].target_position = 0.0; // 初始位置
}


/*
* @brief 电机控制器节点订阅回调函数：当收到上层计算好的动作指令时触发
*
*/
void MotorControllerNode::Cmd_Topic_Callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) 
{
    // 校验数据长度是否为12
    if (msg->data.size() == 12)
    {
        // 只做一件事：更新内存中的目标值
        for (size_t i = 0; i < 12; ++i) 
        {
            unittree_motor_data_vector_[i].target_position = msg->data[i];
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


    auto start_time = std::chrono::high_resolution_clock::now();  
  //发送指令并获取数据
    #ifdef DEMO
    exchange_motor_data();
    #endif
    #ifdef TEST
    exchange_motor_data_test();
    #endif

    // 第二步：将刚刚拿到的实际硬件状态发布给 ROS2 的上层算法
    // auto state_msg = sensor_msgs::msg::JointState();
    // state_msg.header.stamp = this->now();
    
    // for (int i = 0; i < MOTOR_COUNT; ++i) 
    // {
    //     state_msg.name.push_back("motor_" + std::to_string(i));
    //     state_msg.position.push_back(unittree_motor_data_vector_[i].actual_position);
    //     state_msg.velocity.push_back(unittree_motor_data_vector_[i].actual_velocity);
    //     state_msg.effort.push_back(unittree_motor_data_vector_[i].actual_effort);
    // }
    // joint_state_pub_->publish(state_msg);

// --- 2. 记录结束时间并计算差值 ---
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    
    RCLCPP_INFO(this->get_logger(), "Execution Time: %ld us", duration.count());    
}

/*
*
* @brief 电机控制器节点与硬件交互函数：发送控制指令并接收状态反馈
*
*/
void MotorControllerNode::exchange_motor_data() 
{
    //  𝜏 = 𝜏𝑓𝑓 + 𝑘𝑝 × (𝑝𝑑𝑒𝑠 − 𝑝) + 𝑘𝑑 × (𝜔𝑑𝑒𝑠 − 𝜔)
    // 构造发送数据包 
    MotorCmd    cmd;

    cmd.motorType = MotorType::GO_M8010_6; 

    // 位置环，腿部电机ID 0~7
    for(int i =0;i<8;++i)
    {
        cmd.id = i;
        cmd.mode = 1;
        cmd.K_P   = K_P;
        cmd.K_W   = 0.0;
        cmd.Pos   = slope_filter.update(unittree_motor_data_vector_[i].target_position,0.0f);
        cmd.W     = 0.0;
        cmd.T     = 0.0;         

        // 与硬件进行通信，发送指令并接收状态
        serial_.sendRecv(&cmd, &unittree_motor_data_vector_[i].data);
        
        // 更新内存中的实际状态 
        unittree_motor_data_vector_[i].actual_position = unittree_motor_data_vector_[i].data.Pos; 
        unittree_motor_data_vector_[i].actual_velocity = unittree_motor_data_vector_[i].data.W;
        unittree_motor_data_vector_[i].actual_effort = unittree_motor_data_vector_[i].data.T;      
    }

    // 速度环，轮毂电机ID 8~11
    for(int i = 8;i<12;++i)
    {
        cmd.id = i;
        cmd.mode = 1;
        cmd.K_P   = 0.0;
        cmd.K_W   = K_W;
        cmd.Pos   = 0.0; 
        cmd.W     = slope_filter.update(0.0f, unittree_motor_data_vector_[i].target_velocity);
        cmd.T     = 0.0;
        
        // 与硬件进行通信，发送指令并接收状态
        serial_.sendRecv(&cmd, &unittree_motor_data_vector_[i].data);
        
        // 更新内存中的实际状态 
        unittree_motor_data_vector_[i].actual_position = unittree_motor_data_vector_[i].data.Pos;
        unittree_motor_data_vector_[i].actual_velocity = unittree_motor_data_vector_[i].data.W;
        unittree_motor_data_vector_[i].actual_effort = unittree_motor_data_vector_[i].data.T;         
    }
}
#ifdef TEST
void MotorControllerNode::exchange_motor_data_test()
{
  // if(++test % 100 ==0)
  // {
  //   unittree_motor_data_vector_[0].target_position+=0.1*6.33;
  //   unittree_motor_data_vector_[1].target_position+=0.1*6.33;
  //   for(int i=0;i<2;++i)
  //   {
  //     cmd.id = i;
  //     cmd.mode = 1;
  //     cmd.K_P   = K_P;
  //     cmd.K_W   = 0.0;
  //     cmd.Pos   = unittree_motor_data_vector_[i].target_position;
  //     cmd.W     = 0.0; 
  //     cmd.T     = 0.0; 

  //     serial_.sendRecv(&cmd, &unittree_motor_data_vector_[i].data);
  //   }
  //   }
  //   if(test>60000)
  //   {
  //     test = 0;
  //   }
    // for (int  i = 0; i < MOTOR_COUNT; i++)
    // {
    //   send_cmds_vec_[i].motorType = MotorType::GO_M8010_6;
    //   send_cmds_vec_[i].id = i;
    //   send_cmds_vec_[i].mode = 1;
    //   send_cmds_vec_[i].K_P   = 0.0;
    //   send_cmds_vec_[i].K_W   = K_W;
    //   send_cmds_vec_[i].Pos   = 0.0;
    //   send_cmds_vec_[i].W     = 1.57*6.33;
    //   send_cmds_vec_[i].T     = 0.0;
    // }
    
    // serial_.sendRecv(send_cmds_vec_,recv_datas_vec_);

    // MotorCmd    cmd;
    // cmd.motorType = MotorType::GO_M8010_6;
    // for(int i =0;i<2;++i)
    // {
    //     cmd.id = i;
    //     cmd.mode = 1;
    //     cmd.K_P   = 0.0;
    //     cmd.K_W   = K_W;
    //     cmd.Pos   = 0.0;
    //     cmd.W     = 1.57*6.33;
    //     cmd.T     = 0.0;

    //     serial_.sendRecv(&cmd, &unittree_motor_data_vector_[i].data);
    // }
    MotorCmd    cmd;
    cmd.motorType = MotorType::GO_M8010_6;
    cmd.id = 0;
    cmd.mode = 1;
    cmd.K_P   = 0.0;
    cmd.K_W   = K_W;
    cmd.Pos   = 0.0;
    cmd.W     = 1.57*6.33;
    cmd.T     = 0.0;
    serial_.sendRecv(&cmd, &unittree_motor_data_vector_[0].data);
//     # 查看当前的延迟设置，默认通常是 16
// cat /sys/class/tty/ttyUSB0/device/latency_timer

// # 将其修改为 1 (这是最小值)
// sudo sh -c 'echo 1 > /sys/class/tty/ttyUSB0/device/latency_timer'

}

#endif
//主函数
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MotorControllerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}