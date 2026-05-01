/* 头文件*/
#include "wheeled_dog/driver/drv_unittree_motor.hpp"
#include <chrono>


/* 线程调度头文件*/
#include <sched.h>
#include <pthread.h>

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

  Motor_Init();

  joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

  // 初始化订阅者：接收上层的控制指令 (这里用Float64数组简化，实际可用更复杂的消息)
  cmd_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "joint_cmds", 10, std::bind(&MotorControllerNode::Cmd_Topic_Callback, this, _1));

  // 初始化控制循环定时器
  timer_ = this->create_wall_timer(20ms, std::bind(&MotorControllerNode::TIM_PeriodElapsedCallback, this));
    
  RCLCPP_INFO(this->get_logger(), "四轮足电机控制节点已启动，运行频率: 50Hz");    


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
  
  class_fsm_controller.handleCommand(DogCommand::CMD_LAY_DOWN); // 切换状态机到趴下状态，确保安全
  RCLCPP_INFO(this->get_logger(), "节点关闭，执行安全停机...");
}

/*
* @brief 电机角度初始化函数：在节点启动时设置初始状态
*
*/
void MotorControllerNode::Motor_Init()
{
  unittree_motor_data_vector_.resize(MOTOR_COUNT); // 预分配12个电机的数据结构

  unittree_motor_data_vector_[0].K_P = 0.6;
  unittree_motor_data_vector_[1].K_P = 0.6;
  unittree_motor_data_vector_[2].K_P = 0.6;
  unittree_motor_data_vector_[3].K_P = 0.6;
  unittree_motor_data_vector_[4].K_P = 0.75;
  unittree_motor_data_vector_[5].K_P = 0.75;
  unittree_motor_data_vector_[6].K_P = 0.75;
  unittree_motor_data_vector_[7].K_P = 0.75;
  unittree_motor_data_vector_[0].K_W = 0.05;
  unittree_motor_data_vector_[1].K_W = 0.05;
  unittree_motor_data_vector_[2].K_W = 0.05;
  unittree_motor_data_vector_[3].K_W = 0.05;
  unittree_motor_data_vector_[4].K_W = 0.05;
  unittree_motor_data_vector_[5].K_W = 0.05; 
  unittree_motor_data_vector_[6].K_W = 0.05;
  unittree_motor_data_vector_[7].K_W = 0.05;
  unittree_motor_data_vector_[0].target_position = 8.78;
  unittree_motor_data_vector_[1].target_position = 7.66;
  unittree_motor_data_vector_[2].target_position = -7.58;
  unittree_motor_data_vector_[3].target_position = -1.16; 
  unittree_motor_data_vector_[4].target_position = 10.09; 
  unittree_motor_data_vector_[5].target_position = 7.10; 
  unittree_motor_data_vector_[6].target_position = -3.99; 
  unittree_motor_data_vector_[7].target_position = -0.32; 


  unittree_motor_data_vector_[0].slope_filter.Init(0.23f,25.0f,0.01f);
  unittree_motor_data_vector_[1].slope_filter.Init(5.61f,10.0f,0.01f);
  unittree_motor_data_vector_[2].slope_filter.Init(1.31f,25.0f,0.01f);
  unittree_motor_data_vector_[3].slope_filter.Init(0.85f,10.0f,0.01f);    
  unittree_motor_data_vector_[4].slope_filter.Init(1.26f,25.0f,0.01f);
  unittree_motor_data_vector_[5].slope_filter.Init(4.59f,10.0f,0.01f);
  unittree_motor_data_vector_[6].slope_filter.Init(4.91f,25.0f,0.01f);
  unittree_motor_data_vector_[7].slope_filter.Init(1.61f,10.0f,0.01f);

  class_fsm_controller.handleCommand(DogCommand::CMD_STAND_UP); // 切换状态机到站立状态，准备接受运动指令
}


/*
* @brief 电机控制器节点订阅回调函数：当收到上层计算好的动作指令时触发
*
*/
void MotorControllerNode::Cmd_Topic_Callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) 
{
    (void)msg;
    int fsm_state_cmd = 0;
    Alive_Flag+=1;
    if(Alive_Flag == Pre_Alive_Flag)
    {
      fsm_state_cmd = 2; // 2 代表断联，进入停止状态
    }
    else 
    {
      fsm_state_cmd = 1; // 1 代表正常通信，进入移动状态
      //数据处理：将上层发来的指令解析并存入电机数据结构中
    }
    Update_Fsm_State(fsm_state_cmd);
    Pre_Alive_Flag = Alive_Flag;
}



/*
*
* @brief 电机控制器节点控制循环函数：持续执行控制逻辑
*
*/
void MotorControllerNode::TIM_PeriodElapsedCallback() 
{


     //auto start_time = std::chrono::high_resolution_clock::now();  
  //发送指令并获取数据
    if(class_fsm_controller.getCurrentState() == DogState::MOVING)
    {
      Inverse_Kinematics_Calculation();          
    }
    Update_Leg_Data();
    Update_Wheel_Data();

    Rs485_Send_Data();


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
     //auto end_time = std::chrono::high_resolution_clock::now();
     //auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    
     //RCLCPP_INFO(this->get_logger(), "Execution Time: %ld us", duration.count());
    // if(++test <= 50)
    // {
    //   RCLCPP_INFO(this->get_logger(), "motor0 %.2f", recv_datas_vec_[8].T);
    //   RCLCPP_INFO(this->get_logger(), "motor1 %.2f", recv_datas_vec_[9].T);
    //   RCLCPP_INFO(this->get_logger(), "motor2 %.2f", recv_datas_vec_[10].T);
    //   RCLCPP_INFO(this->get_logger(), "motor3 %.2f", recv_datas_vec_[11].T);                      
    // }

}

void MotorControllerNode::Update_Fsm_State(int cmd_msg)
{
  DogCommand cmd = static_cast<DogCommand>(cmd_msg);
  class_fsm_controller.handleCommand(cmd);  
}

void MotorControllerNode::Update_Leg_Data()
{
    // 位置环，腿部电机ID 0~7
    for(int i =0;i<8;++i)
    {
        send_cmds_vec_[i].motorType = MotorType::GO_M8010_6;
        send_cmds_vec_[i].id = i;
        send_cmds_vec_[i].mode = 1;
        send_cmds_vec_[i].K_P   = unittree_motor_data_vector_[i].K_P;
        send_cmds_vec_[i].K_W   = unittree_motor_data_vector_[i].K_W;
        send_cmds_vec_[i].Pos   = unittree_motor_data_vector_[i].slope_filter.update(unittree_motor_data_vector_[i].target_position);
        send_cmds_vec_[i].W     = 0.0; 
        send_cmds_vec_[i].T     = 0.0; 
    }
}
void MotorControllerNode::Update_Wheel_Data()
{
  switch (class_fsm_controller.getCurrentState()) 
    {
      case DogState::PRONE:    
          break;
      case DogState::STAND_LOCKED:
        {
          for(int i = 8;i<12;++i)
          {
            send_cmds_vec_[i].motorType = MotorType::GO_M8010_6;
            send_cmds_vec_[i].id = i;
            send_cmds_vec_[i].mode = 1;
            send_cmds_vec_[i].K_P   = 0.0;
            send_cmds_vec_[i].K_W   = 0.0;
            send_cmds_vec_[i].Pos   = 0.0; 
            send_cmds_vec_[i].W     = 0.0;
            send_cmds_vec_[i].T     = 0.01; 
          }
        }
          break;
      case DogState::MOVING:
      {
          for(int i = 8;i<12;++i)
          {
            send_cmds_vec_[i].motorType = MotorType::GO_M8010_6;
            send_cmds_vec_[i].id = i;
            send_cmds_vec_[i].mode = 1;
            send_cmds_vec_[i].K_P = 0.0;
            send_cmds_vec_[i].K_W   = K_W;
            send_cmds_vec_[i].Pos   = 0.0; 
            send_cmds_vec_[i].W     = unittree_motor_data_vector_[i].target_velocity;
            send_cmds_vec_[i].T     = 0.00;             
          }
      }
          break;  
    }

}

void MotorControllerNode::Inverse_Kinematics_Calculation()
{
  double tmp_target_velocity = x_velocity_command * max_wheel_speed;//在这一步进行速度限制
  double tmp_target_angular = z_angular_command * max_angular_speed;

  double v_left = tmp_target_velocity - tmp_target_angular * ( track_width / 2.0 );
  double v_right = tmp_target_velocity + tmp_target_angular * ( track_width / 2.0 );//如果以逆时针为正，那么需要修改

  unittree_motor_data_vector_[8].target_velocity = v_left/wheel_radius*MOTOR_REDUCTION ;
  unittree_motor_data_vector_[9].target_velocity = v_right/wheel_radius*MOTOR_REDUCTION;
  unittree_motor_data_vector_[10].target_velocity = v_left/wheel_radius*MOTOR_REDUCTION;
  unittree_motor_data_vector_[11].target_velocity = v_right/wheel_radius*MOTOR_REDUCTION;   

}
/*
*
* @brief 电机控制器节点与硬件交互函数：发送控制指令并接收状态反馈
*
*/
void MotorControllerNode::Rs485_Send_Data() 
{
  feedback_flag = serial_.sendRecv(send_cmds_vec_,recv_datas_vec_);
}

//主函数
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
// // --- 1. 强制绑定 CPU 核心 (对号入座到 Core 15) ---
   cpu_set_t cpuset;
   CPU_ZERO(&cpuset);       // 清空集合
   CPU_SET(15, &cpuset);    // 将 Core 15 加入集合
  
//   // 将当前主线程绑定到指定的 CPU 集合
   int set_result = pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset);
   if (set_result != 0) 
   {
     RCLCPP_WARN(rclcpp::get_logger("MotorControllerNode"), "Failed to set CPU affinity");
   }

//   // --- 2. 设置实时调度策略 (授予 VIP 特权) ---
   sched_param sch;
   sch.sched_priority = 90; // 优先级范围通常为 1-99，90 属于极高优先级
   // 将当前线程设置为先进先出 (SCHED_FIFO) 的实时策略
  pthread_setschedparam(pthread_self(), SCHED_FIFO, &sch);


  auto node = std::make_shared<MotorControllerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
