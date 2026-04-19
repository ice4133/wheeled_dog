#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
// 引入第三方库的头文件
#include "serialPort/SerialPort.h" 


using namespace std::chrono_literals;

class MotorControllerNode : public rclcpp::Node
{
public:
  // 1. 构造函数：节点启动（相当于插上对讲机电源）
  MotorControllerNode() 
  : Node("motor_controller_node"), 
    serial_("/dev/ttyUSB0") // 在初始化列表中打开串口
  {
    RCLCPP_INFO(this->get_logger(), "串口已连接，开始发送运动指令...");
    
    // 创建一个高频定时器，替代 example 里的 while(true) 和 usleep(200)
    // 这里设置 2ms 执行一次 (500Hz)
    timer_ = this->create_wall_timer(
      2ms, std::bind(&MotorControllerNode::timer_callback, this));
  }

  // 2. 析构函数：按下 Ctrl+C 触发（对应 stop.cpp 的逻辑）
  ~MotorControllerNode()
  {
    RCLCPP_WARN(this->get_logger(), "接收到 Ctrl+C，正在发送急停指令...");
    
    // 构造停止指令数据包 (提取自 stop.cpp)
    MotorCmd    stop_cmd;
    MotorData   stop_data;

    stop_cmd.motorType = MotorType::GO_M8010_6;
    stop_cmd.id    = 0;
    stop_cmd.mode  = 0;    // 核心：0代表停止
    stop_cmd.K_P   = 0.0;
    stop_cmd.K_W   = 0.00;
    stop_cmd.Pos   = 0.0;
    stop_cmd.W     = 0;
    stop_cmd.T     = 0.0;

    serial_.sendRecv(&stop_cmd, &stop_data);      

    // 发送最后一条停止指令

    
    RCLCPP_INFO(this->get_logger(), "急停指令发送完毕，安全退出。");
  }

private:
  // 3. 定时器回调：持续发送运动控制（对应 main.cpp 里的 while 循环体）
  int cnt = 0; // 用于记录发送次数，验证定时器是否正常工作
  float Pos = 0.0; // 用于动态调整位置指令，验证运动效果 
  void timer_callback()
  {
    MotorCmd    move_cmd;
    MotorData   move_data;

    if(++cnt % 100 == 0) 
    { 
      this->Pos += 0.1*6.33; // 每 100 次增加位置指令，验证运动效果

      if(this->Pos > 3.14*6.33*10) // 超过一定位置后重置，验证循环运动效果
      {
        this->Pos = 3.14*6.33*10;
      }
    }
    // 构造运动指令数据包 (提取自 main.cpp)
    move_cmd.motorType = MotorType::GO_M8010_6;
    move_cmd.id    = 0;
    move_cmd.mode  = 1;    // 核心：1代表运行
    move_cmd.K_P   = 0.02;
    move_cmd.K_W   = 0.0;
    move_cmd.Pos   = this->Pos;
    move_cmd.W     = 0.0;
    move_cmd.T     = 0.0;

    // 发送数据
    serial_.sendRecv(&move_cmd, &move_data);
  }
  
  // 类成员变量
  SerialPort serial_; // 保持串口连接的生命周期
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotorControllerNode>());
  rclcpp::shutdown();
  return 0;
}