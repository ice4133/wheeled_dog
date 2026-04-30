#pragma once

#include <string>

// 1. 状态定义
enum class DogState {
    PRONE,          // 趴下
    STAND_LOCKED,   // 站立自锁
    MOVING          // 行进
};

// 2. 指令定义
enum class DogCommand {
    CMD_STAND_UP,   // 起身
    CMD_MOVE,       // 移动
    CMD_STOP,       // 停止
    CMD_LAY_DOWN    // 趴下
};

// 3. 状态机类声明
class Class_FSM {
private:
    DogState currentState;

public:
    Class_FSM();

    // 核心接口
    void handleCommand(DogCommand cmd);

    // 获取当前状态（用于外部 ROS 2 节点读取并发布状态话题）
    DogState getCurrentState() const;
    std::string getStateString() const;
};