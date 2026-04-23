#ifndef __ALG_SLOPE_HPP
#define __ALG_SLOPE_HPP



namespace algorithm 
{

class Class_Slope_Filter {
public:
    /**
     * @brief 构造函数
     * @param max_step 每次调用 update 时允许的最大变化绝对值 (斜率控制)
     * @param initial_value 初始输出值，默认为 0.0
     */
    Class_Slope_Filter(float __max_step_position= 0.0f, float __max_step_velocity= 0.0f, float initial_value = 0.0f);

    ~Class_Slope_Filter() = default;

    /**
     * @brief 更新并获取平滑后的输出值
     * @param target 期望的目标输入值 (如目标速度/力矩)
     * @return 经过斜坡限制后的实际输出值
     */
    float update(float target_position,float target_velocity);

    /**
     * @brief 重置当前值 (通常在电机急停或重新使能时调用)
     * @param value 重置的目标状态
     */
    void reset(float value);

    /**
     * @brief 动态调整步长
     * @param max_step 新的最大步长
     */
    void setMaxStep(float __max_step_position, float __max_step_velocity);

private:
    float max_step_position;
    float max_step_velocity;
    float current_value_;
};

} // namespace algorithm

#endif