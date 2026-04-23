#include "wheeled_dog/algorithm/alg_slope.hpp"
#include <algorithm> // 引入 std::clamp (C++17标准)

namespace algorithm 
{

Class_Slope_Filter::Class_Slope_Filter(float __max_step_position, float __max_step_velocity, float initial_value)
    : max_step_position(__max_step_position), max_step_velocity(__max_step_velocity), current_value_(initial_value) 

{
    
}

float Class_Slope_Filter::update(float target_position,float target_velocity)
{
    if(target_velocity ==0.0f && target_position != 0.0f)
    {
        // 计算目标值与当前值的差值
        float error = target_position - current_value_;
        
        // 限制单次变化量在 [-max_step_, max_step_] 之间
        float step = std::clamp(error, -max_step_position, max_step_position);
        
        // 更新当前状态
        current_value_ += step;
        
        return current_value_;
    }
    else
    {
        // 计算目标值与当前值的差值
        float error = target_velocity - current_value_;
        
        // 限制单次变化量在 [-max_step_, max_step_] 之间
        float step = std::clamp(error, -max_step_velocity, max_step_velocity);
        
        // 更新当前状态
        current_value_ += step;
        
        return current_value_;
    }

}

void Class_Slope_Filter::reset(float value) 
{
    current_value_ = value;
}


/*
*
* @brief 动态调整步长
* @param max_step 新的最大步长
*/

void Class_Slope_Filter::setMaxStep(float __max_step_position, float __max_step_velocity) 
{
    max_step_position = __max_step_position;
    max_step_velocity = __max_step_velocity;
}

} // namespace algorithm