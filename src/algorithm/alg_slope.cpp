#include "wheeled_dog/algorithm/alg_slope.hpp"
#include <algorithm> // 引入 std::clamp (C++17标准)

namespace algorithm 
{

void Class_Slope_Filter::Init(float __start_pos, float __max_speed, float __delta_t)
{
    q_current_des = __start_pos;
    v_max = __max_speed;
    dt = __delta_t;    
}
float Class_Slope_Filter::update(float target_position)
{
    float diff = target_position - q_current_des;
            
            // 2. 计算当前周期允许的最大步长
            float max_step = v_max * dt;

            // 3. 逻辑判断与状态更新
            if (std::abs(diff) <= max_step) 
            {
                // 距离已经足够小，一步到位
                q_current_des = target_position;
            } 
            else if (diff > 0.0f) 
            {
                // 目标在正方向，按最大步长累加
                q_current_des += max_step;
            } 
            else 
            {
                // 目标在反方向，按最大步长递减
                q_current_des -= max_step;
            }

            return q_current_des;

}




} // namespace algorithm