#ifndef __ALG_SLOPE_HPP
#define __ALG_SLOPE_HPP



namespace algorithm 
{

class Class_Slope_Filter 
{
public:


    float update(float target_position);

    void Init(float __start_pos, float __max_speed, float __delta_t);

private:
    float q_current_des = 0.0f;
    float v_max = 0.0f;
    float dt = 0.0f;
};

} // namespace algorithm

#endif