#pragma once

#include "types.hpp"

namespace robot
{
// 底盘的几何参数，全部用国际单位制。
// 【单位】radius_m 轮半径(m)；half_length_m 轴距的一半(m)；half_width_m 轮距的一半(m)。
// 【顺序】四个轮子固定按"左前、右前、左后、右后"排列，和配置里的 motor_ids 一一对应。
struct Geometry
{
    float radius_m, half_length_m, half_width_m;
};

// 【参数】motion 车体速度(vx m/s, vy m/s, wz rad/s)；geometry 几何参数；
//         limit_rad_s 每个轮子的转速上限（输出轴 rad/s，配置里的 max_wheel_speed）。
// 【输出】result 四个轮子的目标角速度(rad/s)，顺序同 Geometry 注释。
// 【返回】false 表示输入非法（含 NaN/Inf、几何参数非正、限速非正），此时 result 已清零。
bool wheel_speeds(const Motion& motion, const Geometry& geometry, float limit_rad_s,
    std::array<float, 4>& result);

// 对速度指令做加速度限制，返回"本周期允许走到的速度"。
// 【参数】current 上一周期指令；target 本周期期望；linear_accel 线加速度上限(m/s^2)；
//         angular_accel 角加速度上限(rad/s^2)；dt 控制周期(s)。
// 【返回】限制后的新指令；输入非法时返回全 0。
Motion ramp(const Motion& current, const Motion& target, float linear_accel, float angular_accel,
    float dt);

// 计算两个航向角的最短差值（目标 - 实测），结果落在 [-π, π]。
// 【单位】弧度。返回正值表示需要逆时针转过去。
// 【为什么需要】角度是周期量，359° 和 1° 只差 2°，直接相减会得到 -358°，
//               PID 会朝错误方向猛转一圈。
float heading_error(float target_rad, float measured_rad);
} // namespace robot
