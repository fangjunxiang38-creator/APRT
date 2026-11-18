// 算法顺序：车体速度按四麦轮几何分配，再同比缩放超限轮速，保持运动方向比例。
// ramp 限制平移向量和转向的变化率；停车许可撤销时 runtime 可直接清零而不缓降。
// 阅读要点：重点看 wheel_speeds 里的分配公式和"等比例缩放"，
//           以及 ramp 为什么对 (dx,dy) 取模长限幅、对 wz 单独限幅。

#include "chassis.hpp"

#include <algorithm>
#include <cmath>

namespace robot
{
bool wheel_speeds(const Motion& m, const Geometry& g, float limit, std::array<float, 4>& result)
{
    // 先清零：任何一步失败都返回 "四轮 0 速"
    result = {};
    // 防御式校验。控制链路里只要出现 NaN 就会一路污染到电机电流，
    // 所以在唯一的入口处把非法输入（NaN/Inf、零/负半径、零/负限速）全部挡掉。
    // 几何参数为 0 通常意味着"配置还没填"，此时不应该让车动。
    if (!std::isfinite(m.vx) || !std::isfinite(m.vy) || !std::isfinite(m.wz) ||
        !std::isfinite(g.radius_m) || g.radius_m <= 0 || !std::isfinite(g.half_length_m) ||
        g.half_length_m <= 0 || !std::isfinite(g.half_width_m) || g.half_width_m <= 0 ||
        !std::isfinite(limit) || limit <= 0)
    {
        return false;
    }
    // 自转项
    const float turn = (g.half_length_m + g.half_width_m) * m.wz;
    // 四个轮子的线速度（m/s），再除以半径换成输出轴角速度（rad/s）：
    result = {(m.vx - m.vy - turn) / g.radius_m, (m.vx + m.vy + turn) / g.radius_m,
        (m.vx + m.vy - turn) / g.radius_m, (m.vx - m.vy + turn) / g.radius_m};
    // 等比例缩放：先找出"最大的那个轮速"，仍然以 limit 作为下限起点。
    float maximum = limit;
    for (float value : result)
    {
        // 分配结果仍然可能是 Inf（例如半径极小），再检查一次。
        if (!std::isfinite(value))
        {
            result = {};
            return false;
        }
        maximum = std::max(maximum, std::fabs(value));
    }
    for (float& value : result)
    {
        value *= limit / maximum;
    }
    return true;
}

Motion ramp(const Motion& current, const Motion& target, float linear, float angular, float dt)
{
    if (!std::isfinite(dt) || dt <= 0 || !std::isfinite(linear) || linear < 0 ||
        !std::isfinite(angular) || angular < 0)
    {
        return {};
    }
    // 平移分量按"向量"限幅：把 (dx,dy) 看成平面上的位移，
    // 长度不超过 linear*dt 就原样走，超过就沿同方向缩短。
    // 【效果】斜着走和直着走的加速度上限一致，不会出现"斜走更快"。
    float dx = target.vx - current.vx;
    float dy = target.vy - current.vy;
    const float length = std::hypot(dx, dy);
    const float allowed = linear * dt;
    if (length > allowed && length > 0)
    {
        dx *= allowed / length;
        dy *= allowed / length;
    }
    // 旋转分量单独限幅：角加速度是独立的物理约束（电机扭矩/轮胎附着）
    return {current.vx + dx, current.vy + dy,
        current.wz + clamp(target.wz - current.wz, angular * dt)};
}

float heading_error(float target, float measured)
{
    // std::remainder(x, 2π) 返回 (-π, π] 区间内的余数，正好是"最短转向角"。
    return std::remainder(target - measured, 6.283185307f);
}
} // namespace robot
