#pragma once

#include <array>
#include <cstdint>

namespace robot
{

struct Motion
{
    float vx = 0;
    float vy = 0;
    float wz = 0;
};

struct Remote
{
    float lx = 0;
    float ly = 0;
    float rx = 0;
    float ry = 0;
    uint16_t buttons = 0;
    uint32_t received_ms = 0;
    bool valid = false;
};

inline bool fresh(bool valid, uint32_t received_ms, uint32_t now_ms, uint32_t timeout_ms)
{
    return valid && uint32_t(now_ms - received_ms) <= timeout_ms;
}

// 把 value 限幅到 [-bound, bound]。bound 必须为正
// 【用途】视觉对准的输出限速、航向保持的角速度限幅
inline float clamp(float value, float bound)
{
    if (value > bound)
    {
        return bound;
    }
    if (value < -bound)
    {
        return -bound;
    }
    return value;
}

} // namespace robot