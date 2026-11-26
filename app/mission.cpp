#include "mission.hpp"

#include "remoter.hpp"

namespace robot
{
bool Mission::allow_drive(const Remote& remote, bool chassis_ready)
{
    constexpr uint16_t r1_mask = 1U << 11;
    if (!chassis_ready || !remote.valid || !remotes::enabled())
    {
        disarm_drive();
        return false;
    }

    // 上电、失联或执行失败后，从有效输入重新确认 R1 已松开。
    if ((remote.buttons & r1_mask) == 0U)
    {
        r1_released_ = true;
        return false;
    }

    return r1_released_;
}

void Mission::disarm_drive()
{
    r1_released_ = false;
}
} // namespace robot
