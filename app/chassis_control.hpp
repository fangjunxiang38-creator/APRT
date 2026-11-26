// chassis_control.hpp
#pragma once

#include "status.hpp"
#include "types.hpp"

namespace robot::chassis_control
{
pnxtypes::status init();
bool poll_feedback();
// 已许可的驱动指令无法执行时返回 false。
bool update(const Remote& remote, bool drive_allowed);
} // namespace robot::chassis_control