#pragma once

#include <cstdint>

namespace robot::servo_control
{
void update(uint32_t target_us, bool allow_move);
}