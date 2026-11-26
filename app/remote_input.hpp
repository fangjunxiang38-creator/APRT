// remote_input.hpp
#pragma once

#include "status.hpp"
#include "types.hpp"

namespace robot::remote_input
{
pnxtypes::status init();
void update(Remote& remote);

struct ServoJog
{
    uint32_t target_us;
    bool allow_move;
};

ServoJog servo_jog(const Remote& remote);
} // namespace robot::remote_input