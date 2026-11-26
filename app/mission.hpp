#pragma once

#include "types.hpp"

namespace robot
{
class Mission
{
public:
    bool allow_drive(const Remote& remote, bool chassis_ready);
    void disarm_drive();

private:
    bool r1_released_ = false;
};
} // namespace robot
