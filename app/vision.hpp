#pragma once

#include "attitude.hpp"

#include <cstdint>

namespace robot::vision
{
// 视觉误差。ex_m、ey_m 为米，eyaw_rad 为弧度。
struct Error
{
    float ex_m = 0;
    float ey_m = 0;
    float eyaw_rad = 0;
    uint16_t sequence = 0;
    uint32_t received_ms = 0;
    bool valid = false;
};

pnxtypes::status init();
void update(Error& error, const imu::attitude::state& attitude);
} // namespace robot::vision