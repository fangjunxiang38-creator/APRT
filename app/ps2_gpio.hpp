#pragma once

#include "config.hpp"
#include "types.hpp"

namespace robot
{
class Ps2Gpio
{
public:
    bool init(const bsp::gpio::config (&pins)[4], uint32_t bit_delay_us);
    bool poll(Remote& result);

private:
    bool exchange(const uint8_t (&tx)[9], uint8_t (&rx)[9]);

    bsp::gpio::config pins_[4]{};
    uint32_t delay_us_ = 5;
    uint8_t setup_step_ = 0;
    uint8_t missed_ = 0;
    bool initialized_ = false;
};
} // namespace robot