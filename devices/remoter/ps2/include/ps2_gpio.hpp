#pragma once

#include "ps2.hpp"

namespace remotes::ps2
{
class gpio
{
public:
    // 引脚顺序为 DATA、CMD、CS、CLK。
    bool init(const bsp::gpio::config (&pins)[4], uint32_t bit_delay_us);
    bool poll(sample& result, float deadzone);

private:
    bool exchange(const uint8_t (&tx)[frame_size], uint8_t (&rx)[frame_size]);

    bsp::gpio::config pins_[4]{};
    uint32_t delay_us_ = 5;
    uint8_t setup_step_ = 0;
    uint8_t missed_ = 0;
    bool initialized_ = false;
};
} // namespace remotes::ps2